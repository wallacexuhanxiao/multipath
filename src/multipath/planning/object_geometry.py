"""Planning geometry from the scene's authored PhysX collision model."""


def native_convex_meshes(stage,prim):
    """Read each cooked convex piece without changing or advancing physics.

    The PhysX cooking interface uses the collider's authored approximation;
    replacing its decomposition with one hull fills real empty space.
    Vertices are returned in the collider mesh's local coordinates.
    """
    import numpy as np
    import trimesh
    from pxr import UsdUtils,PhysicsSchemaTools
    from omni.physx import get_physx_cooking_interface
    from omni.physx.bindings._physx import PhysxCollisionRepresentationResult
    result=[]
    def receive(status,convexes):
        if status!=PhysxCollisionRepresentationResult.RESULT_VALID:
            raise RuntimeError(f'Native collision geometry unavailable for {prim.GetPath()}: {status}')
        for convex in convexes:
            indices=np.asarray(convex.indices,dtype=int);faces=[]
            for polygon in convex.polygons:
                vertices=indices[polygon.index_base:polygon.index_base+polygon.num_vertices]
                faces.extend((vertices[0],vertices[j],vertices[j+1]) for j in range(1,len(vertices)-1))
            result.append(trimesh.Trimesh(vertices=np.asarray(convex.vertices,dtype=float),
                faces=np.asarray(faces,dtype=int),process=False))
    get_physx_cooking_interface().request_convex_collision_representation(
        stage_id=UsdUtils.StageCache.Get().GetId(stage).ToLongInt(),
        collision_prim_id=PhysicsSchemaTools.sdfPathToInt(str(prim.GetPath())),
        run_asynchronously=False,on_result=receive)
    if not result:raise RuntimeError(f'No native convex pieces for {prim.GetPath()}')
    return result


def convex_contact_envelope(mesh,offset):
    """Offset every convex support plane; retain the complete original hull."""
    import numpy as np
    import trimesh
    from scipy.spatial import ConvexHull,HalfspaceIntersection
    if offset<=0:return mesh
    planes=ConvexHull(mesh.vertices).equations
    planes=planes[np.unique(np.round(planes,10),axis=0,return_index=True)[1]].copy()
    planes[:,3]-=offset*np.linalg.norm(planes[:,:3],axis=1)
    vertices=HalfspaceIntersection(planes,np.mean(mesh.vertices,axis=0)).intersections
    return trimesh.convex.convex_hull(vertices)


def planning_object_mesh(scene,role):
    """Keep dynamic convex cooking out of a visual-triangle-only world.

    Keep all native convex pieces individually; their contact envelopes retain
    authored empty space. Kinematic triangle surfaces keep their geometry.
    """
    from pxr import Usd,UsdGeom,UsdPhysics
    if not hasattr(scene,'planning_object_meshes'):
        scene.planning_object_meshes={};scene.planning_object_geometry={};scene.planning_object_parts={}
    if role not in scene.planning_object_meshes:
        root=scene.world.stage.GetPrimAtPath(scene.objects[role].prim_path)
        colliders=[prim for prim in Usd.PrimRange(root,Usd.TraverseInstanceProxies())
                   if prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.CollisionAPI)
                   and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False]
        approximate={UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() for prim in colliders}
        kinematic=bool(UsdPhysics.RigidBodyAPI(root).GetKinematicEnabledAttr().Get())
        convex=not kinematic and bool(approximate&{'convexHull','convexDecomposition'})
        import numpy as np
        view=scene.objects[role]._rigid_prim_view._physics_view
        contact=np.asarray(view.get_contact_offsets());rest=np.asarray(view.get_rest_offsets())
        offset=float(np.max(contact[np.isfinite(contact)],initial=0.)) if convex else 0.
        mesh=scene.meshes[role]
        parts=[mesh]
        if convex:
            from multipath.execution.mounted_scene import pose_matrix
            import trimesh
            p,q=scene.objects[role].get_world_pose();inverse=np.linalg.inv(pose_matrix(np.r_[p,q]))
            cache=UsdGeom.XformCache();parts=[]
            for prim in colliders:
                matrix=inverse@np.asarray(cache.GetLocalToWorldTransform(prim)).T
                for cooked in native_convex_meshes(scene.world.stage,prim):
                    cooked.apply_transform(matrix)
                    parts.append(convex_contact_envelope(cooked,offset))
            mesh=trimesh.util.concatenate(parts)
        scene.planning_object_parts[role]=parts
        scene.planning_object_meshes[role]=mesh
        scene.planning_object_geometry[role]=dict(physx_approximation=sorted(str(x) for x in approximate),
            kinematic=kinematic,planning_representation='native_convex_pieces_with_contact_offset' if convex else 'source_triangle_mesh',
            native_physics_unchanged=True,exact_native_cooked_geometry=False if convex else None,
            geometry_origin='PhysX cooking representation of authored collider; not active actor shape introspection',
            native_piece_count=len(parts),authored_collision_prims=[str(p.GetPath()) for p in colliders],
            native_contact_offset_tensor_shape_slots=int(contact.shape[-1]),
            source_mesh_bounds_m=scene.meshes[role].bounds.tolist(),planning_mesh_bounds_m=mesh.bounds.tolist(),
            native_contact_offsets_m=contact.tolist(),native_rest_offsets_m=rest.tolist(),
            contact_envelope_inflation_m=offset)
        print('NATIVE_OBJECT_COLLISION_GEOMETRY',role,len(parts),mesh.bounds.tolist(),flush=True)
    return scene.planning_object_meshes[role]


def supported_object_targets(scene,poses,entry):
    """Command-side support projection; measured and source states stay intact.

    Retain the actual object's measured clearance above the declared table
    plane. Preserve source translation parallel to the plane and rotation.
    Original source poses remain the acceptance targets.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation
    from multipath.execution.mounted_scene import pose_matrix
    from multipath.validation.physics import object_tracking_score
    values=np.asarray(poses).copy();single=values.ndim==2
    if single:values=values[None]
    table=scene.table;rotation=Rotation.from_quat(np.asarray(table['pose'])[[4,5,6,3]])
    normal=rotation.apply([0.,0.,1.])
    plane=np.asarray(table['pose'][:3])+rotation.apply([0.,0.,table['dims'][2]/2])
    vertices=np.asarray(scene.meshes['tool'].vertices)
    measured=pose_matrix(entry['objects']['tool']['pose'])
    clearance=float(np.min((vertices@measured[:3,:3].T+measured[:3,3]-plane)@normal))
    shifts=[]
    for pose in values:
        proposed=float(np.min((vertices@pose[:3,:3].T+pose[:3,3]-plane)@normal))
        shift=clearance-proposed
        if not object_tracking_score({'position_m':abs(shift),'rotation_rad':0.})['feasible']:
            raise RuntimeError('Declared table support conflicts with the source interaction boundary')
        pose[:3,3]+=normal*shift;shifts.append(float(shift))
    report=dict(support='table',measured_support_clearance_m=clearance,normal_translation_m=shifts,
                original_source_condition_unchanged=True,object_state_assignment=False)
    return (values[0] if single else values),report
