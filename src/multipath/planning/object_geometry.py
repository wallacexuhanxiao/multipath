"""Conservative planning envelopes for the scene's authored collision model."""


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

    A convex decomposition can occupy a visual cavity. Its convex envelope
    conservatively covers that space; native continuous physics still decides
    acceptance. Kinematic triangle surfaces keep their original geometry.
    """
    from pxr import Usd,UsdPhysics
    if not hasattr(scene,'planning_object_meshes'):
        scene.planning_object_meshes={};scene.planning_object_geometry={}
    if role not in scene.planning_object_meshes:
        root=scene.world.stage.GetPrimAtPath(scene.objects[role].prim_path)
        approximate={UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
                     for prim in Usd.PrimRange(root) if prim.HasAPI(UsdPhysics.CollisionAPI)}
        kinematic=bool(UsdPhysics.RigidBodyAPI(root).GetKinematicEnabledAttr().Get())
        convex=not kinematic and bool(approximate&{'convexHull','convexDecomposition'})
        import numpy as np
        view=scene.objects[role]._rigid_prim_view._physics_view
        contact=np.asarray(view.get_contact_offsets());rest=np.asarray(view.get_rest_offsets())
        offset=float(np.max(contact[np.isfinite(contact)],initial=0.)) if convex else 0.
        mesh=scene.meshes[role]
        scene.planning_object_meshes[role]=convex_contact_envelope(mesh.convex_hull,offset) if convex else mesh
        scene.planning_object_geometry[role]=dict(physx_approximation=sorted(str(x) for x in approximate),
            kinematic=kinematic,planning_representation='convex_envelope_with_native_contact_offset' if convex else 'source_triangle_mesh',
            native_physics_unchanged=True,exact_native_cooked_geometry=False if convex else None,
            native_contact_offsets_m=contact.tolist(),native_rest_offsets_m=rest.tolist(),
            contact_envelope_inflation_m=offset)
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
