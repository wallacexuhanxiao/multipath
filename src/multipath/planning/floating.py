"""cuRobo wrist transport and fixed-time free-hand boundary interpolation.

The six virtual joints are a planning parameterization only. PhysX executes
external wrenches on the real floating hand; no virtual joint drives exist there.
"""
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.interpolate import CubicHermiteSpline
from multipath.planning.timing import fixed_budget_path,parameterize_path,boundary_fitted_path


def pose_coordinates(pose,near=None):
    pose=np.asarray(pose);angles=Rotation.from_quat(pose[[4,5,6,3]]).as_euler('ZYX')
    if near is not None:angles+=2*np.pi*np.round((np.asarray(near)[3:]-angles)/(2*np.pi))
    return np.r_[pose[:3],angles]


def coordinates_pose(q):
    q=np.asarray(q);return np.r_[q[:3],Rotation.from_euler('ZYX',q[3:]).as_quat()[[3,0,1,2]]]


def relaxed_hand_path(scene,start,goal,ticks):
    """Connect free pregrasp drive targets inside the original wrist budget.

    Transport of an acquired grasp keeps its existing preload. Changing grasp
    mode still belongs to an Anchor, and final measured boundaries must match.
    """
    from multipath.anchors.floating import hand_command_boundary
    q0=hand_command_boundary(start)
    if goal is None or start['contact']['mode']=='grasped':return np.repeat(q0[None],ticks+1,axis=0)
    q1=hand_command_boundary(goal);duration=ticks*scene.dt
    if np.allclose(q0,q1,rtol=0,atol=1e-10):
        return np.repeat(q0[None],ticks+1,axis=0)
    # FloatingScene commands zero finger velocity feedforward. Measured dq
    # contains contact/drive deflection, not a derivative of that drive target.
    dq0=np.zeros(12) if 'command_hand_q' in start else np.asarray(start['hand_dq'])
    dq1=np.zeros(12) if 'command_hand_q' in goal else np.asarray(goal['hand_dq'])
    curve=CubicHermiteSpline([0.,duration],np.stack((q0,q1)),
                            np.stack((dq0,dq1)))
    time=np.arange(ticks+1)*scene.dt;hand=curve(time)
    speed=np.maximum(np.abs(curve(time,1)),np.vstack((np.zeros(12),np.abs(np.diff(hand,axis=0)/scene.dt))))
    if np.any(hand<scene.hand_limits[:,0]-1e-6) or np.any(hand>scene.hand_limits[:,1]+1e-6) or np.any(speed>scene.hand_velocity*1.0001):
        raise RuntimeError('RELAXED hand boundary cannot fit source-time joint limits')
    return hand


def cover_points(points,cell=.025,faces=None):
    """Conservative cell spheres; mesh inputs cover surface, not its empty box.

    Every triangle is inside its coordinate bounding cells, and every such
    cell is inside the returned sphere. This preserves full face coverage
    without treating empty corners/interior as solid geometry.
    """
    center=points.mean(0);_,_,axes=np.linalg.svd(points-center,full_matrices=False)
    coords=(points-center)@axes.T;lo=coords.min(0);hi=coords.max(0)
    counts=np.maximum(1,np.ceil((hi-lo)/cell).astype(int));width=np.maximum((hi-lo)/counts,1e-12)
    radius=float(np.linalg.norm(width)*.5+.001)
    if faces is None:cells=list(np.ndindex(*counts))
    else:
        triangles=coords[np.asarray(faces,dtype=int)]
        first=np.clip(np.floor((triangles.min(1)-lo)/width).astype(int),0,counts-1)
        last=np.clip(np.floor((triangles.max(1)-lo)/width).astype(int),0,counts-1)
        occupied=[]
        for offset in np.ndindex(*((last-first).max(0)+1)):
            trial=first+offset;occupied.append(trial[np.all(trial<=last,axis=1)])
        cells=np.unique(np.concatenate(occupied),axis=0)
    return [{'center':(center+(lo+(np.asarray(i)+.5)*width)@axes).tolist(),'radius':radius} for i in cells]


def usd_mesh_faces(mesh):
    faces=[];offset=0;indices=np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
    for count in mesh.GetFaceVertexCountsAttr().Get():
        polygon=indices[offset:offset+count]
        faces.extend([[polygon[0],polygon[j],polygon[j+1]] for j in range(1,count-1)])
        offset+=count
    return np.asarray(faces,dtype=int)


def carried_proxy(scene):
    if not hasattr(scene,'carried_collision_proxy'):
        scene.carried_collision_proxy=cover_points(scene.meshes['tool'].vertices,faces=scene.meshes['tool'].faces)
    return scene.carried_collision_proxy


def sphere_world_clearance(pose,spheres,boxes):
    """Minimum signed box clearance and its world translation gradient."""
    pose=np.asarray(pose);rotation=Rotation.from_quat(pose[[4,5,6,3]])
    centers=rotation.apply(np.array([s['center'] for s in spheres]))+pose[:3]
    radii=np.array([s['radius'] for s in spheres]);values={};worst=None
    for role,box in boxes.items():
        b=np.asarray(box['pose']);r=Rotation.from_quat(b[[4,5,6,3]])
        local=r.inv().apply(centers-b[:3]);offset=np.abs(local)-np.asarray(box['dims'])/2
        outside=np.maximum(offset,0);norm=np.linalg.norm(outside,axis=1)
        distance=norm+np.minimum(np.max(offset,axis=1),0)-radii
        i=int(np.argmin(distance));values[role]=float(distance[i])
        if worst is None or distance[i]<worst[0]:
            if norm[i]>1e-12:gradient=np.sign(local[i])*outside[i]/norm[i]
            else:
                axis=int(np.argmax(offset[i]));gradient=np.zeros(3);gradient[axis]=1 if local[i,axis]>=0 else -1
            worst=(float(distance[i]),r.apply(gradient))
    return values,worst


def mesh_box_intersects(vertices,faces,box,closed=False):
    """Triangle/box SAT, with containment for a closed mesh.

    Used to refine a conservative sphere overlap; no distance tolerance or
    collision threshold is relaxed.
    """
    b=np.asarray(box['pose']);half=np.asarray(box['dims'])/2
    local=Rotation.from_quat(b[[4,5,6,3]]).inv().apply(vertices-b[:3])
    all_tri=local[np.asarray(faces,dtype=int)]
    candidate=np.all(all_tri.min(1)<=half,axis=1)&np.all(all_tri.max(1)>=-half,axis=1)
    tri=all_tri[candidate]
    if len(tri):
        edges=np.roll(tri,-1,axis=1)-tri
        axes=[np.cross(edges[:,0],edges[:,1])]
        axes.extend(np.cross(edges[:,j],basis) for j in range(3) for basis in np.eye(3))
        separated=np.zeros(len(tri),dtype=bool)
        for axis in axes:
            projection=np.einsum('nki,ni->nk',tri,axis);radius=np.abs(axis)@half
            separated|=(projection.min(1)>radius)|(projection.max(1)<-radius)
        if np.any(~separated):return True
    if closed:
        # A box completely inside a solid mesh has no surface intersections.
        direction=np.array([1.,np.sqrt(2)/7,np.sqrt(3)/11]);edge1=all_tri[:,1]-all_tri[:,0];edge2=all_tri[:,2]-all_tri[:,0]
        h=np.cross(direction,edge2);det=np.einsum('ni,ni->n',edge1,h);keep=np.abs(det)>1e-12
        inverse=1/det[keep];s=-all_tri[keep,0];u=np.einsum('ni,ni->n',s,h[keep])*inverse
        q=np.cross(s,edge1[keep]);v=q@direction*inverse;t=np.einsum('ni,ni->n',edge2[keep],q)*inverse
        hits=np.sort(t[(u>=0)&(v>=0)&(u+v<=1)&(t>1e-12)])
        crossings=0 if not len(hits) else 1+np.count_nonzero(np.diff(hits)>1e-9)
        if crossings%2:return True
    return False


def boundary_collision_cover(vertices,faces,pose,boxes):
    """Conservative surface spheres without false overlap at a reached entry.

    Start with the usual coarse cells. Only near a collision-free boundary,
    subdivide surface triangles until their bounding spheres also clear it.
    Every triangle remains covered; world obstacles and acceptance stay fixed.
    """
    pose=np.asarray(pose);rotation=Rotation.from_quat(pose[[4,5,6,3]])
    vertices=np.asarray(vertices);faces=np.asarray(faces,dtype=int)
    coarse=cover_points(rotation.inv().apply(vertices-pose[:3]),faces=faces)
    if min(sphere_world_clearance(pose,coarse,boxes)[0].values())>=0:return coarse
    collisions=[name for name,box in boxes.items() if mesh_box_intersects(vertices,faces,box)]
    if collisions:raise RuntimeError(f'Actual mesh intersects a planning obstacle at entry: {collisions}')
    center=rotation.apply(np.asarray([s['center'] for s in coarse]))+pose[:3]
    radius=np.asarray([s['radius'] for s in coarse]);clear=np.ones(len(coarse),dtype=bool)
    for box in boxes.values():
        b=np.asarray(box['pose']);r=Rotation.from_quat(b[[4,5,6,3]])
        offset=np.abs(r.inv().apply(center-b[:3]))-np.asarray(box['dims'])/2
        clear&=np.linalg.norm(np.maximum(offset,0),axis=1)+np.minimum(offset.max(1),0)-radius>=0
    centers=[center[clear]];radii=[radius[clear]];triangles=vertices[faces]
    needed=np.zeros(len(triangles),dtype=bool);lo=triangles.min(1);hi=triangles.max(1)
    for c,s in zip(center[~clear],radius[~clear]):
        needed|=np.all(lo<=c+s,axis=1)&np.all(hi>=c-s,axis=1)
    # Outside the rejected cells' bounding cubes, the retained coarse spheres
    # already cover every surface point. Only neighboring faces need refinement.
    pending=triangles[needed]
    for _ in range(18):
        center=pending.mean(1);radius=np.linalg.norm(pending-center[:,None],axis=2).max(1)+1e-7
        clear=np.ones(len(pending),dtype=bool)
        for box in boxes.values():
            b=np.asarray(box['pose']);r=Rotation.from_quat(b[[4,5,6,3]])
            offset=np.abs(r.inv().apply(center-b[:3]))-np.asarray(box['dims'])/2
            distance=np.linalg.norm(np.maximum(offset,0),axis=1)+np.minimum(offset.max(1),0)-radius
            clear&=distance>=0
        centers.append(center[clear]);radii.append(radius[clear])
        if clear.all():
            local=rotation.inv().apply(np.concatenate(centers)-pose[:3]);sizes=np.concatenate(radii)
            return [dict(center=c.tolist(),radius=float(s)) for c,s in zip(local,sizes)]
        pending=pending[~clear]
        # Bisect the longest triangle edge; each child still covers an exact
        # part of the parent rather than sampling a few unprotected vertices.
        edge=np.linalg.norm(np.roll(pending,-1,axis=1)-pending,axis=2).argmax(1);row=np.arange(len(pending))
        a=pending[row,edge];b=pending[row,(edge+1)%3];c=pending[row,(edge+2)%3];mid=(a+b)/2
        pending=np.concatenate((np.stack((a,mid,c),axis=1),np.stack((mid,b,c),axis=1)))
    raise RuntimeError('Boundary collision cover could not resolve a safe entry')


def compact_collision_cover(spheres,pose,boxes,cell=.025):
    """Merge child spheres only when their enclosing parent remains clear."""
    pose=np.asarray(pose);rotation=Rotation.from_quat(pose[[4,5,6,3]])
    centers=rotation.apply(np.asarray([s['center'] for s in spheres]))+pose[:3]
    radii=np.asarray([s['radius'] for s in spheres])
    minimum=sphere_world_clearance(pose,spheres,boxes)[0]
    for width in (cell,cell/2,cell/4):
        _,groups=np.unique(np.floor((centers-centers.min(0))/width).astype(int),axis=0,return_inverse=True)
        merged=[];sizes=[]
        for group in range(groups.max()+1):
            indices=np.flatnonzero(groups==group);center=centers[indices].mean(0)
            radius=float(np.max(np.linalg.norm(centers[indices]-center,axis=1)+radii[indices]))
            parent=[dict(center=rotation.inv().apply(center-pose[:3]).tolist(),radius=radius)]
            clearance=sphere_world_clearance(pose,parent,boxes)[0]
            if all(clearance[k]>=minimum[k]-1e-10 for k in minimum):
                merged.append(center);sizes.append(radius)
            else:merged.extend(centers[indices]);sizes.extend(radii[indices])
        centers=np.asarray(merged);radii=np.asarray(sizes)
    local=rotation.inv().apply(centers-pose[:3])
    return [dict(center=c.tolist(),radius=float(s)) for c,s in zip(local,radii)]


def robot_model(scene,carry,out):
    from pxr import Usd,UsdGeom
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    names=['tx','ty','tz','yaw','pitch','roll'];axes=['1 0 0','0 1 0','0 0 1','0 0 1','0 1 0','1 0 0']
    xml=['<robot name="floating_wrist"><link name="base"/>'];parent='base'
    for i,(name,axis) in enumerate(zip(names,axes)):
        child='wrist' if i==5 else 'virtual_'+name
        limit=2 if i<3 else 6.28318530718
        xml.append(f'<link name="{child}"/><joint name="{name}" type="{"prismatic" if i<3 else "revolute"}"><parent link="{parent}"/><child link="{child}"/><axis xyz="{axis}"/><limit lower="{-limit}" upper="{limit}" velocity="{scene.cfg["planning"]["velocity_limits"][i]}" effort="100"/></joint>')
        parent=child
    xml.append('</robot>');urdf=out/'floating_wrist.urdf';urdf.write_text('\n'.join(xml))
    pose,_=scene.root_motion();rot=Rotation.from_quat(pose[[4,5,6,3]]);spheres=[];cache=UsdGeom.XformCache();measured=[]
    scene.measured_hand_mesh_data={}
    for prim in Usd.PrimRange.Stage(scene.world.stage,Usd.TraverseInstanceProxies()):
        if not str(prim.GetPath()).startswith(scene.root+'/') or not prim.IsA(UsdGeom.Mesh):continue
        if 'collision' in str(prim.GetPath()).lower():continue
        mesh=UsdGeom.Mesh(prim);points=np.asarray(mesh.GetPointsAttr().Get());faces=usd_mesh_faces(mesh)
        trans=np.asarray(cache.GetLocalToWorldTransform(prim)).T
        scene.measured_hand_mesh_data[str(prim.GetPath())]=(points,faces)
        scale=np.linalg.norm(trans[:3,:3],axis=0);local=cover_points(points*scale,faces=faces)
        measured.append((prim,scale,local))
        world_points=points@trans[:3,:3].T+trans[:3,3]
        spheres+=cover_points(rot.inv().apply(world_points-pose[:3]),faces=faces)
    scene.measured_hand_collision_meshes=measured
    if carry:
        p,q=scene.objects['tool'].get_world_pose()
        proxy=carried_proxy(scene)
        points=Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).apply(np.array([s['center'] for s in proxy]))+p
        relative=rot.inv().apply(points-pose[:3])
        spheres+=[dict(center=c.tolist(),radius=s['radius']) for c,s in zip(relative,proxy)]
    if not spheres:raise RuntimeError('Floating planner has no hand collision geometry')
    # Diagnose the previous controller error, but do not inflate a new path
    # that starts at the measured pose by an old BC target's residual. At the
    # new command boundary that residual is absent; propagating it here made
    # known physical starts falsely collide with the table/target.
    command=np.asarray(scene.command_pose)
    angle=float((Rotation.from_quat(command[[4,5,6,3]])*rot.inv()).magnitude())
    radius=max(np.linalg.norm(s['center'])+s['radius'] for s in spheres)
    margin=float(np.linalg.norm(command[:3]-pose[:3])+2*radius*np.sin(angle/2))
    scene.previous_command_tracking_margin_m=margin
    scene.planning_tracking_margin_m=0.
    kin={'urdf_path':str(urdf.resolve()),'asset_root_path':str(out.resolve()),'base_link':'base','ee_link':'wrist',
        'collision_link_names':['wrist'],'collision_spheres':{'wrist':spheres},'collision_sphere_buffer':0.,
        'self_collision_ignore':{},'self_collision_buffer':{},'use_global_cumul':True,
        'mesh_link_names':[],'lock_joints':{},'cspace':{'joint_names':names,'retract_config':pose_coordinates(pose).tolist(),
        'null_space_weight':[1.]*6,'cspace_distance_weight':[1.]*6,'max_acceleration':scene.cfg['planning']['acceleration_limits'],
        'max_jerk':500.,'velocity_scale':1.,'acceleration_scale':1.,'jerk_scale':1.}}
    import json
    (out/'collision_geometry.json').write_text(json.dumps(dict(spheres=spheres,
        measured_start_pose=pose.tolist(),previous_command_tracking_margin_m=margin,
            planning_tracking_margin_m=0.,carried_object=carry,
            proxy='conservative triangle-bounding-cell spheres; empty cells excluded'),indent=2))
    return {'kinematics':kin},len(spheres)


class FloatingPlanner:
    def __init__(self,scene,carry,out):
        from curobo.types.base import TensorDeviceType
        from curobo.wrap.reacher.motion_gen import MotionGen,MotionGenConfig
        from curobo.geom.sdf.world import CollisionCheckerType
        self.scene=scene;self.tensor=TensorDeviceType(device='cuda:0');self.carry=carry;self.out=Path(out)
        cfg,self.sphere_count=robot_model(scene,carry,out)
        live={'cuboid':{'table':scene.table}}
        for role in ['target']:
            p,q=scene.objects[role].get_world_pose();bounds=scene.meshes[role].bounds;rot=Rotation.from_quat(np.asarray(q)[[1,2,3,0]])
            live['cuboid'][role]={'dims':(bounds[1]-bounds[0]).tolist(),'pose':np.r_[p+rot.apply(bounds.mean(0)),q].tolist()}
        if not carry:
            p,q=scene.objects['tool'].get_world_pose();oid=scene.manifest['initial_objects']['tool']['id']
            live['mesh']={'tool':dict(file_path=str(scene.args.object_cache/oid/(oid+'_m.obj')),pose=np.r_[p,q].tolist())}
        self.obstacle_roles=['target']
        self.nominal_spheres=cfg['kinematics']['collision_spheres']['wrist']
        self.live=live
        import json
        (Path(out)/'world_geometry.json').write_text(json.dumps(live,indent=2))
        pose,_=scene.root_motion()
        if min(sphere_world_clearance(pose,self.nominal_spheres,live['cuboid'])[0].values())<0:
            from pxr import UsdGeom
            cache=UsdGeom.XformCache();refined=[]
            for prim,_,_ in scene.measured_hand_collision_meshes:
                vertices,faces=scene.measured_hand_mesh_data[str(prim.GetPath())]
                matrix=np.asarray(cache.GetLocalToWorldTransform(prim)).T
                refined+=boundary_collision_cover(vertices@matrix[:3,:3].T+matrix[:3,3],faces,pose,live['cuboid'])
            if carry:
                p,q=scene.objects['tool'].get_world_pose();mesh=scene.meshes['tool']
                refined+=boundary_collision_cover(Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).apply(mesh.vertices)+p,mesh.faces,pose,live['cuboid'])
            refined=compact_collision_cover(refined,pose,live['cuboid'])
            cfg['kinematics']['collision_spheres']['wrist']=refined;self.nominal_spheres=refined;self.sphere_count=len(refined)
            import json
            (self.out/'boundary_collision_cover.json').write_text(json.dumps(dict(conservative_surface_coverage=True,
                sphere_count=len(refined),clearance_m=sphere_world_clearance(pose,refined,live['cuboid'])[0]),indent=2))
            geometry=json.loads((self.out/'collision_geometry.json').read_text())
            geometry.update(spheres=refined,proxy='coarse surface cells with exact boundary triangle subdivision')
            (self.out/'collision_geometry.json').write_text(json.dumps(geometry,indent=2))
        self.planner=MotionGen(MotionGenConfig.load_from_robot_config(cfg,live,tensor_args=self.tensor,interpolation_dt=scene.dt,
            optimize_dt=False,trajopt_dt=1/scene.manifest['fps'],js_trajopt_dt=1/scene.manifest['fps'],
            self_collision_check=False,self_collision_opt=False,
            collision_checker_type=CollisionCheckerType.MESH if live.get('mesh') else CollisionCheckerType.PRIMITIVE,use_cuda_graph=False))
        # Catch a virtual-coordinate convention mismatch before physical use.
        pose,_=scene.root_motion();q=pose_coordinates(pose)
        fk=self.planner.kinematics.get_state(self.tensor.to_device(q.copy())[None])
        actual=np.r_[fk.ee_position.detach().cpu().numpy()[0],fk.ee_quaternion.detach().cpu().numpy()[0]]
        error=(Rotation.from_quat(actual[[4,5,6,3]])*Rotation.from_quat(pose[[4,5,6,3]]).inv()).magnitude()
        if np.linalg.norm(actual[:3]-pose[:3])>1e-5 or error>1e-4:raise RuntimeError('Virtual wrist FK convention mismatch')

    def js(self,q):
        from curobo.types.robot import JointState
        return JointState.from_position(self.tensor.to_device(np.asarray(q).copy())[None],joint_names=self.planner.joint_names)

    def valid(self,q):
        try:return bool(self.planner.check_start_state(self.js(q))[0])
        finally:self.planner.rollout_fn.primitive_collision_constraint.enable_cost()

    def goal_clearance(self,pose):
        """Signed sphere/world clearance from the planning collision backend.

        Mesh bowls retain their empty cavities. The cover already includes
        each cell's radius; adding a second cell radius at the goal would
        displace an otherwise valid pregrasp by centimetres.
        """
        if not self.live.get('mesh'):
            return sphere_world_clearance(pose,self.nominal_spheres,self.live['cuboid'])[1]
        spheres=self.nominal_spheres;pose=np.asarray(pose)
        centers=Rotation.from_quat(pose[[4,5,6,3]]).apply(np.asarray([s['center'] for s in spheres]))+pose[:3]
        return self.world_clearance(centers,np.asarray([s['radius'] for s in spheres]),gradient=True)

    def world_clearance(self,centers,radii,gradient=False):
        from curobo.geom.sdf.world import CollisionQueryBuffer
        query=self.tensor.to_device(np.c_[centers,radii])[None,None].requires_grad_(gradient)
        checker=self.planner.world_coll_checker
        if self.live.get('mesh'):
            from curobo.types.math import Pose
            p,q=self.scene.objects['tool'].get_world_pose()
            checker.update_mesh_pose(w_obj_pose=Pose.from_list(np.r_[p,q].tolist(),self.tensor),name='tool')
        buffer=CollisionQueryBuffer.initialize_from_shape(query.shape,self.tensor,checker.collision_types)
        distance=checker.get_sphere_distance(query,buffer,self.tensor.to_device([1.]),self.tensor.to_device([0.]),compute_esdf=True)
        penetration=distance.max();direction=np.zeros(3)
        if gradient:
            penetration.backward();direction=-query.grad[0,0,:,:3].sum(0).detach().cpu().numpy()
            norm=np.linalg.norm(direction)
            if norm>1e-12:direction/=norm
        return float(-penetration.detach().cpu()),direction,'world'

    def measured_clearance(self):
        # Tracking margin is for commanded paths. Do not count it a second time
        # against measured states; check the underlying physical geometry here.
        pose,_=self.scene.root_motion();r=Rotation.from_quat(pose[[4,5,6,3]])
        centers=r.apply(np.asarray([s['center'] for s in self.nominal_spheres]))+pose[:3]
        radii=np.asarray([s['radius'] for s in self.nominal_spheres])
        if hasattr(self.scene,'measured_hand_collision_meshes'):
            from pxr import UsdGeom
            cache=UsdGeom.XformCache();groups=[];sizes=[]
            for prim,scale,local_spheres in self.scene.measured_hand_collision_meshes:
                matrix=np.asarray(cache.GetLocalToWorldTransform(prim)).T
                local=np.asarray([s['center'] for s in local_spheres])
                groups.append(local@(matrix[:3,:3]/scale).T+matrix[:3,3])
                sizes.extend(s['radius'] for s in local_spheres)
            if self.carry:
                p,q=self.scene.objects['tool'].get_world_pose()
                current=carried_proxy(self.scene)
                groups.append(Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).apply(np.array([s['center'] for s in current]))+p)
                sizes.extend(s['radius'] for s in current)
            centers=np.concatenate(groups);radii=np.asarray(sizes)
        boxes=self.current_boxes()
        clearances={}
        for role,box in boxes.items():
            b=np.asarray(box['pose']);rotation=Rotation.from_quat(b[[4,5,6,3]])
            local=rotation.inv().apply(centers-b[:3]);offset=np.abs(local)-np.asarray(box['dims'])/2
            distance=np.linalg.norm(np.maximum(offset,0),axis=1)+np.minimum(np.max(offset,axis=1),0)-radii
            clearances[role]=float(distance.min())
        if self.live.get('mesh'):clearances['mesh_world']=self.world_clearance(centers,radii)[0]
        return clearances

    def current_boxes(self):
        boxes=dict(self.live['cuboid'])
        for role in getattr(self,'obstacle_roles',[]):
            p,q=self.scene.objects[role].get_world_pose();bounds=self.scene.meshes[role].bounds
            current=Rotation.from_quat(np.asarray(q)[[1,2,3,0]])
            boxes[role]={'dims':(bounds[1]-bounds[0]).tolist(),'pose':np.r_[p+current.apply(bounds.mean(0)),q].tolist()}
        return boxes

    def measured_mesh_collisions(self):
        from pxr import UsdGeom
        cache=UsdGeom.XformCache();meshes=[]
        for prim,_,_ in self.scene.measured_hand_collision_meshes:
            vertices,faces=self.scene.measured_hand_mesh_data[str(prim.GetPath())]
            matrix=np.asarray(cache.GetLocalToWorldTransform(prim)).T
            meshes.append((str(prim.GetPath()),vertices@matrix[:3,:3].T+matrix[:3,3],faces,False))
        if self.carry:
            p,q=self.scene.objects['tool'].get_world_pose();mesh=self.scene.meshes['tool']
            vertices=Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).apply(mesh.vertices)+p
            meshes.append(('tool',vertices,mesh.faces,mesh.is_watertight))
        collisions=[dict(mesh=name,obstacle=role) for name,vertices,faces,closed in meshes
                for role,box in self.current_boxes().items() if mesh_box_intersects(vertices,faces,box,closed)]
        if self.live.get('mesh'):
            # Exact mesh confirmation avoids treating a bowl's cavity as solid
            # and distinguishes a conservative sphere overlap from penetration.
            import fcl
            if not hasattr(self,'fcl_meshes'):
                self.fcl_meshes={}
                for prim,scale,_ in self.scene.measured_hand_collision_meshes:
                    vertices,faces=self.scene.measured_hand_mesh_data[str(prim.GetPath())]
                    model=fcl.BVHModel();model.beginModel(len(vertices),len(faces))
                    model.addSubModel(vertices*scale,faces);model.endModel()
                    self.fcl_meshes[str(prim.GetPath())]=model
                tool=self.scene.meshes['tool'];model=fcl.BVHModel();model.beginModel(len(tool.vertices),len(tool.faces))
                model.addSubModel(tool.vertices,tool.faces);model.endModel();self.fcl_tool=model
            p,q=self.scene.objects['tool'].get_world_pose()
            obstacle=fcl.CollisionObject(self.fcl_tool,fcl.Transform(Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix(),p))
            for prim,scale,_ in self.scene.measured_hand_collision_meshes:
                matrix=np.asarray(cache.GetLocalToWorldTransform(prim)).T
                body=fcl.CollisionObject(self.fcl_meshes[str(prim.GetPath())],fcl.Transform(matrix[:3,:3]/scale,matrix[:3,3]))
                if fcl.collide(body,obstacle,fcl.CollisionRequest(),fcl.CollisionResult()):
                    collisions.append(dict(mesh=str(prim.GetPath()),obstacle='tool',geometry='triangle_mesh'))
        return collisions

    def measured_valid(self):
        return min(self.measured_clearance().values())>=0.

    def plan(self,goal,ticks,goal_twist=None,goal_state=None):
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
        # Native contact buffers are stale immediately after a development
        # restore. snapshot() retains the real acquisition's contact/drive
        # boundary until the first executed step; continuous runs use the
        # current measured state. No solver contact state is fabricated.
        scene=self.scene;start=scene.snapshot();q0=pose_coordinates(start['wrist_xyz_wxyz']);q1=pose_coordinates(goal,q0)
        # Joint parameterization derivatives are obtained from the measured spatial twist.
        twist=np.asarray(start['wrist_twist']);eps=1e-4
        pose=np.asarray(start['wrist_xyz_wxyz']);advanced=pose.copy();advanced[:3]+=eps*twist[:3]
        advanced[3:]=(Rotation.from_rotvec(eps*twist[3:])*Rotation.from_quat(pose[[4,5,6,3]])).as_quat()[[3,0,1,2]]
        dq0=(pose_coordinates(advanced,q0)-q0)/eps
        dq1=None
        if goal_twist is not None:
            terminal=np.asarray(goal).copy();target_twist=np.asarray(goal_twist)
            terminal[:3]+=eps*target_twist[:3]
            terminal[3:]=(Rotation.from_rotvec(eps*target_twist[3:])*Rotation.from_quat(terminal[[4,5,6,3]])).as_quat()[[3,0,1,2]]
            dq1=(pose_coordinates(terminal,q1)-q1)/eps
        begin=self.js(q0);begin.velocity=self.tensor.to_device(dq0.copy())[None]
        end=self.js(q1)
        if dq1 is not None:end.velocity=self.tensor.to_device(dq1.copy())[None]
        result=self.planner.plan_single_js(begin,end,MotionGenPlanConfig(max_attempts=6,enable_graph_attempt=2))
        geometry_solver='trajectory_optimization_fixed_dt'
        if not bool(result.success.item()) and result.status.name in ('DT_EXCEPTION','TRAJOPT_FAIL','FINETUNE_TRAJOPT_FAIL'):
            # A moving entry cannot use cuRobo's static-start time optimizer.
            # Its graph is also a cuRobo geometric planner; every returned
            # path still needs the same fixed-time dynamics/collision fit.
            result=self.planner.plan_single_js(begin,end,MotionGenPlanConfig(enable_graph=True,enable_opt=False,max_attempts=2))
            geometry_solver='curobo_graph_then_fixed_source_time_fit'
        if not bool(result.success.item()):raise RuntimeError(f'Floating cuRobo failed: {result.status}')
        raw=result.get_interpolated_plan().position.detach().cpu().numpy().reshape(-1,6).copy()
        limits=scene.cfg['planning'];dt=scene.dt
        # Impose command boundaries before time fitting and collision checks;
        # these are planned targets, never assignments to simulator state.
        if goal_twist is not None:
            raw[0]=q0;raw[-1]=q1
        np.savez_compressed(self.out/'raw_geometric_path.npz',
                            positions=raw,initial_velocity=dq0,final_velocity=np.zeros(6) if dq1 is None else dq1)
        # A nominally feasible fit can still cut through an obstacle. That is
        # also a rejected fit, not a reason to skip the remaining fixed-time
        # parameterizations of the same cuRobo geometric proposal.
        rejected=[]
        for name,fit in [('fixed_budget',lambda:fixed_budget_path(raw,float(result.interpolation_dt),ticks,dt,np.array(limits['velocity_limits']),np.array(limits['acceleration_limits']),initial_velocity=dq0,final_velocity=dq1)),
                         ('path_parameterization',lambda:parameterize_path(raw,ticks,dt,np.array(limits['velocity_limits']),np.array(limits['acceleration_limits']),initial_velocity=dq0,final_velocity=dq1)),
                         ('boundary_fit',lambda:boundary_fitted_path(raw,ticks,dt,np.array(limits['velocity_limits']),np.array(limits['acceleration_limits']),initial_velocity=dq0,final_velocity=dq1))]:
            try:
                path,timing=fit()
                for k,q in enumerate(path):
                    if not self.valid(q):raise RuntimeError(f'collision at fitted tick {k}')
                break
            except RuntimeError as error:rejected.append(f'{name}: {error}')
        else:
            import json
            (self.out/'fit_rejections.json').write_text(json.dumps(rejected,indent=2))
            raise RuntimeError(f'No collision-free trajectory in source time: {rejected}')
        if rejected:timing['previous_fit_rejections']=rejected
        poses=np.asarray([coordinates_pose(q) for q in path])
        from multipath.anchors.floating import controls_from_poses
        # Preserve the active drive target/preload, rather than commanding the
        # measured contact-deflected hand posture with zero closing error.
        hands=relaxed_hand_path(scene,start,goal_state,ticks)
        commands=controls_from_poses(poses,hands,dt)
        timing.update(planner='cuRobo_virtual_6DOF',hand_collision_spheres=self.sphere_count,carried_object_collision=self.carry,
            geometry_solver=geometry_solver,curobo_optimize_dt=False,
            reference_tracking=False,source_time_stretch=False,arm_model_used=False,tracking_margin_m=scene.planning_tracking_margin_m,
            previous_command_tracking_margin_m=scene.previous_command_tracking_margin_m,
            hand_boundary_motion=bool(np.any(np.abs(hands-hands[0])>1e-8)),
            planned_hand_geometry='entry_posture',
            hand_boundary_collision_validation='measured_geometry_each_physics_step')
        return commands,timing
