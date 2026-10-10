"""cuRobo wrist transport and fixed-time free-hand boundary interpolation.

The six virtual joints are a planning parameterization only. PhysX executes
external wrenches on the real floating hand; no virtual joint drives exist there.
"""
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.interpolate import CubicHermiteSpline
from multipath.planning.timing import fixed_budget_path,parameterize_path,boundary_fitted_path,collision_fitted_path
from multipath.planning.object_geometry import planning_object_mesh


def pose_coordinates(pose,near=None):
    import warnings
    pose=np.asarray(pose)
    with warnings.catch_warnings():
        warnings.filterwarnings('ignore',message='Gimbal lock detected.*',category=UserWarning)
        angles=Rotation.from_quat(pose[[4,5,6,3]]).as_euler('ZYX')
    if near is not None:
        # A pitch crossing pi/2 also flips yaw and roll by pi. Wrapping
        # individual angles by 2*pi misses that equivalent Euler branch and
        # turns a small spatial rotation into an impossible timed bridge.
        if abs(np.cos(angles[1]))<1e-7:
            sign=np.sign(angles[1]);previous=np.asarray(near)[[3,5]];normal=np.array([1.,-sign])
            target=angles[0]+2*np.pi*round((normal@previous-angles[0])/(2*np.pi))
            angles[[0,2]]=previous+normal*(target-normal@previous)/2
        alternatives=np.stack((angles,angles+np.array([np.pi,np.pi-2*angles[1],np.pi])))
        alternatives+=2*np.pi*np.round((np.asarray(near)[None,3:]-alternatives)/(2*np.pi))
        angles=alternatives[np.argmin(np.linalg.norm(alternatives-np.asarray(near)[None,3:],axis=1))]
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
        mesh=planning_object_mesh(scene,'tool')
        scene.carried_collision_proxy=cover_points(mesh.vertices,faces=mesh.faces)
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


def boundary_collision_cover(vertices,faces,pose,boxes,clearance=None):
    """Conservative surface spheres without false overlap at a reached entry.

    Start with the usual coarse cells. Only near a collision-free boundary,
    subdivide surface triangles until their bounding spheres also clear it.
    Every triangle remains covered; world obstacles and acceptance stay fixed.
    """
    pose=np.asarray(pose);rotation=Rotation.from_quat(pose[[4,5,6,3]])
    vertices=np.asarray(vertices);faces=np.asarray(faces,dtype=int)
    coarse=cover_points(rotation.inv().apply(vertices-pose[:3]),faces=faces)
    def clear_mask(center,radius):
        if clearance is not None:return np.asarray(clearance(center,radius))>=0
        clear=np.ones(len(center),dtype=bool)
        for box in boxes.values():
            b=np.asarray(box['pose']);r=Rotation.from_quat(b[[4,5,6,3]])
            offset=np.abs(r.inv().apply(center-b[:3]))-np.asarray(box['dims'])/2
            clear&=np.linalg.norm(np.maximum(offset,0),axis=1)+np.minimum(offset.max(1),0)-radius>=0
        return clear
    center=rotation.apply(np.asarray([s['center'] for s in coarse]))+pose[:3]
    radius=np.asarray([s['radius'] for s in coarse])
    if clear_mask(center,radius).all():return coarse
    if clearance is not None and np.any(np.asarray(clearance(vertices,np.zeros(len(vertices))))<0):
        # Refinement cannot cover an already blocked surface point with a
        # positive-radius sphere. Avoid exponential triangle subdivision.
        raise RuntimeError('Boundary collision cover could not resolve a safe entry: surface lies in collision world')
    collisions=[name for name,box in boxes.items() if mesh_box_intersects(vertices,faces,box)]
    if collisions:raise RuntimeError(f'Actual mesh intersects a planning obstacle at entry: {collisions}')
    clear=clear_mask(center,radius)
    centers=[center[clear]];radii=[radius[clear]];triangles=vertices[faces]
    needed=np.zeros(len(triangles),dtype=bool);lo=triangles.min(1);hi=triangles.max(1)
    for c,s in zip(center[~clear],radius[~clear]):
        needed|=np.all(lo<=c+s,axis=1)&np.all(hi>=c-s,axis=1)
    # Outside the rejected cells' bounding cubes, the retained coarse spheres
    # already cover every surface point. Only neighboring faces need refinement.
    pending=triangles[needed]
    for _ in range(18):
        center=pending.mean(1);radius=np.linalg.norm(pending-center[:,None],axis=2).max(1)+1e-7
        if clearance is not None and np.any(np.asarray(clearance(center,np.zeros(len(center))))<0):
            raise RuntimeError('Boundary collision cover could not resolve a safe entry: surface lies in collision world')
        clear=clear_mask(center,radius)
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


def compact_collision_cover(spheres,pose,boxes,cell=.025,clearance=None):
    """Merge child spheres only when their enclosing parent remains clear."""
    pose=np.asarray(pose);rotation=Rotation.from_quat(pose[[4,5,6,3]])
    centers=rotation.apply(np.asarray([s['center'] for s in spheres]))+pose[:3]
    radii=np.asarray([s['radius'] for s in spheres])
    if clearance is not None:
        minimum=float(np.min(clearance(centers,radii)))
        for width in (cell,cell/2,cell/4):
            _,groups=np.unique(np.floor((centers-centers.min(0))/width).astype(int),axis=0,return_inverse=True)
            members=[np.flatnonzero(groups==group) for group in range(groups.max()+1)]
            parents=np.array([centers[indices].mean(0) for indices in members])
            sizes=np.array([np.max(np.linalg.norm(centers[indices]-center,axis=1)+radii[indices])
                for indices,center in zip(members,parents)])
            safe=np.asarray(clearance(parents,sizes))>=minimum-1e-10
            centers=np.concatenate([parents[i:i+1] if safe[i] else centers[indices] for i,indices in enumerate(members)])
            radii=np.concatenate([sizes[i:i+1] if safe[i] else radii[indices] for i,indices in enumerate(members)])
        local=rotation.inv().apply(centers-pose[:3])
        return [dict(center=c.tolist(),radius=float(s)) for c,s in zip(local,radii)]
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


def robot_model(scene,carry,out,goal_hand=None,separate_carried_object=False):
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
    scene.measured_hand_mesh_data={};excluded=[]
    owned_links=tuple(str(path) for path in scene.contacts.prim_paths)
    for prim in Usd.PrimRange.Stage(scene.world.stage,Usd.TraverseInstanceProxies()):
        path=str(prim.GetPath())
        if not path.startswith(scene.root+'/') or not prim.IsA(UsdGeom.Mesh):continue
        if not any(path.startswith(link+'/') for link in owned_links):
            excluded.append(path);continue
        if 'collision' in path.lower():continue
        mesh=UsdGeom.Mesh(prim);points=np.asarray(mesh.GetPointsAttr().Get());faces=usd_mesh_faces(mesh)
        trans=np.asarray(cache.GetLocalToWorldTransform(prim)).T
        scene.measured_hand_mesh_data[str(prim.GetPath())]=(points,faces)
        scale=np.linalg.norm(trans[:3,:3],axis=0);local=cover_points(points*scale,faces=faces)
        measured.append((prim,scale,local))
        world_points=points@trans[:3,:3].T+trans[:3,3]
        spheres+=cover_points(rot.inv().apply(world_points-pose[:3]),faces=faces)
    scene.measured_hand_collision_meshes=measured
    import json
    (out/'articulation_geometry.json').write_text(json.dumps(dict(
        owned_physical_links=list(owned_links),selected_meshes=[str(p.GetPath()) for p,_,_ in measured],
        excluded_other_articulation_meshes=excluded,ownership='actual hand rigid-body view; not USD ancestor prefix'),indent=2))
    if not measured:raise RuntimeError('No geometry belongs to this physical articulation')
    sweep_count=1
    if not carry and goal_hand is not None and not np.allclose(goal_hand,scene.state()['hand_q'],atol=1e-8,rtol=0):
        from multipath.planning.hand_geometry import hand_sweep_spheres
        meshes=[(prim,*scene.measured_hand_mesh_data[str(prim.GetPath())]) for prim,_,_ in measured]
        spheres,sweep_count=hand_sweep_spheres(scene,goal_hand,meshes,cover_points)
    scene.planned_hand_sweep_samples=sweep_count
    if carry and not separate_carried_object:
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
    def __init__(self,scene,carry,out,goal_hand=None,allowed_object_contacts=()):
        from multipath.planning.curobo_compat import install_curobo_compatibility
        install_curobo_compatibility()
        from curobo.types.base import TensorDeviceType
        from curobo.wrap.reacher.motion_gen import MotionGen,MotionGenConfig
        from curobo.geom.sdf.world import CollisionCheckerType
        self.scene=scene;self.tensor=TensorDeviceType(device='cuda:0');self.carry=carry;self.out=Path(out)
        self.goal_hand=goal_hand;self.goal_cover_checked=False
        self.allowed_object_contacts=tuple(allowed_object_contacts)
        self.separate_object=bool(carry and self.allowed_object_contacts)
        cfg,self.sphere_count=robot_model(scene,carry,out,goal_hand,self.separate_object)
        live={'cuboid':{'table':scene.table},'mesh':{}}
        for role in (['target'] if carry else ['tool','target']):
            p,q=scene.objects[role].get_world_pose()
            path=self.out/(role+'_planning_collision.obj');planning_object_mesh(scene,role).export(path)
            live['mesh'][role]=dict(file_path=str(path),pose=np.r_[p,q].tolist())
        from multipath.planning.hand_geometry import measured_peer_meshes
        self.peer_meshes=measured_peer_meshes(scene,self.out)
        for role,value in self.peer_meshes.items():
            p,q=self.mesh_obstacle_pose(role)
            live['mesh'][role]=dict(file_path=value['file_path'],pose=np.r_[p,q].tolist())
        self.obstacle_roles=[]
        self.nominal_spheres=cfg['kinematics']['collision_spheres']['wrist']
        self.live=live
        if any(role not in live['cuboid'] and role not in live['mesh'] for role in self.allowed_object_contacts):
            raise ValueError('Allowed object contact names an absent environment geometry')
        import json
        (Path(out)/'world_geometry.json').write_text(json.dumps(live,indent=2))
        (Path(out)/'object_collision_model.json').write_text(json.dumps(scene.planning_object_geometry,indent=2))
        pose,_=scene.root_motion()
        if min(sphere_world_clearance(pose,self.nominal_spheres,live['cuboid'])[0].values())<0:
            from pxr import UsdGeom
            cache=UsdGeom.XformCache();refined=[]
            for prim,_,_ in scene.measured_hand_collision_meshes:
                vertices,faces=scene.measured_hand_mesh_data[str(prim.GetPath())]
                matrix=np.asarray(cache.GetLocalToWorldTransform(prim)).T
                refined+=boundary_collision_cover(vertices@matrix[:3,:3].T+matrix[:3,3],faces,pose,live['cuboid'])
            if carry and not self.separate_object:
                p,q=scene.objects['tool'].get_world_pose();mesh=planning_object_mesh(scene,'tool')
                refined+=boundary_collision_cover(Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).apply(mesh.vertices)+p,mesh.faces,pose,live['cuboid'])
            refined=compact_collision_cover(refined,pose,live['cuboid'])
            cfg['kinematics']['collision_spheres']['wrist']=refined;self.nominal_spheres=refined;self.sphere_count=len(refined)
            import json
            (self.out/'boundary_collision_cover.json').write_text(json.dumps(dict(conservative_surface_coverage=True,
                sphere_count=len(refined),clearance_m=sphere_world_clearance(pose,refined,live['cuboid'])[0]),indent=2))
            geometry=json.loads((self.out/'collision_geometry.json').read_text())
            geometry.update(spheres=refined,proxy='coarse surface cells with exact boundary triangle subdivision')
            (self.out/'collision_geometry.json').write_text(json.dumps(geometry,indent=2))
        def build_planner(source_ticks=None):
            import faulthandler
            # This constructor stalled for an entire allocation. Capture its
            # actual stack and stop the native startup, not a physical rollout.
            print('CUROBO_BUILD_BEGIN',len(cfg['kinematics']['collision_spheres']['wrist']),flush=True)
            faulthandler.dump_traceback_later(90,exit=True)
            try:
                # Moving carry boundaries cannot be optimized on a one-second
                # clock and then compressed into a short source interval: that
                # scales the physical entry velocity. Optimize in the interval's
                # actual budget. The established free-motion backend is retained.
                # cuRobo requires trajectory dt >= interpolation dt. Short
                # intervals therefore use fewer knots, without adding time.
                steps=32 if source_ticks is None else min(32,source_ticks+1)
                opt_dt=(1/scene.manifest['fps'] if source_ticks is None
                        else source_ticks*scene.dt/(steps-1))
                options={} if source_ticks is None else dict(trajopt_tsteps=steps,
                    minimum_trajectory_dt=scene.dt)
                return MotionGen(MotionGenConfig.load_from_robot_config(cfg,live,tensor_args=self.tensor,interpolation_dt=scene.dt,
                    optimize_dt=False,trajopt_dt=opt_dt,js_trajopt_dt=opt_dt,
                    self_collision_check=False,self_collision_opt=False,
                    collision_checker_type=CollisionCheckerType.MESH if live.get('mesh') else CollisionCheckerType.PRIMITIVE,use_cuda_graph=False,**options))
            finally:faulthandler.cancel_dump_traceback_later()
        self.robot_config=cfg;self.build_planner=build_planner
        self.planner=build_planner()
        pose,_=scene.root_motion()
        centers=Rotation.from_quat(pose[[4,5,6,3]]).apply(np.array([s['center'] for s in self.nominal_spheres]))+pose[:3]
        radii=np.array([s['radius'] for s in self.nominal_spheres])
        if live.get('mesh') and np.min(self.world_clearance(centers,radii,per_sphere=True))<0:
            collisions=self.measured_mesh_collisions()
            if collisions:raise RuntimeError(f'Actual mesh intersects a planning obstacle at entry: {collisions}')
            # The box-only cover refinement missed conservative sphere overlap
            # with mesh obstacles. Refine the same full surfaces against the
            # actual cuRobo world, without disabling collision or shrinking a
            # triangle's coverage.
            from pxr import UsdGeom
            cache=UsdGeom.XformCache();refined=[]
            clearance=lambda c,r:self.world_clearance(c,r,per_sphere=True)
            for prim,_,_ in scene.measured_hand_collision_meshes:
                vertices,faces=scene.measured_hand_mesh_data[str(prim.GetPath())]
                matrix=np.asarray(cache.GetLocalToWorldTransform(prim)).T
                refined+=boundary_collision_cover(vertices@matrix[:3,:3].T+matrix[:3,3],faces,pose,live['cuboid'],clearance=clearance)
            if carry and not self.separate_object:
                p,q=scene.objects['tool'].get_world_pose();mesh=planning_object_mesh(scene,'tool')
                vertices=Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).apply(mesh.vertices)+p
                refined+=boundary_collision_cover(vertices,mesh.faces,pose,live['cuboid'],clearance=clearance)
            refined=compact_collision_cover(refined,pose,live['cuboid'],clearance=clearance)
            self.nominal_spheres=refined;self.sphere_count=len(refined)
            cfg['kinematics']['collision_spheres']['wrist']=refined
            self.planner=build_planner()
            geometry=json.loads((self.out/'collision_geometry.json').read_text())
            geometry.update(spheres=refined,proxy='full surface coverage refined against actual mesh world')
            (self.out/'collision_geometry.json').write_text(json.dumps(geometry,indent=2))
            (self.out/'boundary_collision_cover.json').write_text(json.dumps(dict(conservative_surface_coverage=True,
                sphere_count=len(refined),obstacles='actual cuboid and mesh world'),indent=2))
        if self.separate_object:
            p,q=scene.objects['tool'].get_world_pose();mesh=planning_object_mesh(scene,'tool')
            vertices=Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).apply(mesh.vertices)+p
            self.object_spheres=boundary_collision_cover(vertices,mesh.faces,pose,{},
                clearance=lambda world,radius:self.world_clearance(world,radius,per_sphere=True,
                    ignored=self.allowed_object_contacts))
            geometry=json.loads((self.out/'collision_geometry.json').read_text())
            geometry.update(attached_object_spheres=self.object_spheres,
                allowed_attached_object_contacts=list(self.allowed_object_contacts),
                hand_environment_collision_unchanged=True,
                attached_object_collision='separate full geometry query; only declared object/environment pairs filtered')
            (self.out/'collision_geometry.json').write_text(json.dumps(geometry,indent=2))
        self.verify_fk()

    def verify_fk(self):
        import json
        # Catch a virtual-coordinate convention mismatch before physical use.
        pose,_=self.scene.root_motion();q=pose_coordinates(pose)
        fk=self.planner.kinematics.get_state(self.tensor.to_device(q.copy())[None])
        actual=np.r_[fk.ee_position.detach().cpu().numpy()[0],fk.ee_quaternion.detach().cpu().numpy()[0]]
        error=(Rotation.from_quat(actual[[4,5,6,3]])*Rotation.from_quat(pose[[4,5,6,3]]).inv()).magnitude()
        if np.linalg.norm(actual[:3]-pose[:3])>1e-5 or error>1e-4:raise RuntimeError('Virtual wrist FK convention mismatch')
        # EEF-only validation missed sphere-address overflow: its pose stayed
        # correct while dense collision geometry corrupted GPU memory.
        actual_spheres=fk.get_link_spheres().detach().cpu().numpy()[0]
        expected_centers=Rotation.from_quat(pose[[4,5,6,3]]).apply(np.asarray(
            [s['center'] for s in self.nominal_spheres]))+pose[:3]
        sphere_error=float(np.max(np.linalg.norm(actual_spheres[:,:3]-expected_centers,axis=1)))
        radius_error=float(np.max(np.abs(actual_spheres[:,3]-np.asarray([s['radius'] for s in self.nominal_spheres]))))
        (self.out/'sphere_fk_verification.json').write_text(json.dumps(dict(
            sphere_count=self.sphere_count,max_position_error_m=sphere_error,max_radius_error_m=radius_error,
            forward_sphere_chunk_size=8192,full_geometry_retained=True),indent=2))
        if not np.isfinite([sphere_error,radius_error]).all() or sphere_error>1e-5 or radius_error>1e-6:
            raise RuntimeError('Virtual wrist collision sphere FK mismatch')

    def js(self,q):
        from curobo.types.robot import JointState
        return JointState.from_position(self.tensor.to_device(np.asarray(q).copy())[None],joint_names=self.planner.joint_names)

    def valid(self,q):
        try:
            valid=bool(self.planner.check_start_state(self.js(q))[0])
            return valid and (not self.separate_object or self.object_path_clearance(np.asarray(q)[None])[0]>=0)
        finally:self.planner.rollout_fn.primitive_collision_constraint.enable_cost()

    def goal_clearance(self,pose):
        """Signed sphere/world clearance from the planning collision backend.

        Mesh bowls retain their empty cavities. The cover already includes
        each cell's radius; adding a second cell radius at the goal would
        displace an otherwise valid pregrasp by centimetres.
        """
        if not self.goal_cover_checked:
            self.refine_goal_cover(pose)
            self.goal_cover_checked=True
        if not self.live.get('mesh'):
            return sphere_world_clearance(pose,self.nominal_spheres,self.live['cuboid'])[1]
        spheres=self.nominal_spheres;pose=np.asarray(pose)
        centers=Rotation.from_quat(pose[[4,5,6,3]]).apply(np.asarray([s['center'] for s in spheres]))+pose[:3]
        return self.world_clearance(centers,np.asarray([s['radius'] for s in spheres]),gradient=True)

    def refine_goal_cover(self,goal):
        """Resolve conservative sphere overlap at a proposed free boundary.

        Refining only the current start left valid millimetre-clear pregrasps
        displaced by centimetres at the goal. Cover the same full hand sweep
        against both boundaries; neither geometry nor obstacles are removed.
        """
        if self.carry:
            start,_=self.scene.root_motion()
            path=np.stack((pose_coordinates(start),pose_coordinates(goal,pose_coordinates(start))))
            if not self.valid(path[-1]):self.refine_path_cover(path,[1])
            return
        if self.goal_hand is None:return
        from multipath.planning.hand_geometry import hand_sweep_meshes
        start,_=self.scene.root_motion();start=np.asarray(start);goal=np.asarray(goal)
        r0=Rotation.from_quat(start[[4,5,6,3]]);r1=Rotation.from_quat(goal[[4,5,6,3]])
        centers=r1.apply(np.asarray([s['center'] for s in self.nominal_spheres]))+goal[:3]
        radii=np.asarray([s['radius'] for s in self.nominal_spheres])
        margin=1e-4
        if np.min(self.world_clearance(centers,radii,per_sphere=True))>=margin:return
        meshes=[(prim,*self.scene.measured_hand_mesh_data[str(prim.GetPath())])
                for prim,_,_ in self.scene.measured_hand_collision_meshes]
        geometry,count=hand_sweep_meshes(self.scene,self.goal_hand,meshes)
        import json
        def collision_diagnostic(index,boundary,obstacle):
            (self.out/'goal_boundary_geometry.json').write_text(json.dumps(dict(
                refined=False,reason='full hand mesh intersection',
                hand_mesh=str(meshes[index%len(meshes)][0].GetPath()),
                boundary=boundary,obstacle=obstacle,goal_pose=goal.tolist()),indent=2))
        import fcl
        obstacles=[]
        for role in self.live.get('mesh',{}):
            mesh=self.mesh_obstacle(role);model=fcl.BVHModel();model.beginModel(len(mesh.vertices),len(mesh.faces))
            model.addSubModel(mesh.vertices,mesh.faces);model.endModel()
            p,q=self.mesh_obstacle_pose(role)
            obstacles.append((role,fcl.CollisionObject(model,fcl.Transform(Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix(),p))))
        # If an actual swept surface intersects at either boundary, retain
        # normal boundary projection. Refinement cannot make it collision-free.
        for i,(vertices,faces) in enumerate(geometry):
            model=fcl.BVHModel();model.beginModel(len(vertices),len(faces));model.addSubModel(vertices,faces);model.endModel()
            for boundary,pose,rotation in (('start',start,r0),('goal',goal,r1)):
                world=rotation.apply(vertices)+pose[:3]
                for role,box in self.live['cuboid'].items():
                    if mesh_box_intersects(world,faces,box):
                        collision_diagnostic(i,boundary,role);return
                body=fcl.CollisionObject(model,fcl.Transform(rotation.as_matrix(),pose[:3]))
                for role,obstacle in obstacles:
                    if fcl.collide(body,obstacle,fcl.CollisionRequest(),fcl.CollisionResult()):
                        collision_diagnostic(i,boundary,role);return
        def clearance(world,radius):
            at_goal=r1.apply(r0.inv().apply(world-start[:3]))+goal[:3]
            return np.minimum(self.world_clearance(world,radius,per_sphere=True),
                              self.world_clearance(at_goal,radius,per_sphere=True))-margin
        refined=[]
        for vertices,faces in geometry:
            refined+=boundary_collision_cover(r0.apply(vertices)+start[:3],faces,start,self.live['cuboid'],clearance=clearance)
        refined=compact_collision_cover(refined,start,self.live['cuboid'],clearance=clearance)
        self.robot_config['kinematics']['collision_spheres']['wrist']=refined
        self.nominal_spheres=refined;self.sphere_count=len(refined);self.planner=self.build_planner()
        self.verify_fk()
        import json
        path=self.out/'collision_geometry.json';report=json.loads(path.read_text())
        report.update(spheres=refined,proxy='full hand sweep refined against both free boundary meshes',
                      free_goal_pose=goal.tolist(),hand_sweep_samples=count)
        path.write_text(json.dumps(report,indent=2))

    def world_clearance(self,centers,radii,gradient=False,per_sphere=False,ignored=()):
        checker=self.planner.world_coll_checker
        for role in ignored:checker.enable_obstacle(role,False)
        try:return self._world_clearance(centers,radii,gradient,per_sphere)
        finally:
            for role in ignored:checker.enable_obstacle(role,True)

    def _world_clearance(self,centers,radii,gradient=False,per_sphere=False):
        from curobo.geom.sdf.world import CollisionQueryBuffer
        # cuRobo's mesh ESDF kernel returns distance to the sphere *center*;
        # its collision-cost kernel adds the radius separately. Query points
        # explicitly and add the radius after the world union so the clearance
        # used for projection/refinement matches the planner's sphere collision.
        # This also avoids relying on primitive/mesh ESDF radius conventions.
        query=self.tensor.to_device(np.c_[centers,np.zeros(len(radii))])[None,None].requires_grad_(gradient)
        checker=self.planner.world_coll_checker
        if self.live.get('mesh'):
            from curobo.types.math import Pose
            for role in self.live['mesh']:
                p,q=self.mesh_obstacle_pose(role)
                checker.update_mesh_pose(w_obj_pose=Pose.from_list(np.r_[p,q].tolist(),self.tensor),name=role)
        buffer=CollisionQueryBuffer.initialize_from_shape(query.shape,self.tensor,checker.collision_types)
        distance=checker.get_sphere_distance(query,buffer,self.tensor.to_device([1.]),self.tensor.to_device([0.]),
            compute_esdf=True,return_loss=gradient)
        distance=distance+self.tensor.to_device(np.asarray(radii).copy())[None,None]
        if per_sphere:return -distance.detach().cpu().numpy().reshape(-1)
        penetration=distance.max();direction=np.zeros(3)
        if gradient:
            penetration.backward();direction=-query.grad[0,0,:,:3].sum(0).detach().cpu().numpy()
            norm=np.linalg.norm(direction)
            if norm>1e-12:direction/=norm
        return float(-penetration.detach().cpu()),direction,'world'

    def refine_path_cover(self,path,invalid):
        """Refine conservative spheres when exact carried meshes clear a fit.

        Refine against the fit's mesh-safe configurations. Real colliding
        samples remain rejected until the time fitter finds a safe path.
        Every surface is retained; no simulator state is assigned.
        """
        if not self.carry:return False
        import json
        print('PATH_COVER_REFINE_BEGIN',len(path),len(invalid),flush=True)
        poses=np.asarray([coordinates_pose(q) for q in path]);blocked=[]
        for tick in invalid:
            collision=self.measured_mesh_collisions(poses[tick])
            if collision:
                blocked.append(dict(tick=tick,collisions=collision))
        safe=[k for k in range(len(path)) if k not in {v['tick'] for v in blocked}]
        if not safe:return False
        poses=poses[safe]
        from pxr import UsdGeom
        start,_=self.scene.root_motion();start=np.asarray(start)
        r0=Rotation.from_quat(start[[4,5,6,3]])
        rotations=Rotation.from_quat(poses[:,[4,5,6,3]]).as_matrix()
        def clearance(world,radius):
            local=r0.inv().apply(np.asarray(world)-start[:3])
            centers=np.einsum('nij,pj->npi',rotations,local)+poses[:,None,:3]
            distances=self.world_clearance(centers.reshape(-1,3),
                np.tile(radius,len(poses)),per_sphere=True)
            return distances.reshape(len(poses),len(local)).min(0)
        cache=UsdGeom.XformCache();refined=[]
        for prim,_,_ in self.scene.measured_hand_collision_meshes:
            vertices,faces=self.scene.measured_hand_mesh_data[str(prim.GetPath())]
            matrix=np.asarray(cache.GetLocalToWorldTransform(prim)).T
            refined+=boundary_collision_cover(vertices@matrix[:3,:3].T+matrix[:3,3],
                faces,start,self.live['cuboid'],clearance=clearance)
        if not self.separate_object:
            p,q=self.scene.objects['tool'].get_world_pose();mesh=planning_object_mesh(self.scene,'tool')
            refined+=boundary_collision_cover(Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).apply(mesh.vertices)+p,
                mesh.faces,start,self.live['cuboid'],clearance=clearance)
        refined=compact_collision_cover(refined,start,self.live['cuboid'],clearance=clearance)
        self.robot_config['kinematics']['collision_spheres']['wrist']=refined
        self.nominal_spheres=refined;self.sphere_count=len(refined)
        self.planner=self.build_planner(source_ticks=getattr(self,'source_time_trajopt_ticks',None))
        self.verify_fk();self.path_cover_refined=True
        print('PATH_COVER_REFINE_END',self.sphere_count,flush=True)
        geometry=json.loads((self.out/'collision_geometry.json').read_text())
        geometry.update(spheres=refined,proxy='full surfaces refined over fixed-time fitted path')
        (self.out/'collision_geometry.json').write_text(json.dumps(geometry,indent=2))
        (self.out/'fitted_path_geometry.json').write_text(json.dumps(dict(
            refined=True,physics_samples=len(path),safe_samples_used=len(safe),sphere_count=len(refined),
            colliding_fit_samples=blocked,conservative_surface_coverage=True,
            source_time_stretch=False,simulator_state_assignment=False),indent=2))
        return True

    def path_clearance_gradient(self,path):
        """cuRobo distance gradient in all six virtual wrist coordinates."""
        from curobo.geom.sdf.world import CollisionQueryBuffer
        from curobo.types.math import Pose
        poses=np.asarray([coordinates_pose(q) for q in path])
        local=np.asarray([s['center'] for s in self.nominal_spheres])
        rotations=Rotation.from_quat(poses[:,[4,5,6,3]]).as_matrix()
        centers=np.einsum('nij,pj->npi',rotations,local)+poses[:,None,:3]
        radii=np.asarray([s['radius'] for s in self.nominal_spheres])
        points=np.concatenate((centers,np.zeros((*centers.shape[:2],1))),axis=2)
        query=self.tensor.to_device(points)[None].requires_grad_(True)
        checker=self.planner.world_coll_checker
        for role in self.live.get('mesh',{}):
            p,q=self.mesh_obstacle_pose(role)
            checker.update_mesh_pose(w_obj_pose=Pose.from_list(np.r_[p,q].tolist(),self.tensor),name=role)
        buffer=CollisionQueryBuffer.initialize_from_shape(query.shape,self.tensor,checker.collision_types)
        distance=checker.get_sphere_distance(query,buffer,self.tensor.to_device([1.]),
            self.tensor.to_device([0.]),compute_esdf=True,return_loss=True)
        clearance=-(distance+self.tensor.to_device(radii.copy())[None,None]).max(dim=-1).values
        clearance.sum().backward()
        point_gradient=query.grad[0,:,:,:3].detach().cpu().numpy()
        gradient=np.empty_like(path,dtype=float);gradient[:,:3]=point_gradient.sum(axis=1)
        # Chain the active surface normal through the same ZYX FK used by
        # cuRobo. Changing position alone cannot avoid a rotating carried tool.
        epsilon=1e-5
        for j in range(3,6):
            angles=np.asarray(path[:,3:]).copy();angles[:,j-3]+=epsilon
            forward=np.einsum('nij,pj->npi',Rotation.from_euler('ZYX',angles).as_matrix(),local)
            angles[:,j-3]-=2*epsilon
            backward=np.einsum('nij,pj->npi',Rotation.from_euler('ZYX',angles).as_matrix(),local)
            gradient[:,j]=np.sum(point_gradient*(forward-backward)/(2*epsilon),axis=(1,2))
        values=clearance.detach().cpu().numpy().reshape(-1)
        if self.separate_object:
            object_values=self.object_path_clearance(path)
            active=object_values<values
            if active.any():
                derivative=np.empty_like(gradient);epsilon=1e-4
                for joint in range(6):
                    shifted=np.asarray(path).copy();shifted[:,joint]+=epsilon
                    derivative[:,joint]=(self.object_path_clearance(shifted)-object_values)/epsilon
                gradient[active]=derivative[active];values=np.minimum(values,object_values)
        return values,gradient

    def object_path_clearance(self,path):
        poses=np.asarray([coordinates_pose(q) for q in path])
        local=np.asarray([s['center'] for s in self.object_spheres])
        radius=np.asarray([s['radius'] for s in self.object_spheres])
        rotations=Rotation.from_quat(poses[:,[4,5,6,3]]).as_matrix()
        centers=np.einsum('nij,pj->npi',rotations,local)+poses[:,None,:3]
        distance=self.world_clearance(centers.reshape(-1,3),np.tile(radius,len(path)),
            per_sphere=True,ignored=self.allowed_object_contacts)
        return distance.reshape(len(path),len(radius)).min(1)

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
            if self.carry and not self.separate_object:
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
        if self.separate_object:
            p,q=self.scene.objects['tool'].get_world_pose();proxy=carried_proxy(self.scene)
            object_centers=Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).apply(np.asarray([s['center'] for s in proxy]))+p
            clearances['attached_other_environment']=self.world_clearance(object_centers,
                np.asarray([s['radius'] for s in proxy]),ignored=self.allowed_object_contacts)[0]
        return clearances

    def current_boxes(self):
        boxes=dict(self.live['cuboid'])
        for role in getattr(self,'obstacle_roles',[]):
            p,q=self.scene.objects[role].get_world_pose();bounds=planning_object_mesh(self.scene,role).bounds
            current=Rotation.from_quat(np.asarray(q)[[1,2,3,0]])
            boxes[role]={'dims':(bounds[1]-bounds[0]).tolist(),'pose':np.r_[p+current.apply(bounds.mean(0)),q].tolist()}
        return boxes

    def mesh_obstacle(self,role):
        if role in self.peer_meshes:return self.peer_meshes[role]['mesh']
        return planning_object_mesh(self.scene,role)

    def mesh_obstacle_pose(self,role):
        if role not in self.peer_meshes:return self.scene.objects[role].get_world_pose()
        from pxr import UsdGeom
        value=self.peer_meshes[role]
        matrix=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(value['prim'])).T
        quaternion=Rotation.from_matrix(matrix[:3,:3]/value['scale']).as_quat()[[3,0,1,2]]
        return matrix[:3,3],quaternion

    def measured_mesh_collisions(self,planned_pose=None):
        from pxr import UsdGeom
        cache=UsdGeom.XformCache();meshes=[]
        delta=np.eye(4)
        if planned_pose is not None:
            from multipath.execution.mounted_scene import pose_matrix
            actual,_=self.scene.root_motion()
            delta=pose_matrix(planned_pose)@np.linalg.inv(pose_matrix(actual))
        for prim,_,_ in self.scene.measured_hand_collision_meshes:
            vertices,faces=self.scene.measured_hand_mesh_data[str(prim.GetPath())]
            matrix=delta@np.asarray(cache.GetLocalToWorldTransform(prim)).T
            meshes.append((str(prim.GetPath()),vertices@matrix[:3,:3].T+matrix[:3,3],faces,False))
        if self.carry:
            p,q=self.scene.objects['tool'].get_world_pose();mesh=planning_object_mesh(self.scene,'tool')
            vertices=Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).apply(mesh.vertices)+p
            vertices=vertices@delta[:3,:3].T+delta[:3,3]
            meshes.append(('tool',vertices,mesh.faces,mesh.is_watertight))
        collisions=[dict(mesh=name,obstacle=role) for name,vertices,faces,closed in meshes
                for role,box in self.current_boxes().items()
                if not (name=='tool' and role in self.allowed_object_contacts)
                and mesh_box_intersects(vertices,faces,box,closed)]
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
                self.fcl_objects={}
                for role in set(self.scene.meshes)|set(self.live['mesh']):
                    mesh=self.mesh_obstacle(role)
                    model=fcl.BVHModel();model.beginModel(len(mesh.vertices),len(mesh.faces))
                    model.addSubModel(mesh.vertices,mesh.faces);model.endModel();self.fcl_objects[role]=model
            for role in self.live['mesh']:
                p,q=self.mesh_obstacle_pose(role)
                obstacle=fcl.CollisionObject(self.fcl_objects[role],fcl.Transform(Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix(),p))
                bodies=[]
                for prim,scale,_ in self.scene.measured_hand_collision_meshes:
                    matrix=delta@np.asarray(cache.GetLocalToWorldTransform(prim)).T
                    bodies.append((str(prim.GetPath()),fcl.CollisionObject(self.fcl_meshes[str(prim.GetPath())],fcl.Transform(matrix[:3,:3]/scale,matrix[:3,3]))))
                if self.carry:
                    p,q=self.scene.objects['tool'].get_world_pose()
                    rotation=delta[:3,:3]@Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix()
                    position=delta[:3,:3]@p+delta[:3,3]
                    bodies.append(('tool',fcl.CollisionObject(self.fcl_objects['tool'],fcl.Transform(rotation,position))))
                for name,body in bodies:
                    if name=='tool' and role in self.allowed_object_contacts:continue
                    if fcl.collide(body,obstacle,fcl.CollisionRequest(),fcl.CollisionResult()):
                        collisions.append(dict(mesh=name,obstacle=role,geometry='triangle_mesh'))
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
        if self.carry and getattr(self,'source_time_trajopt_ticks',None)!=ticks:
            self.planner=self.build_planner(source_ticks=ticks)
            self.source_time_trajopt_ticks=ticks
            self.verify_fk()
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
        if not bool(result.success.item()):
            import json
            states=self.tensor.to_device(np.stack((q0,q1)))[:,None]
            details={}
            # GRAPH_FAIL's "in collision" text also covers other feasibility
            # terms. Save their actual values rather than repeatedly repairing
            # an Anchor that already passes local physics.
            for label,rollout in [('motion',self.planner.rollout_fn),('graph',self.planner.graph_planner.safety_rollout_fn)]:
                state=rollout.dynamics_model.forward(rollout.start_state,states)
                components=dict(bounds=rollout.bound_constraint.forward(state.state_seq))
                if rollout.primitive_collision_constraint.enabled:
                    components['world']=rollout.primitive_collision_constraint.forward(state.robot_spheres,env_query_idx=None)
                if rollout.robot_self_collision_constraint.enabled:
                    components['self']=rollout.robot_self_collision_constraint.forward(state.robot_spheres)
                details[label]={k:v.detach().cpu().numpy().tolist() for k,v in components.items()}
            (self.out/'planner_failure.json').write_text(json.dumps(dict(status=str(result.status),
                entry_coordinates=q0.tolist(),goal_coordinates=q1.tolist(),constraints=details),indent=2))
            raise RuntimeError(f'Floating cuRobo failed: {result.status}')
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
        path_cover_attempted=False
        for name,fit in [('fixed_budget',lambda:fixed_budget_path(raw,float(result.interpolation_dt),ticks,dt,np.array(limits['velocity_limits']),np.array(limits['acceleration_limits']),initial_velocity=dq0,final_velocity=dq1)),
                         ('path_parameterization',lambda:parameterize_path(raw,ticks,dt,np.array(limits['velocity_limits']),np.array(limits['acceleration_limits']),initial_velocity=dq0,final_velocity=dq1)),
                         ('boundary_fit',lambda:boundary_fitted_path(raw,ticks,dt,np.array(limits['velocity_limits']),np.array(limits['acceleration_limits']),initial_velocity=dq0,final_velocity=dq1))]:
            try:
                path,timing=fit()
                invalid=[k for k,q in enumerate(path) if not self.valid(q)]
                if invalid and self.carry and not path_cover_attempted:
                    path_cover_attempted=True
                    if self.refine_path_cover(path,invalid):
                        invalid=[k for k,q in enumerate(path) if not self.valid(q)]
                if invalid and self.carry and name=='boundary_fit':
                    clearance,gradient=self.path_clearance_gradient(path)
                    np.savez_compressed(self.out/'collision_time_fit_input.npz',path=path,
                        clearance=clearance,gradient=gradient,initial_velocity=dq0,final_velocity=dq1,dt=dt,
                        velocity_limits=np.array(limits['velocity_limits']),acceleration_limits=np.array(limits['acceleration_limits']))
                    fit_args=(path,dt,np.array(limits['velocity_limits']),np.array(limits['acceleration_limits']),
                              dq0,dq1,self.path_clearance_gradient)
                    try:path,timing=collision_fitted_path(*fit_args)
                    except RuntimeError:
                        if goal_state is None or not goal_state.get('unvalidated_oracle_boundary'):raise
                        from multipath.execution.mounted_scene import pose_matrix
                        from multipath.validation.physics import OBJECT_POSITION_TOLERANCE_M,OBJECT_ROTATION_TOLERANCE_RAD
                        source=np.asarray(goal_state['source_tool_boundary']);held=np.asarray(start['hand_object_transform'])
                        def condition(q):
                            tool=pose_matrix(coordinates_pose(q))@held
                            position=np.sum((tool[:3,3]-source[:3,3])**2)/OBJECT_POSITION_TOLERANCE_M**2
                            rotation=Rotation.from_matrix(source[:3,:3].T@tool[:3,:3]).magnitude()**2/OBJECT_ROTATION_TOLERANCE_RAD**2
                            return .98**2-position-rotation
                        path,timing=collision_fitted_path(*fit_args,boundary_condition=condition)
                    invalid=[k for k,q in enumerate(path) if not self.valid(q)]
                if invalid:raise RuntimeError(f'collision at fitted tick {invalid[0]}')
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
        if timing.get('canonical_boundary_jointly_fitted'):
            # Update only an unvalidated command proposal. The executor still
            # verifies the measured tool against the same source condition.
            goal_state.update(wrist_xyz_wxyz=poses[-1].tolist(),wrist_twist=commands[-1,19:25].tolist())
        timing.update(planner='cuRobo_virtual_6DOF',hand_collision_spheres=self.sphere_count,carried_object_collision=self.carry,
            allowed_attached_object_contacts=list(self.allowed_object_contacts),
            geometry_solver=geometry_solver,curobo_optimize_dt=False,
            curobo_optimization_budget_s=(ticks*dt if self.carry else 31/scene.manifest['fps']),
            collision_cover_refined_over_fitted_path=getattr(self,'path_cover_refined',False),
            reference_tracking=False,source_time_stretch=False,arm_model_used=False,tracking_margin_m=scene.planning_tracking_margin_m,
            previous_command_tracking_margin_m=scene.previous_command_tracking_margin_m,
            hand_boundary_motion=bool(np.any(np.abs(hands-hands[0])>1e-8)),
            planned_hand_geometry='sampled_free_finger_sweep' if scene.planned_hand_sweep_samples>1 else 'entry_posture',
            planned_hand_sweep_samples=scene.planned_hand_sweep_samples,
            hand_boundary_collision_validation='measured_geometry_each_physics_step')
        return commands,timing
