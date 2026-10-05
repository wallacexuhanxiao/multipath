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
    curve=CubicHermiteSpline([0.,duration],np.stack((q0,q1)),
                            np.stack((start['hand_dq'],goal['hand_dq'])))
    time=np.arange(ticks+1)*scene.dt;hand=curve(time)
    speed=np.maximum(np.abs(curve(time,1)),np.vstack((np.zeros(12),np.abs(np.diff(hand,axis=0)/scene.dt))))
    if np.any(hand<scene.hand_limits[:,0]-1e-6) or np.any(hand>scene.hand_limits[:,1]+1e-6) or np.any(speed>scene.hand_velocity*1.0001):
        raise RuntimeError('RELAXED hand boundary cannot fit source-time joint limits')
    return hand


def cover_points(points,cell=.025):
    center=points.mean(0);_,_,axes=np.linalg.svd(points-center,full_matrices=False)
    coords=(points-center)@axes.T;lo=coords.min(0);hi=coords.max(0)
    counts=np.maximum(1,np.ceil((hi-lo)/cell).astype(int));width=(hi-lo)/counts
    radius=float(np.linalg.norm(width)*.5+.001)
    return [{'center':(center+(lo+(np.asarray(i)+.5)*width)@axes).tolist(),'radius':radius} for i in np.ndindex(*counts)]


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
    for prim in Usd.PrimRange.Stage(scene.world.stage,Usd.TraverseInstanceProxies()):
        if not str(prim.GetPath()).startswith(scene.root+'/') or not prim.IsA(UsdGeom.Mesh):continue
        if 'collision' in str(prim.GetPath()).lower():continue
        points=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get());trans=np.asarray(cache.GetLocalToWorldTransform(prim)).T
        scale=np.linalg.norm(trans[:3,:3],axis=0);local=cover_points(points*scale)
        measured.append((prim,scale,local))
        world_points=points@trans[:3,:3].T+trans[:3,3]
        spheres+=cover_points(rot.inv().apply(world_points-pose[:3]))
    scene.measured_hand_collision_meshes=measured
    if carry:
        p,q=scene.objects['tool'].get_world_pose()
        points=Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).apply(scene.meshes['tool'].vertices)+p
        spheres+=cover_points(rot.inv().apply(points-pose[:3]))
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
        planning_tracking_margin_m=0.,carried_object=carry),indent=2))
    return {'kinematics':kin},len(spheres)


class FloatingPlanner:
    def __init__(self,scene,carry,out):
        from curobo.types.base import TensorDeviceType
        from curobo.wrap.reacher.motion_gen import MotionGen,MotionGenConfig
        from curobo.geom.sdf.world import CollisionCheckerType
        self.scene=scene;self.tensor=TensorDeviceType(device='cuda:0');self.carry=carry;self.out=Path(out)
        cfg,self.sphere_count=robot_model(scene,carry,out)
        live={'cuboid':{'table':scene.table}}
        for role in (['target'] if carry else ['tool','target']):
            p,q=scene.objects[role].get_world_pose();bounds=scene.meshes[role].bounds;rot=Rotation.from_quat(np.asarray(q)[[1,2,3,0]])
            live['cuboid'][role]={'dims':(bounds[1]-bounds[0]).tolist(),'pose':np.r_[p+rot.apply(bounds.mean(0)),q].tolist()}
        self.obstacle_roles=['target'] if carry else ['tool','target']
        self.nominal_spheres=cfg['kinematics']['collision_spheres']['wrist']
        self.live=live
        import json
        (Path(out)/'world_geometry.json').write_text(json.dumps(live,indent=2))
        self.planner=MotionGen(MotionGenConfig.load_from_robot_config(cfg,live,tensor_args=self.tensor,interpolation_dt=scene.dt,
            self_collision_check=False,self_collision_opt=False,collision_checker_type=CollisionCheckerType.PRIMITIVE,use_cuda_graph=False))
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

    def measured_valid(self):
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
                current=cover_points(Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).apply(self.scene.meshes['tool'].vertices)+p)
                groups.append(np.asarray([s['center'] for s in current]));sizes.extend(s['radius'] for s in current)
            centers=np.concatenate(groups);radii=np.asarray(sizes)
        boxes=dict(self.live['cuboid'])
        for role in getattr(self,'obstacle_roles',[]):
            p,q=self.scene.objects[role].get_world_pose();bounds=self.scene.meshes[role].bounds
            current=Rotation.from_quat(np.asarray(q)[[1,2,3,0]])
            boxes[role]={'dims':(bounds[1]-bounds[0]).tolist(),'pose':np.r_[p+current.apply(bounds.mean(0)),q].tolist()}
        for box in boxes.values():
            b=np.asarray(box['pose']);rotation=Rotation.from_quat(b[[4,5,6,3]])
            local=rotation.inv().apply(centers-b[:3]);outside=np.maximum(np.abs(local)-np.asarray(box['dims'])/2,0.)
            if np.any(np.linalg.norm(outside,axis=1)<radii):return False
        return True

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
        if not bool(result.success.item()):raise RuntimeError(f'Floating cuRobo failed: {result.status}')
        raw=result.get_interpolated_plan().position.detach().cpu().numpy().reshape(-1,6).copy()
        limits=scene.cfg['planning'];dt=scene.dt
        # Impose command boundaries before time fitting and collision checks;
        # these are planned targets, never assignments to simulator state.
        if goal_twist is not None:
            raw[0]=q0;raw[-1]=q1
        np.savez_compressed(self.out/'raw_geometric_path.npz',
                            positions=raw,initial_velocity=dq0,final_velocity=np.zeros(6) if dq1 is None else dq1)
        try:path,timing=fixed_budget_path(raw,float(result.interpolation_dt),ticks,dt,np.array(limits['velocity_limits']),np.array(limits['acceleration_limits']),initial_velocity=dq0,final_velocity=dq1)
        except RuntimeError as compression_error:
            try:path,timing=parameterize_path(raw,ticks,dt,np.array(limits['velocity_limits']),np.array(limits['acceleration_limits']),initial_velocity=dq0,final_velocity=dq1)
            except RuntimeError as path_error:
                path,timing=boundary_fitted_path(raw,ticks,dt,np.array(limits['velocity_limits']),np.array(limits['acceleration_limits']),initial_velocity=dq0,final_velocity=dq1)
                timing['previous_fit_rejections']=[str(compression_error),str(path_error)]
        for q in path:
            if not self.valid(q):raise RuntimeError('Floating resampled path collision')
        poses=np.asarray([coordinates_pose(q) for q in path])
        from multipath.anchors.floating import controls_from_poses
        # Preserve the active drive target/preload, rather than commanding the
        # measured contact-deflected hand posture with zero closing error.
        hands=relaxed_hand_path(scene,start,goal_state,ticks)
        commands=controls_from_poses(poses,hands,dt)
        timing.update(planner='cuRobo_virtual_6DOF',hand_collision_spheres=self.sphere_count,carried_object_collision=self.carry,
            reference_tracking=False,source_time_stretch=False,arm_model_used=False,tracking_margin_m=scene.planning_tracking_margin_m,
            previous_command_tracking_margin_m=scene.previous_command_tracking_margin_m,
            hand_boundary_motion=bool(np.any(np.abs(hands-hands[0])>1e-8)),
            planned_hand_geometry='entry_posture',
            hand_boundary_collision_validation='measured_geometry_each_physics_step')
        return commands,timing
