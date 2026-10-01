"""cuRobo waypoint search with wrist-only virtual path exclusions.

All cuRobo subplans retain full robot, hand, payload, and real-world collision
checks. Virtual exclusions reject candidate wrist curves, not body geometry.
"""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .path_exclusion import resample, compare_path, box_clearance


def wrist_mount_from_start(ee_position, ee_rotation, wrist_position, wrist_rotation):
    """Measured fixed transform from cuRobo's EE link to the XHand wrist."""
    return (ee_rotation.inv().apply(np.asarray(wrist_position)-ee_position),
            ee_rotation.inv()*wrist_rotation)


def wrist_targets_to_ee(positions, rotations, offset_position, offset_rotation):
    ee_rotations=rotations*offset_rotation.inv()
    return np.asarray(positions)-ee_rotations.apply(offset_position),ee_rotations


def ee_path_to_wrist(positions, rotations, offset_position):
    return np.asarray(positions)+rotations.apply(offset_position)


def detour_waypoints(start, end, exclusions, clearance=.008):
    old=np.asarray(exclusions['sources'][-1]['wrist_path_xyz'])
    axis=np.asarray(end)-start;axis/=max(np.linalg.norm(axis),1e-9)
    xy=np.array([-axis[1],axis[0],0.])
    xy_norm=np.linalg.norm(xy)
    half=max(max(b['dims'])/2 for b in exclusions['cuboid'].values())
    if xy_norm>1e-5:
        xy/=xy_norm
        midpoint=resample(old,101)[50]
        minimum=max(half+clearance+.02,
                    1.2*exclusions.get('minimum_arc_distance_m',0.))
        for radius in (minimum,1.3*minimum):
            for side in (-1.,1.):
                fractions=np.array([.25,.5,.75])
                arc=(1-fractions[:,None])*np.asarray(start)+fractions[:,None]*np.asarray(end)
                arc+=side*radius*np.sin(np.pi*fractions)[:,None]*xy
                if box_clearance(arc,exclusions['cuboid']).min()>clearance:
                    yield arc
        for radius in (minimum,1.3*minimum):
            for side in (-1.,1.):
                point=midpoint+side*radius*xy
                if box_clearance(point[None],exclusions['cuboid']).min()>clearance:
                    yield np.asarray([point])
    basis=np.eye(3)[np.argmin(np.abs(axis))]
    n1=np.cross(axis,basis);n1/=np.linalg.norm(n1);n2=np.cross(axis,n1)
    center=resample(old,101)[[30,70]]
    for radius in (half+clearance,2*half+clearance,3*half+clearance):
        for angle in np.arange(8)*np.pi/4:
            offset=radius*(np.cos(angle)*n1+np.sin(angle)*n2)
            points=center+offset
            if box_clearance(points,exclusions['cuboid']).min()>clearance:
                yield points


def plan_wrist_detour(planner, start, exclusions, tensor, budget_s, solve,
                     goal_joints=None, live_wrist_pose=None):
    from curobo.types.math import Pose
    from curobo.types.robot import JointState
    from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
    initial_q=start.position.detach().cpu().numpy().reshape(-1,7)[0].copy()
    initial_state=JointState.from_position(tensor.to_device(initial_q.copy())[None],
                                           joint_names=planner.joint_names)
    state=planner.compute_kinematics(initial_state)
    first=state.ee_pos_seq.detach().cpu().numpy().reshape(-1,3)[0].copy()
    quat=state.ee_quat_seq.detach().cpu().numpy().reshape(-1,4)[0].copy()
    begin=Rotation.from_quat(quat[[1,2,3,0]])
    if live_wrist_pose is None:
        raise ValueError('Measured wrist pose is required for wrist-space exclusions')
    live_wrist_pose=np.asarray(live_wrist_pose,dtype=float)
    wrist_begin=Rotation.from_quat(live_wrist_pose[[4,5,6,3]])
    wrist_goal_pose=np.asarray(exclusions['boundary_wrist_xyz_wxyz'],dtype=float)
    wrist_goal=Rotation.from_quat(wrist_goal_pose[[4,5,6,3]])
    # The cuRobo EE link and the rendered XHand wrist are separated by a
    # fixed mount transform. Generate and score all arcs in wrist world space.
    wrist_offset_position,wrist_offset_rotation=wrist_mount_from_start(
        first,begin,live_wrist_pose[:3],wrist_begin)
    trials=[];best=None
    for trial,waypoints in enumerate(detour_waypoints(live_wrist_pose[:3],wrist_goal_pose[:3],exclusions)):
        wrist_rotations=Slerp([0,1],Rotation.from_quat([wrist_begin.as_quat(),wrist_goal.as_quat()]))(
            np.linspace(0,1,len(waypoints)+2)[1:])
        wrist_positions=np.vstack([waypoints,wrist_goal_pose[:3]])
        ee_positions,rotations=wrist_targets_to_ee(
            wrist_positions,wrist_rotations,wrist_offset_position,wrist_offset_rotation)
        current=JointState.from_position(tensor.to_device(initial_q.copy())[None],
                                         joint_names=planner.joint_names)
        pieces=[];ok=True;subplan_start_errors=[]
        for waypoint_index,(position,rotation) in enumerate(zip(ee_positions,rotations)):
            pose=Pose(position=tensor.to_device(position.copy())[None],quaternion=tensor.to_device(rotation.as_quat()[[3,0,1,2]])[None])
            try:
                seed=current.position.detach().cpu().numpy().reshape(-1,7)[0].copy()
                qgoal=np.asarray(goal_joints,dtype=float).copy() if waypoint_index==len(waypoints) and goal_joints is not None else solve(position,rotation,seed)
                joint_goal=JointState.from_position(tensor.to_device(qgoal.copy())[None],joint_names=planner.joint_names)
                valid,_=planner.check_start_state(joint_goal)
            except RuntimeError:
                valid=False
            current_q=current.position.detach().cpu().numpy().reshape(-1,7)[0].copy()
            if valid:
                result=planner.plan_single_js(current,joint_goal,MotionGenPlanConfig(max_attempts=3,enable_graph_attempt=1))
            else:
                result=planner.plan_single(current,pose,MotionGenPlanConfig(max_attempts=3,enable_graph_attempt=1))
            if not bool(result.success.item()):
                trials.append({'trial':trial,'status':str(result.status)});ok=False;break
            part=result.get_interpolated_plan().position.detach().cpu().numpy().reshape(-1,7).copy()
            subplan_start_errors.append(float(np.max(np.abs(
                part[0]-current_q))))
            pieces.append(part if not pieces else part[1:])
            current=JointState.from_position(tensor.to_device(part[-1].copy())[None],joint_names=planner.joint_names)
        if not ok:continue
        path=np.concatenate(pieces)
        state=planner.compute_kinematics(JointState.from_position(tensor.to_device(path.copy()),joint_names=planner.joint_names))
        ee_points=state.ee_pos_seq.detach().cpu().numpy().reshape(-1,3)
        ee_quats=state.ee_quat_seq.detach().cpu().numpy().reshape(-1,4)
        ee_rotations=Rotation.from_quat(ee_quats[:,[1,2,3,0]])
        points=ee_path_to_wrist(ee_points,ee_rotations,wrist_offset_position)
        metrics=compare_path(points,exclusions)
        metrics['planned_start_wrist_xyz']=points[0].tolist()
        metrics['planned_end_wrist_xyz']=points[-1].tolist()
        metrics['measured_start_wrist_xyz']=live_wrist_pose[:3].tolist()
        metrics['measured_boundary_wrist_xyz']=wrist_goal_pose[:3].tolist()
        metrics['joint_start_error_rad']=float(np.max(np.abs(path[0]-initial_q)))
        metrics['max_subplan_start_error_rad']=max(subplan_start_errors)
        if goal_joints is not None:
            metrics['joint_goal_error_rad']=float(np.max(np.abs(path[-1]-np.asarray(goal_joints,dtype=float))))
        limits=np.array([2.175]*4+[2.61]*3)
        # A lower bound for ranking fixed-budget routes; actual tracking is
        # measured separately and can still reject the entire rollout.
        minimum_time=float(np.max(np.abs(np.diff(path,axis=0))/(.90*limits),axis=1).sum())
        trials.append({'trial':trial,'status':'candidate','estimated_minimum_time_s':minimum_time,**metrics})
        # The planned wrist must remain outside the virtual box. The measured
        # PhysX wrist later has the stricter 2 mm clearance requirement.
        if (metrics['max_subplan_start_error_rad']<.02 and
                np.linalg.norm(points[0]-live_wrist_pose[:3])<.01 and
                metrics['actual_wrist_min_exclusion_clearance_m']>0 and
                metrics['actual_arc_pass'] and metrics['smooth_arc_pass']):
            if best is None or minimum_time<best[0]:best=(minimum_time,path.copy(),trial)
            if minimum_time<=budget_s*.9:break

    if best is None:raise RuntimeError(f'No cuRobo wrist detour; trials={trials}')
    if best[0]>budget_s:raise RuntimeError(f'Wrist detour needs at least {best[0]:.4f}s but budget is {budget_s:.4f}s')
    return best[1],{'selected_trial':best[2],'estimated_minimum_time_s':best[0],'trials':trials,
                    'wrist_offset_from_planner_ee_xyz':wrist_offset_position.tolist(),
                    'wrist_offset_from_planner_ee_xyzw':wrist_offset_rotation.as_quat().tolist(),
                    'planned_virtual_clearance_threshold_m':0.,
                    'virtual_exclusion_scope':'wrist','physical_collision_scope':'robot_hand_payload'}


def resample_with_velocity_budget(path, ticks, limits, dt=None):
    """Allocate equal budget using joint travel, not planner sample indices.

    Stops and unequal subplan lengths must not compress motion into a few
    physics ticks. The caller checks the route's required total duration.
    """
    path=np.asarray(path);limits=np.asarray(limits)
    durations=np.max(np.abs(np.diff(path,axis=0))/limits,axis=1)
    clock=np.r_[0.,np.cumsum(durations)]
    keep=np.r_[True,np.diff(clock)>1e-10]
    if clock[-1]<1e-10:return np.repeat(path[:1],ticks,axis=0)
    u=np.linspace(0,1,ticks)
    if dt is not None:
        available=(ticks-1)*dt
        if clock[-1]>available+1e-8:
            raise RuntimeError(f'Joint travel needs {clock[-1]:.4f}s but usable budget is {available:.4f}s')
        # Smoothstep peaks at 1.5 times average speed. Blend only as much
        # easing as the available time permits; never exceed speed limits.
        blend=min(1.,max(0.,2*(available/clock[-1]-1)))
        u=(1-blend)*u+blend*(3*u*u-2*u*u*u)
    query=u*clock[-1]
    return np.stack([np.interp(query,clock[keep],path[keep,k]) for k in range(path.shape[1])],axis=1)
