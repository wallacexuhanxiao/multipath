"""cuRobo waypoint search with wrist-only virtual path exclusions.

All cuRobo subplans retain full robot, hand, payload, and real-world collision
checks. Virtual exclusions reject candidate wrist curves, not body geometry.
"""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .path_exclusion import resample, compare_path, box_clearance


def detour_waypoints(start, end, exclusions, clearance=.008):
    old=np.asarray(exclusions['sources'][-1]['wrist_path_xyz'])
    axis=np.asarray(end)-start;axis/=max(np.linalg.norm(axis),1e-9)
    basis=np.eye(3)[np.argmin(np.abs(axis))]
    n1=np.cross(axis,basis);n1/=np.linalg.norm(n1);n2=np.cross(axis,n1)
    half=max(max(b['dims'])/2 for b in exclusions['cuboid'].values())
    center=resample(old,101)[[30,70]]
    for radius in (half+clearance,2*half+clearance,3*half+clearance):
        for angle in np.arange(8)*np.pi/4:
            offset=radius*(np.cos(angle)*n1+np.sin(angle)*n2)
            points=center+offset
            if box_clearance(points,exclusions['cuboid']).min()>clearance:
                yield points


def plan_wrist_detour(planner, start, goal_position, goal_rotation, exclusions, tensor, budget_s, solve, goal_joints=None):
    from curobo.types.math import Pose
    from curobo.types.robot import JointState
    from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
    state=planner.compute_kinematics(start)
    first=state.ee_pos_seq.detach().cpu().numpy().reshape(-1,3)[0].copy()
    quat=state.ee_quat_seq.detach().cpu().numpy().reshape(-1,4)[0].copy()
    begin=Rotation.from_quat(quat[[1,2,3,0]])
    rotations=Slerp([0,1],Rotation.from_quat([begin.as_quat(),goal_rotation.as_quat()]))([.3,.7,1])
    trials=[];best=None
    for trial,waypoints in enumerate(detour_waypoints(first,goal_position,exclusions)):
        current=start;pieces=[];ok=True
        for waypoint_index,(position,rotation) in enumerate(zip([*waypoints,goal_position],rotations)):
            pose=Pose(position=tensor.to_device(position.copy())[None],quaternion=tensor.to_device(rotation.as_quat()[[3,0,1,2]])[None])
            try:
                seed=current.position.detach().cpu().numpy().reshape(-1,7)[0].copy()
                qgoal=np.asarray(goal_joints,dtype=float).copy() if waypoint_index==2 and goal_joints is not None else solve(position,rotation,seed)
                joint_goal=JointState.from_position(tensor.to_device(qgoal.copy())[None],joint_names=planner.joint_names)
                valid,_=planner.check_start_state(joint_goal)
            except RuntimeError:
                valid=False
            if valid:
                result=planner.plan_single_js(current,joint_goal,MotionGenPlanConfig(max_attempts=3,enable_graph_attempt=1))
            else:
                result=planner.plan_single(current,pose,MotionGenPlanConfig(max_attempts=3,enable_graph_attempt=1))
            if not bool(result.success.item()):
                trials.append({'trial':trial,'status':str(result.status)});ok=False;break
            part=result.get_interpolated_plan().position.detach().cpu().numpy().reshape(-1,7).copy()
            pieces.append(part if not pieces else part[1:])
            current=JointState.from_position(tensor.to_device(part[-1].copy())[None],joint_names=planner.joint_names)
        if not ok:continue
        path=np.concatenate(pieces)
        state=planner.compute_kinematics(JointState.from_position(tensor.to_device(path.copy()),joint_names=planner.joint_names))
        points=state.ee_pos_seq.detach().cpu().numpy().reshape(-1,3)
        metrics=compare_path(points,exclusions)
        limits=np.array([2.175]*4+[2.61]*3)
        # A lower bound for ranking fixed-budget routes; actual tracking is
        # measured separately and can still reject the entire rollout.
        minimum_time=float(np.max(np.abs(np.diff(path,axis=0))/(.90*limits),axis=1).sum())
        trials.append({'trial':trial,'status':'candidate','estimated_minimum_time_s':minimum_time,**metrics})
        if metrics['actual_wrist_min_exclusion_clearance_m']>.005:
            if best is None or minimum_time<best[0]:best=(minimum_time,path.copy(),trial)
            if minimum_time<=budget_s*.9:break

    if best is None:raise RuntimeError(f'No cuRobo wrist detour; trials={trials}')
    if best[0]>budget_s:raise RuntimeError(f'Wrist detour needs at least {best[0]:.4f}s but budget is {budget_s:.4f}s')
    return best[1],{'selected_trial':best[2],'estimated_minimum_time_s':best[0],'trials':trials,
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
