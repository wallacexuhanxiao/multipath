"""Cheap necessary bridge checks before cuRobo initialization."""
import numpy as np
from scipy.spatial.transform import Rotation
from multipath.planning.floating import pose_coordinates


def displacement_bounds(v0,v1,budget,velocity_limits,acceleration_limits):
    """Exact scalar displacement envelopes with both endpoint velocities.

    Integrate the upper/lower piecewise-linear velocity envelopes; checking
    distance and velocity change separately misses adverse initial momentum.
    """
    low=[];high=[]
    for first,last,vmax,amax in zip(v0,v1,velocity_limits,acceleration_limits):
        values=[]
        for upper in (False,True):
            lines=[(0,vmax),(amax,first),(-amax,last+amax*budget)] if upper else [(0,-vmax),(-amax,first),(amax,last-amax*budget)]
            knots=[0.,budget]
            for i,(a,b) in enumerate(lines):
                for c,d in lines[i+1:]:
                    if a!=c:
                        t=(d-b)/(a-c)
                        if 0<t<budget:knots.append(t)
            knots=np.array(sorted(set(knots)));curve=np.array([a*knots+b for a,b in lines])
            speed=np.min(curve,axis=0) if upper else np.max(curve,axis=0)
            values.append(float(np.sum(np.diff(knots)*(speed[:-1]+speed[1:])/2)))
        low.append(values[0]);high.append(values[1])
    return np.array(low),np.array(high)


def coordinate_velocity(pose, twist, near=None):
    pose=np.asarray(pose,dtype=float);twist=np.asarray(twist,dtype=float)
    q=pose_coordinates(pose,near);eps=1e-5;next_pose=pose.copy()
    next_pose[:3]+=eps*twist[:3]
    next_pose[3:]=(Rotation.from_rotvec(eps*twist[3:])*Rotation.from_quat(pose[[4,5,6,3]])).as_quat()[[3,0,1,2]]
    return (pose_coordinates(next_pose,q)-q)/eps


def bridge_precheck(start,goal,budget,planning):
    """Necessary conditions only; passing still requires planning and physics.

    Independent Anchor success is not a proof of fixed-time compatibility.
    These limits are the configured planning budgets, not hardware ratings.
    """
    q0=pose_coordinates(start['wrist_xyz_wxyz']);q1=pose_coordinates(goal['wrist_xyz_wxyz'],q0)
    v0=coordinate_velocity(start['wrist_xyz_wxyz'],start['wrist_twist'],q0)
    v1=coordinate_velocity(goal['wrist_xyz_wxyz'],goal['wrist_twist'],q1)
    vmax=np.asarray(planning['velocity_limits']);amax=np.asarray(planning['acceleration_limits'])
    time_position=float(np.max(np.abs(q1-q0)/vmax))
    time_velocity=float(np.max(np.abs(v1-v0)/amax))
    low,high=displacement_bounds(v0,v1,budget,vmax,amax)
    hand_delta=float(np.max(np.abs(np.asarray(start['hand_q'])-goal['hand_q'])))
    reasons=[]
    if start['contact']['mode']!=goal['contact']['mode']:reasons.append('contact_mode_change_requires_anchor')
    if start['scene_id']!=goal['scene_id']:reasons.append('scene_mismatch')
    if start['contact']['mode']=='grasped' and hand_delta>.10:
        reasons.append('frozen_grasp_hand_boundary_mismatch')
    if time_position>budget+1e-8:reasons.append('displacement_exceeds_source_velocity_budget')
    if np.any(np.abs(v0)>vmax*1.0001):reasons.append('initial_velocity_exceeds_planning_budget')
    if np.any(np.abs(v1)>vmax*1.0001):reasons.append('terminal_velocity_exceeds_planning_budget')
    if time_velocity>budget+1e-8:reasons.append('boundary_velocity_change_exceeds_acceleration_budget')
    if np.any(q1-q0<low-1e-6) or np.any(q1-q0>high+1e-6):
        reasons.append('displacement_exceeds_acceleration_reachability')
    return dict(feasible=not reasons,reasons=reasons,budget_s=float(budget),
                minimum_duration_velocity_only_s=time_position,
                minimum_duration_velocity_change_only_s=time_velocity,
                coordinate_displacement=(q1-q0).tolist(),initial_coordinate_velocity=v0.tolist(),
                terminal_coordinate_velocity=v1.tolist(),hand_q_max_difference_rad=hand_delta,
                reachable_displacement_lower=low.tolist(),reachable_displacement_upper=high.tolist(),
                limits_provenance='configured nominal planning budget; not hardware ratings')


def project_boundary_target(start,target,budget,planning):
    """A deterministic resynthesis target, never a physical state artifact.

    Project the oracle target into its necessary joint-time envelope. The
    caller must still check the unchanged oracle pose condition and run cuRobo.
    This does not modify the validated Anchor or relax its source condition.
    """
    from multipath.planning.floating import coordinates_pose
    goal=dict(target);q0=pose_coordinates(start['wrist_xyz_wxyz'])
    v0=coordinate_velocity(start['wrist_xyz_wxyz'],start['wrist_twist'],q0)
    vmax=np.asarray(planning['velocity_limits'])*.98
    amax=np.asarray(planning['acceleration_limits'])*.95
    if np.any(np.abs(v0)>vmax):return goal
    desired=pose_coordinates(target['wrist_xyz_wxyz'],q0)
    for _ in range(4):
        v1=coordinate_velocity(goal['wrist_xyz_wxyz'],goal['wrist_twist'],q0)
        low,high=displacement_bounds(v0,v1,budget,vmax,amax)
        if np.any(low>high):return goal
        q=q0+np.clip(desired-q0,low,high)
        goal['wrist_xyz_wxyz']=coordinates_pose(q).tolist()
    goal['state_origin']='unvalidated_source_conditioned_boundary_proposal'
    goal['physically_acquired_state']=False
    return goal
