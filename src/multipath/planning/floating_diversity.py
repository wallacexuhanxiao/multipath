"""Interval-local wrist exclusions, cuRobo detours and immutable source time.

Artificial obstacles constrain the wrist curve only. The existing cuRobo
model still checks the hand, carried object and physical world separately.
"""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from multipath.path_exclusion import (
    resample, interval_distance_profile, tapered_obstacles_along_path,
    box_clearance, compare_path,
)
from multipath.planning.floating import pose_coordinates, coordinates_pose, relaxed_hand_path
from multipath.planning.timing import boundary_fitted_path, fixed_budget_path
from multipath.anchors.floating import controls_from_poses


def interval_exclusions(paths, window, budget_s, limits, multiplier=1.,free_budget_s=None,boundary_velocity=None):
    """Use only this interval's accepted actual paths; taper at both ends.

    A short interval has less lateral acceleration authority. Cap the spatial
    distance-based size by the same source-time velocity/acceleration budgets.
    This is a proposal size, never a relaxation of physical acceptance.
    """
    free_budget_s=budget_s if free_budget_s is None else free_budget_s
    movable_fraction=np.clip(free_budget_s/budget_s,0.,1.)
    def movable_prefix(points):
        points=np.asarray(points)
        stop=max(2,1+int(np.floor((len(points)-1)*movable_fraction)))
        return points[:stop]
    profile=interval_distance_profile(movable_prefix(paths[0]['points']),peak_ratio=.35,
                                     target_arc_ratio=.35)
    time_cap=min(min(limits['velocity_limits'][:3])*free_budget_s/(2*np.pi),
                 min(limits['acceleration_limits'][:3])*free_budget_s**2/(4*np.pi**2))
    peak=min(profile['requested_max_half_size_m'],time_cap)*multiplier
    clearance=max(.00025,min(.002,peak/5))
    boxes={};planning_boxes={};sources=[]
    # Common boundary neighborhoods need room for velocity continuity. Their
    # radius scales with the interval rather than a sample's frame indices.
    def protect(points):
        points=np.asarray(points);chord=points[-1]-points[0]
        radius=np.full(2,max(.008,.25*np.linalg.norm(chord)))
        if boundary_velocity is not None:
            residual=np.asarray(boundary_velocity)-chord/budget_s
            turnaround=np.linalg.norm(residual**2/(2*np.asarray(limits['acceleration_limits'][:3])),axis=1)
            radius=np.maximum(radius,turnaround)
        return np.c_[points[[0,-1]],radius]
    protection=protect(movable_prefix(paths[0]['points']))
    for i,path in enumerate(paths):
        generated,stats=tapered_obstacles_along_path(
            movable_prefix(path['points']),peak,boundary_spheres=protection,
            clearance=clearance,oriented=True)
        boxes.update({f'path_exclusion_{i}_{j}':b for j,b in enumerate(generated.values())})
        sources.append(dict(run=path['id'],wrist_path_xyz=resample(path['points']).tolist(),
                            source_time_wrist_path_xyz=np.asarray(path['points']).tolist(),
                            placement_diagnostics=stats))
        # A held object's executed wrist can lag its drive by centimetres.
        # Exclude old drive curves in planning, old actual curves in PhysX.
        # These are two distinct evidence spaces with the same source budget.
        command=np.asarray(path.get('command_points',path['points']))
        protected=protect(movable_prefix(command))
        generated,command_stats=tapered_obstacles_along_path(movable_prefix(command),peak,boundary_spheres=protected,
                                                            clearance=clearance,oriented=True)
        planning_boxes.update({f'path_exclusion_{i}_{j}':b for j,b in enumerate(generated.values())})
        sources[-1].update(command_wrist_path_xyz=resample(command).tolist(),source_time_command_path_xyz=command.tolist(),
                          command_placement_diagnostics=command_stats)
    return dict(window=list(window),cuboid=boxes,planning_cuboid=planning_boxes,sources=sources,
                minimum_arc_distance_m=0.,virtual_exclusion_scope='wrist',
                clearance_threshold_m=clearance,
                require_simple_arc=True,box_orientation='path_tangent',
                physical_collision_scope='hand_carried_object_world',
                distance_profile=profile,source_time_cap_m=time_cap,
                free_diversity_budget_s=free_budget_s,
                shared_terminal_fraction=1-float(movable_fraction),
                shared_handoff_history_excluded_from_obstacle_placement=True,
                requested_peak_half_size_m=peak,size_multiplier=multiplier,
                size_profile='small_large_small',intervals_share_exclusions=False,
                boundary_velocity=None if boundary_velocity is None else np.asarray(boundary_velocity).tolist(),
                boundary_protection_method='source_time_velocity_turnaround_envelope')


def actual_feasible_exclusions(exclusions,points,budget_s,limits):
    """Size virtual wrist exclusions to the achieved closed-loop detour.

    Physical constraints, source time and commands stay fixed. Every prior
    curve must remain excluded by a nonempty positive-clearance envelope.
    """
    original=compare_path(points,exclusions)
    if original['actual_wrist_exclusion_pass']:return exclusions,original
    paths=[dict(id=s['run'],points=np.asarray(s.get('source_time_wrist_path_xyz',s['wrist_path_xyz'])),
        command_points=np.asarray(s.get('source_time_command_path_xyz',s.get('command_wrist_path_xyz',s['wrist_path_xyz'])))) for s in exclusions['sources']]
    attempts=[]
    for reduction in (.5,.25,.125,.0625,.03125,.015625):
        try:
            candidate=interval_exclusions(paths,exclusions['window'],budget_s,limits,
                exclusions['size_multiplier']*reduction,exclusions.get('free_diversity_budget_s',budget_s),
                exclusions.get('boundary_velocity'))
        except ValueError:continue
        for key in ('arc_reference_length_m','arc_reference_method'):
            if key in exclusions:candidate[key]=exclusions[key]
        if any(compare_path(p['points'],candidate)['actual_wrist_exclusion_pass'] for p in paths):continue
        metrics=compare_path(points,candidate);attempts.append(dict(reduction=reduction,metrics=metrics))
        if metrics['actual_wrist_exclusion_pass']:
            candidate.update(actual_feedback_sizing=True,proposal_peak_half_size_m=exclusions['requested_peak_half_size_m'],
                proposal_metrics=original,sizing_attempts=attempts,
                previous_paths_remain_excluded=True,physical_constraints_changed=False)
            return candidate,metrics
    return exclusions,original


def boundary_history_ticks(scene,carry,ticks,has_next_anchor=True):
    """Keep moving drive history where a free approach establishes a grasp.

    This follows the canonical moving drive inside the source budget, with
    no pause. Carry bridges retain only the two pose/velocity samples.
    """
    if not has_next_anchor:return 0
    if carry:return min(2,ticks)
    from multipath.execution.transport_dynamics import TransportDynamics
    model=TransportDynamics(scene,False);cfg=scene.cfg['controller']
    response=max(np.sqrt(model.mass/cfg['wrist_kp']),
                 np.sqrt(np.max(np.linalg.eigvalsh(model.inertia))/cfg['wrist_kp_rot']))
    return max(2,min(ticks//3,int(np.ceil(3*response/scene.dt))))


DETOUR_DIRECTION_COUNT=8


def detour_directions(axis):
    axis=np.asarray(axis,dtype=float);axis/=max(np.linalg.norm(axis),1e-9)
    lateral=np.array([-axis[1],axis[0],0.])
    if np.linalg.norm(lateral)<1e-6:lateral=np.cross(axis,np.eye(3)[np.argmin(abs(axis))])
    lateral/=max(np.linalg.norm(lateral),1e-9)
    other=np.cross(axis,lateral)
    # Preserve the original seed indices; cover both missing lower diagonals.
    return [lateral,-lateral,other,-other,(lateral+other)/np.sqrt(2),
            (-lateral+other)/np.sqrt(2),(lateral-other)/np.sqrt(2),
            (-lateral-other)/np.sqrt(2)]


def detour_seeds(start,goal,exclusions):
    """Broad lateral seeds around accepted paths, with unchanged endpoints."""
    start=np.asarray(start);goal=np.asarray(goal)
    axis=goal-start;axis/=max(np.linalg.norm(axis),1e-9)
    lateral=np.array([-axis[1],axis[0],0.])
    if np.linalg.norm(lateral)<1e-6:
        lateral=np.cross(axis,np.eye(3)[np.argmin(np.abs(axis))])
    lateral/=max(np.linalg.norm(lateral),1e-9)
    other=np.cross(axis,lateral);other/=max(np.linalg.norm(other),1e-9)
    old=resample(exclusions['sources'][0]['wrist_path_xyz'],101)
    fractions=np.array([.25,.5,.75]);base=old[[25,50,75]].copy()
    # Correct different drive/measured endpoints without prescribing a human
    # trajectory. The seed is the previously successful robot path.
    base+=(1-fractions[:,None])*(start-old[0])+fractions[:,None]*(goal-old[-1])
    peak=exclusions['requested_peak_half_size_m']
    clearance=float(exclusions.get('clearance_threshold_m',.002))
    for amplitude in (1.4*(peak+clearance),2*(peak+clearance)):
        for direction in detour_directions(goal-start):
            points=base+amplitude*np.sin(np.pi*fractions)[:,None]*direction
            if box_clearance(points,exclusions['cuboid']).min()>clearance:
                yield points,dict(amplitude_m=amplitude,direction=direction.tolist())


def pose_velocity(pose,twist,near):
    pose=np.asarray(pose,dtype=float);twist=np.asarray(twist,dtype=float);eps=1e-4
    advanced=pose.copy();advanced[:3]+=eps*twist[:3]
    advanced[3:]=(Rotation.from_rotvec(eps*twist[3:])*
                  Rotation.from_quat(pose[[4,5,6,3]])).as_quat()[[3,0,1,2]]
    return (pose_coordinates(advanced,near)-near)/eps


def set_arc_reference(exclusions,start,goal_command,ticks,dt,limits):
    """Quality reference for a nominal straight bridge with moving endpoints.

    This uses robot boundary states only. It is not a shortest-path proof or
    a human-reference tracking objective. All physics gates remain separate.
    """
    q0=pose_coordinates(start['wrist_xyz_wxyz']);q1=pose_coordinates(goal_command[:7],q0)
    dq0=pose_velocity(start['wrist_xyz_wxyz'],start['wrist_twist'],q0)
    dq1=pose_velocity(goal_command[:7],goal_command[19:25],q1)
    path,_=boundary_fitted_path(np.vstack((q0,q1)),ticks,dt,
        np.asarray(limits['velocity_limits']),np.asarray(limits['acceleration_limits'])*1.00005,dq0,dq1)
    exclusions.update(arc_reference_length_m=float(np.linalg.norm(np.diff(path[:,:3],axis=0),axis=1).sum()),
        arc_reference_method='fixed_time_straight_bridge_with_actual_boundary_velocities')


def free_terminal_nominal(scene,start,ticks):
    """No next Anchor: derive a feasible braking target from the real release.

    This is a planning proposal, never an assigned state. cuRobo detour
    geometry, measured collision and the unchanged object task contract still
    decide acceptance. The old free-hand endpoint is not a task constraint.
    """
    from multipath.planning.bridge import terminal_stop_target
    limits=scene.cfg['planning'];goal=terminal_stop_target(start,ticks*scene.dt,scene.dt,limits)
    goal['hand_q']=list(start['command_hand_q']);goal['hand_dq']=np.zeros(12).tolist()
    q0=pose_coordinates(start['wrist_xyz_wxyz']);q1=pose_coordinates(goal['wrist_xyz_wxyz'],q0)
    dq0=pose_velocity(start['wrist_xyz_wxyz'],start['wrist_twist'],q0)
    path,_=boundary_fitted_path(np.vstack((q0,q1)),ticks,scene.dt,
        np.asarray(limits['velocity_limits']),np.asarray(limits['acceleration_limits'])*1.00005,dq0,np.zeros(6))
    poses=np.asarray([coordinates_pose(q) for q in path])
    hands=np.repeat(np.asarray(start['command_hand_q'])[None],ticks+1,axis=0)
    return controls_from_poses(poses,hands,scene.dt),goal


def dynamic_detour_seed(baseline,dt,limits,dq0,dq1,exclusions,index):
    """A broad seed inside the actual source-time reachability envelope.

    Convex velocity/acceleration bounds include the two moving boundaries.
    This only initializes cuRobo; collision and actual exclusion acceptance
    remain mandatory after geometric planning and time fitting.
    """
    from scipy.optimize import linprog
    nominal,_=boundary_fitted_path(baseline,len(baseline)-1,dt,
        np.asarray(limits['velocity_limits']),np.asarray(limits['acceleration_limits'])*1.00005,dq0,dq1)
    axis=nominal[-1,:3]-nominal[0,:3];axis/=max(np.linalg.norm(axis),1e-9)
    lateral=np.array([-axis[1],axis[0],0.])
    if np.linalg.norm(lateral)<1e-6:lateral=np.cross(axis,np.eye(3)[np.argmin(abs(axis))])
    lateral/=np.linalg.norm(lateral);other=np.cross(axis,lateral)
    directions=detour_directions(axis)
    direction=directions[index];n=len(nominal);phase=np.sin(np.linspace(0,np.pi,n))**2
    amplitude=2.5*(exclusions['requested_peak_half_size_m']+exclusions.get('clearance_threshold_m',.002))
    identity=np.eye(n);d1=np.diff(identity,axis=0);d2=np.diff(identity,n=2,axis=0)
    matrix=np.vstack((d1,-d1,d2,-d2));path=nominal.copy()
    for j in range(3):
        if abs(direction[j])<1e-8:continue
        bounds=list(zip(nominal[:,j]-amplitude*phase,nominal[:,j]+amplitude*phase))
        for k,value in [(0,nominal[0,j]),(1,nominal[0,j]+dq0[j]*dt),
                        (n-2,nominal[-1,j]-dq1[j]*dt),(n-1,nominal[-1,j])]:bounds[k]=(value,value)
        rhs=np.r_[np.full(2*(n-1),limits['velocity_limits'][j]*dt),
                  np.full(2*(n-2),limits['acceleration_limits'][j]*dt**2*1.00005)]
        result=linprog(-direction[j]*phase,A_ub=matrix,b_ub=rhs,bounds=bounds,method='highs')
        if not result.success:raise RuntimeError('No dynamic detour seed from measured moving boundaries')
        path[:,j]=result.x
    fractions=np.array([.075,.15,.25,.5,.75,.85,.925])
    indices=np.unique(np.clip(np.rint(fractions*(n-1)).astype(int),1,n-2));points=path[indices,:3]
    progress=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path[:,:3],axis=0),axis=1))]
    progress/=max(progress[-1],1e-12)
    return points,dict(amplitude_m=float(np.linalg.norm(path[:,:3]-nominal[:,:3],axis=1).max()),direction=direction.tolist(),
                       waypoint_source_fractions=(indices/(n-1)).tolist(),
                       waypoint_coordinates=path[indices].tolist(),
                       method='source_time_reachability_seed'),progress


def exclusion_fitted_path(raw,ticks,dt,limits,dq0,dq1,exclusions,direction,fixed_positions=None,desired=None,face_mode='seed',regularize=False):
    """Keep fixed-time fitting outside the chosen side of virtual obstacles.

    An unconstrained fit can erase a geometric detour to meet acceleration
    limits. Add separating bounds only at violated ticks; reject when these
    constraints and the unchanged source-time boundaries cannot coexist.
    """
    from scipy.optimize import linprog
    from scipy import sparse
    n=ticks+1;dof=raw.shape[1];size=n*dof
    if desired is None:
        desired=np.stack([np.interp(np.linspace(0,len(raw)-1,n),np.arange(len(raw)),raw[:,j]) for j in range(dof)],axis=1)
    identity=sparse.eye(n,format='csr')
    d1=sparse.diags([-np.ones(n-1),np.ones(n-1)],[0,1],shape=(n-1,n),format='csr')
    d2=sparse.diags([np.ones(n-2),-2*np.ones(n-2),np.ones(n-2)],[0,1,2],shape=(n-2,n),format='csr')
    velocity=sparse.kron(sparse.eye(dof),d1,format='csr')
    acceleration=sparse.kron(sparse.eye(dof),d2,format='csr')
    full_identity=sparse.eye(size,format='csr')
    d3=sparse.diags([-np.ones(n-3),3*np.ones(n-3),-3*np.ones(n-3),np.ones(n-3)],
                    [0,1,2,3],shape=(n-3,n),format='csr')
    jerk=sparse.kron(sparse.eye(dof),d3,format='csr');jerk_size=jerk.shape[0]
    variable_count=2*size+jerk_size
    derivative=sparse.vstack((velocity,-velocity,acceleration,-acceleration),format='csr')
    zero=sparse.csr_matrix((size,jerk_size));jerk_identity=sparse.eye(jerk_size,format='csr')
    matrix=sparse.vstack((sparse.hstack((derivative,sparse.csr_matrix((derivative.shape[0],size+jerk_size)))),
                          sparse.hstack((full_identity,-full_identity,zero)),
                          sparse.hstack((-full_identity,-full_identity,zero)),
                          sparse.hstack((jerk,sparse.csr_matrix((jerk_size,size)),-jerk_identity)),
                          sparse.hstack((-jerk,sparse.csr_matrix((jerk_size,size)),-jerk_identity))),format='csr')
    vr=np.repeat(limits['velocity_limits'],n-1)*dt
    # The final dynamics checker already permits 1e-4 numerical roundoff.
    # Quaternion -> Euler round trips in a passed baseline use part of it.
    ar=np.repeat(limits['acceleration_limits'],n-2)*dt**2*1.00005
    rhs=np.r_[vr,vr,ar,ar,desired.T.ravel(),-desired.T.ravel(),np.zeros(2*jerk_size)]
    bounds=[(None,None)]*size+[(0,None)]*(size+jerk_size)
    # L1 geometry alone returns bang-bang accelerations. Penalize acceleration
    # changes in nominal-budget units; this is a control cost, not a new gate.
    accel=np.asarray(limits['acceleration_limits'])
    objective=np.r_[np.zeros(size),np.repeat(1/(accel*(ticks*dt)**2),n),
                    np.repeat(1/(accel*dt**2),n-3)] if regularize else np.r_[
                        np.zeros(size),np.ones(size),np.zeros(jerk_size)]
    for j in range(dof):
        for k,value in [(0,raw[0,j]),(1,raw[0,j]+dq0[j]*dt),
                        (n-2,raw[-1,j]-dq1[j]*dt),(n-1,raw[-1,j])]:
            bounds[j*n+k]=(value,value)
    if fixed_positions is not None:
        for k,j in np.argwhere(np.isfinite(fixed_positions)):
            value=float(fixed_positions[k,j]);bounds[j*n+k]=(value,value)
    initial=linprog(objective,A_ub=matrix,b_ub=rhs,
                   bounds=bounds,method='highs',
                   options={'primal_feasibility_tolerance':1e-9,'dual_feasibility_tolerance':1e-9})
    if not initial.success:raise RuntimeError('No source-time fit preserving controller boundary history')
    path=initial.x[:size].reshape(dof,n).T
    separated=set();planes=[];offsets=[]
    clearance=float(exclusions.get('clearance_threshold_m',.002))
    clearance+=max(.0001,clearance/4)
    for _ in range(12):
        changed=False
        # Constrain the segments between drive samples too. Two endpoints
        # outside different faces do not prove their connecting line is free.
        spacing=max(.00005,clearance/2)
        lengths=np.linalg.norm(np.diff(path[:,:3],axis=0),axis=1)
        query=np.r_[np.concatenate([k+np.arange(max(1,int(np.ceil(length/spacing))))/
            max(1,int(np.ceil(length/spacing))) for k,length in enumerate(lengths)]),n-1]
        low=np.floor(query).astype(int);high=np.minimum(low+1,n-1);fraction=query-low
        sampled=(1-fraction[:,None])*path[low,:3]+fraction[:,None]*path[high,:3]
        seed_sample=(1-fraction[:,None])*desired[low,:3]+fraction[:,None]*desired[high,:3]
        for name,box in exclusions['cuboid'].items():
            center=np.asarray(box['pose'][:3]);quat=np.asarray(box['pose'][3:])
            rotation=Rotation.from_quat(quat[[1,2,3,0]])
            half=np.asarray(box['dims'])/2
            local=rotation.inv().apply(sampled-center);delta=np.abs(local)-half
            distance=np.linalg.norm(np.maximum(delta,0),axis=1)+np.minimum(delta.max(1),0)
            indices=np.flatnonzero(distance<=clearance)
            if not len(indices):continue
            basis=rotation.as_matrix()
            seed_local=rotation.inv().apply(seed_sample-center)
            for k in indices:
                key=(name,round(float(query[k]),9))
                if key in separated:continue
                # Select the free face used by cuRobo's seed at this instant.
                # A global lateral face can forbid legitimate entry/exit around
                # an obstacle's longitudinal end under tight acceleration.
                score=np.abs(seed_local[k])/(half+clearance)
                if exclusions.get('box_orientation')=='path_tangent':score[0]=0
                if face_mode=='seed' and score.max()>1:
                    face=int(np.argmax(score));sign=np.sign(seed_local[k,face])
                else:
                    projection=basis.T@np.asarray(direction)
                    if exclusions.get('box_orientation')=='path_tangent':projection[0]=0
                    face=int(np.argmax(np.abs(projection)));sign=np.sign(projection[face])
                normal=basis[:,face]*sign
                value=float(normal@center+half[face]+clearance+.00005)
                separated.add(key)
                columns=np.r_[low[k]+n*np.arange(3),high[k]+n*np.arange(3)]
                weights=np.r_[-normal*(1-fraction[k]),-normal*fraction[k]]
                row=sparse.csr_matrix((weights,(np.zeros(6,dtype=int),columns)),shape=(1,variable_count))
                planes.append(row);offsets.append(-value);changed=True
        if not changed:
            metrics=compare_path(path[:,:3],exclusions)
            if metrics['actual_wrist_exclusion_pass']:
                _,timing=fixed_budget_path(path,dt,ticks,dt,np.asarray(limits['velocity_limits']),
                                          np.asarray(limits['acceleration_limits']),dq0,dq1)
                timing['parameterization']='source_time_fit_with_virtual_separating_planes'
                timing['exclusion_bounds_enforced']=True
                timing['trajectory_cost']='normalized_acceleration_change_and_geometry_L1' if regularize else 'geometry_L1'
                timing['separation_face_mode']=face_mode
                return path,timing
            raise RuntimeError('Fixed-time interpolated path still crosses a virtual exclusion')
        result=linprog(objective,
                       A_ub=sparse.vstack((matrix,*planes),format='csr'),
                       b_ub=np.r_[rhs,offsets],bounds=bounds,method='highs',
                       options={'primal_feasibility_tolerance':1e-9,'dual_feasibility_tolerance':1e-9})
        if not result.success:raise RuntimeError('Detour separation cannot fit immutable source time and dynamics')
        path=result.x[:size].reshape(dof,n).T
    raise RuntimeError('Fixed-time exclusion fit did not converge')


def plan_detour(planner,start,goal_command,ticks,exclusions,goal_state,seed_index,
                baseline_commands=None,has_next_anchor=True,regularize=False,terminal_history_ticks=None):
    """cuRobo graph subplans -> one source-time fit -> both collision checks.

    Waypoints introduce no stops or extra time. Measured start velocity and
    the existing endpoint drive velocity are imposed by the final fitting.
    """
    from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
    scene=planner.scene;dt=scene.dt;limits=scene.cfg['planning']
    pose=np.asarray(start['wrist_xyz_wxyz']);goal=np.asarray(goal_command[:7])
    q0=pose_coordinates(pose);q1=pose_coordinates(goal,q0)
    dq0=pose_velocity(pose,start['wrist_twist'],q0)
    dq1=pose_velocity(goal,goal_command[19:25],q1)
    set_arc_reference(exclusions,start,goal_command,ticks,dt,limits)
    planning_exclusions={**exclusions,'cuboid':exclusions.get('planning_cuboid',exclusions['cuboid']),
        'sources':[{**s,'wrist_path_xyz':s.get('command_wrist_path_xyz',s['wrist_path_xyz'])}
                   for s in exclusions['sources']]}
    fixed=None;tail_ticks=0
    if baseline_commands is not None:
        baseline_commands=np.asarray(baseline_commands)
        baseline_poses=np.vstack((pose,baseline_commands[:,:7]))
        baseline_coordinates=np.asarray([pose_coordinates(p,q0) for p in baseline_poses])
        # Spatial diversity changes translation. Retain the already successful
        # wrist orientation/finger drives and their interaction handoff.
        fixed=np.full_like(baseline_coordinates,np.nan)
        tail_ticks=boundary_history_ticks(scene,planner.carry,ticks,has_next_anchor)
        if terminal_history_ticks is not None:
            tail_ticks=max(tail_ticks,min(int(terminal_history_ticks),ticks//2))
        if tail_ticks:fixed[-tail_ticks:]=baseline_coordinates[-tail_ticks:]
    if baseline_commands is not None:
        points,seed,seed_progress=dynamic_detour_seed(baseline_coordinates,dt,limits,dq0,dq1,
                                                    planning_exclusions,seed_index)
    else:
        seeds=list(detour_seeds(pose[:3],goal[:3],planning_exclusions))
        if seed_index>=len(seeds):raise RuntimeError('No more collision-free detour seeds')
        points,seed=seeds[seed_index]
    if baseline_commands is not None:
        # Translation and rotation must belong to the same measured-boundary
        # fit. An old drive orientation can lead the actual hand near a table,
        # falsely putting every detour's first waypoint below its surface.
        rotations=Rotation.from_euler('ZYX',np.vstack((q0[3:],
            np.asarray(seed['waypoint_coordinates'])[:,3:],q1[3:])))
    else:
        fractions=np.linspace(0,1,len(points)+2)
        rotations=Slerp([0,1],Rotation.from_quat([pose[[4,5,6,3]],goal[[4,5,6,3]]]))(fractions)
    targets=[];near=q0
    for xyz,rotation in zip(np.vstack((points,goal[:3])),rotations[1:]):
        p=np.r_[xyz,rotation.as_quat()[[3,0,1,2]]]
        near=pose_coordinates(p,near);targets.append(near.copy())
    targets[-1]=q1
    pieces=[];current=q0.copy()
    for target in targets:
        if not planner.valid(target):raise RuntimeError('Detour waypoint collides with physical geometry')
        result=planner.planner.plan_single_js(planner.js(current),planner.js(target),
                    MotionGenPlanConfig(enable_graph=True,enable_opt=False,max_attempts=1))
        if not bool(result.success.item()):raise RuntimeError(f'cuRobo detour subplan: {result.status}')
        piece=result.get_interpolated_plan().position.detach().cpu().numpy().reshape(-1,6).copy()
        piece[0]=current;piece[-1]=target
        pieces.append(piece if not pieces else piece[1:]);current=target
    raw=np.concatenate(pieces);rejected=[];desired=None
    if baseline_commands is not None:
        # cuRobo supplies geometry; the successful robot drive supplies source
        # time progress. Graph sample counts are not a time parameterization.
        progress=seed_progress
        geometry=resample(raw[:,:3],max(101,ticks+1))
        desired=baseline_coordinates.copy()
        desired[:,:3]=np.stack([np.interp(progress,np.linspace(0,1,len(geometry)),geometry[:,j]) for j in range(3)],axis=1)
    np.savez_compressed(scene.out/'detour_geometry.npz',raw=raw,
        desired=desired if desired is not None else np.empty((0,6)),
        fixed=fixed if fixed is not None else np.empty((0,6)),dq0=dq0,dq1=dq1)
    variants=[('original',exclusions,planning_exclusions)]
    prior_paths=[dict(id=s['run'],points=np.asarray(s.get('source_time_wrist_path_xyz',s['wrist_path_xyz'])),
        command_points=np.asarray(s.get('source_time_command_path_xyz',s.get('command_wrist_path_xyz',s['wrist_path_xyz'])))) for s in exclusions['sources']]
    try:
        moving=interval_exclusions(prior_paths,exclusions['window'],ticks*dt,limits,
            exclusions['size_multiplier'],exclusions.get('free_diversity_budget_s',ticks*dt),
            np.array([start['wrist_twist'][:3],goal_command[19:22]]))
        moving.update(arc_reference_length_m=exclusions['arc_reference_length_m'],
                      arc_reference_method=exclusions['arc_reference_method'])
        # Boundary neighborhoods are shared. The interior must still exclude
        # every previous curve; an empty/vacuous exclusion is not a solution.
        if all(not compare_path(p['points'],moving)['actual_wrist_exclusion_pass'] for p in prior_paths):
            moving_plan={**moving,'cuboid':moving['planning_cuboid'],
                'sources':[{**s,'wrist_path_xyz':s['command_wrist_path_xyz']} for s in moving['sources']]}
            variants.append(('moving_boundary',moving,moving_plan))
    except ValueError:pass
    fits=[]
    for variant,actual_exclusion,plan_exclusion in variants:
        for mode in ['seed','direction']:
            fits.append((variant+'_'+mode,actual_exclusion,plan_exclusion,
                lambda e=plan_exclusion,m=mode:exclusion_fitted_path(raw,ticks,dt,limits,dq0,dq1,
                    e,seed['direction'],fixed,desired,m,regularize)))
    fits.append(('fixed_budget',exclusions,planning_exclusions,
        lambda:fixed_budget_path(raw,dt,ticks,dt,np.asarray(limits['velocity_limits']),
                                np.asarray(limits['acceleration_limits']),dq0,dq1)))
    for name,actual_exclusion,plan_exclusion,fit in fits:
        try:
            path,timing=fit();metrics=compare_path(path[:,:3],plan_exclusion)
            if fixed is not None and np.isfinite(fixed).any() and np.max(np.abs((path-fixed)[np.isfinite(fixed)]))>1e-8:
                raise RuntimeError('Time fitting changed fixed controller boundary history')
            if not metrics['actual_wrist_exclusion_pass']:
                raise RuntimeError('Source-time fit crosses an excluded wrist path')
            if not metrics['smooth_arc_pass']:
                raise RuntimeError('Source-time detour has excessive loops or backward progress')
            for k,q in enumerate(path):
                if not planner.valid(q):raise RuntimeError(f'Fitted detour physical collision at tick {k}')
            if actual_exclusion is not exclusions:
                exclusions.clear();exclusions.update(actual_exclusion)
                exclusions['moving_boundary_protection_applied']=True
            break
        except RuntimeError as error:rejected.append(f'{name}: {error}')
    else:raise RuntimeError(f'No excluded-path-free detour in original source time: {rejected}')
    poses=np.asarray([coordinates_pose(q) for q in path])
    hands=relaxed_hand_path(scene,start,goal_state,ticks)
    if baseline_commands is not None:hands=np.vstack((start['command_hand_q'],baseline_commands[:,7:19]))
    commands=controls_from_poses(poses,hands,dt)
    timing.update(planner='cuRobo_virtual_6DOF',geometry_solver='curobo_graph_waypoints_fixed_source_time',
        detour_seed=seed,seed_index=seed_index,fit=name,previous_fit_rejections=rejected,
        common_terminal_history_ticks=tail_ticks,finger_commands_preserved=baseline_commands is not None,
        orientation_conditioned_on_passed_bridge=baseline_commands is not None,
        waypoint_orientation_from_measured_boundary_fit=baseline_commands is not None,
        delta_dynamics_feedforward=bool(baseline_commands is not None and not planner.carry),
        planned_diversity=metrics,virtual_exclusion_scope='wrist',
        hand_collision_spheres=planner.sphere_count,carried_object_collision=planner.carry,
        reference_tracking=False,source_time_stretch=False,arm_model_used=False,
        curobo_optimize_dt=False,hand_boundary_motion=bool(np.any(abs(hands-hands[0])>1e-8)),
        planned_hand_geometry='entry_posture',hand_boundary_collision_validation='measured_geometry_each_physics_step')
    return commands,timing
