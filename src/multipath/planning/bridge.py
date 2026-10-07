"""Cheap necessary bridge checks before cuRobo initialization."""
import numpy as np
from scipy.spatial.transform import Rotation
from multipath.planning.floating import pose_coordinates,carried_proxy


def carried_object_clearance(scene,state):
    """Check table support clearance and actual carried/target mesh separation.

    A hand/tool contact while the table supports the bowl is not a transportable
    bridge boundary. This cheap exit check avoids accepting that local optimum.
    """
    spheres=carried_proxy(scene);tool=np.asarray(state['objects']['tool']['pose'])
    rotation=Rotation.from_quat(tool[[4,5,6,3]])
    centers=rotation.apply(np.array([x['center'] for x in spheres]))+tool[:3]
    radii=np.array([x['radius'] for x in spheres]);boxes={'table':scene.table}
    result={}
    for role,box in boxes.items():
        pose=np.asarray(box['pose']);local=Rotation.from_quat(pose[[4,5,6,3]]).inv().apply(centers-pose[:3])
        offset=np.abs(local)-np.asarray(box['dims'])/2
        distance=np.linalg.norm(np.maximum(offset,0),axis=1)+np.minimum(np.max(offset,axis=1),0)-radii
        result[role]=float(np.min(distance))
    # A target's bounding box can fill a tray/bowl cavity or empty corners.
    # That rejected a carried exit with 4.2 cm actual mesh separation.
    import fcl
    if not hasattr(scene,'object_clearance_meshes'):
        scene.object_clearance_meshes={}
        for role in ('tool','target'):
            mesh=scene.meshes[role];model=fcl.BVHModel();model.beginModel(len(mesh.vertices),len(mesh.faces))
            model.addSubModel(mesh.vertices,mesh.faces);model.endModel();scene.object_clearance_meshes[role]=model
    bodies=[]
    for role in ('tool','target'):
        pose=np.asarray(state['objects'][role]['pose'])
        bodies.append(fcl.CollisionObject(scene.object_clearance_meshes[role],
            fcl.Transform(Rotation.from_quat(pose[[4,5,6,3]]).as_matrix(),pose[:3])))
    result['target']=float(fcl.distance(*bodies,fcl.DistanceRequest(),fcl.DistanceResult()))
    if fcl.collide(*bodies,fcl.CollisionRequest(),fcl.CollisionResult()):result['target']=-1e-6
    return result


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


def discrete_displacement_bounds(v0,v1,budget,dt,velocity_limits,acceleration_limits):
    """Envelope for the executor's exact first/last discrete command slope."""
    ticks=round(budget/dt);k=np.arange(ticks)[:,None]
    v0=np.asarray(v0);v1=np.asarray(v1);vmax=np.asarray(velocity_limits);amax=np.asarray(acceleration_limits)
    high=np.minimum(vmax,np.minimum(v0+amax*dt*k,v1+amax*dt*(ticks-1-k)))
    low=np.maximum(-vmax,np.maximum(v0-amax*dt*k,v1-amax*dt*(ticks-1-k)))
    return low.sum(0)*dt,high.sum(0)*dt


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


def project_boundary_target(start,target,budget,planning,dt=None):
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
        low,high=(displacement_bounds(v0,v1,budget,vmax,amax) if dt is None else
                  discrete_displacement_bounds(v0,v1,budget,dt,vmax,amax))
        if np.any(low>high):return goal
        q=q0+np.clip(desired-q0,low,high)
        goal['wrist_xyz_wxyz']=coordinates_pose(q).tolist()
    goal['state_origin']='unvalidated_source_conditioned_boundary_proposal'
    goal['physically_acquired_state']=False
    return goal


def terminal_stop_target(start,budget,dt,planning):
    """No next interaction: brake freely inside the remaining source budget."""
    from multipath.planning.floating import coordinates_pose
    q0=pose_coordinates(start['wrist_xyz_wxyz'])
    v0=coordinate_velocity(start['wrist_xyz_wxyz'],start['wrist_twist'],q0)
    # Brake on each coordinate's acceleration scale. Coasting for half the
    # entire free interval can unnecessarily drive a released hand into a wall.
    stop=np.minimum(budget,np.maximum(2*dt,np.abs(v0)/(np.asarray(planning['acceleration_limits'])*.95)))
    goal=dict(start,wrist_xyz_wxyz=coordinates_pose(q0+.5*stop*v0).tolist(),
        wrist_twist=np.zeros(6).tolist(),physically_acquired_state=False,
        state_origin='unvalidated_free_terminal_stop')
    return project_boundary_target(start,goal,budget,planning,dt=dt)


def free_exit_clearance_offset(scene,actual,geometry):
    """Resynthesize a released Anchor boundary from real collision feedback.

    The offset is a command proposal. Its size comes from the recorded hand
    cover and one physics step of measured surface motion, not a task name.
    """
    from multipath.planning.floating import sphere_world_clearance
    if not actual.get('physically_acquired_state') or actual['contact']['mode']!='free':
        raise ValueError('Free-exit feedback requires a real released state')
    spheres=geometry['spheres'];pose=np.asarray(actual['wrist_xyz_wxyz']);target=pose.copy()
    boxes={'table':scene.table}
    for role in ('tool','target'):
        state=actual['objects'][role];p=np.asarray(state['pose']);bounds=scene.meshes[role].bounds
        rotation=Rotation.from_quat(p[[4,5,6,3]])
        boxes[role]={'dims':(bounds[1]-bounds[0]).tolist(),'pose':np.r_[p[:3]+rotation.apply(bounds.mean(0)),p[3:]].tolist()}
    radius=max(np.linalg.norm(s['center'])+s['radius'] for s in spheres)
    velocity=np.asarray(actual['wrist_twist'])
    margin=max(1e-4,scene.dt*(np.linalg.norm(velocity[:3])+radius*np.linalg.norm(velocity[3:])))
    initial=sphere_world_clearance(target,spheres,boxes)[0]
    for _ in range(20):
        _,worst=sphere_world_clearance(target,spheres,boxes)
        if worst[0]>=margin-1e-8:break
        target[:3]+=(margin-worst[0])*worst[1]
    else:raise RuntimeError('No free Anchor exit-clearance proposal')
    offset=target[:3]-pose[:3]
    return offset,dict(offset_m=offset.tolist(),surface_motion_margin_m=float(margin),initial_clearance_m=initial,
        physically_acquired_feedback=True,physics_acceptance=False,source_time_stretch=False)


def feedback_boundary_target(start,goal,planned_pose,actual_state,source_target,budget,dt,planning):
    """Develop one canonical boundary using a preceding physical rollout.

    Only the next oracle boundary conditions this fit. No relaxed reference
    trajectory is tracked and the fitted target is never a physical artifact.
    A new cuRobo plan and real execution must validate the proposal.
    """
    from scipy.optimize import minimize
    from multipath.validation.physics import OBJECT_POSITION_TOLERANCE_M,OBJECT_ROTATION_TOLERANCE_RAD
    from multipath.execution.mounted_scene import pose_matrix
    from multipath.planning.floating import coordinates_pose
    if not actual_state.get('physically_acquired_state'):
        raise ValueError('Boundary feedback requires real physical execution')
    held=np.asarray(start['hand_object_transform']);source_target=np.asarray(source_target)
    planned_tool=pose_matrix(planned_pose)@held;actual_tool=pose_matrix(actual_state['objects']['tool']['pose'])
    offset=actual_tool[:3,3]-planned_tool[:3,3]
    correction=actual_tool@np.linalg.inv(planned_tool)
    tracking=pose_matrix(actual_state['wrist_xyz_wxyz'])@np.linalg.inv(pose_matrix(planned_pose))
    measured_held=np.asarray(actual_state['hand_object_transform'])
    q0=pose_coordinates(start['wrist_xyz_wxyz']);seed=pose_coordinates(goal['wrist_xyz_wxyz'],q0)
    v0=coordinate_velocity(start['wrist_xyz_wxyz'],start['wrist_twist'],q0)
    vmax=np.asarray(planning['velocity_limits']);amax=np.asarray(planning['acceleration_limits'])
    v1_seed=coordinate_velocity(goal['wrist_xyz_wxyz'],goal['wrist_twist'],seed)
    def tool(x):return pose_matrix(coordinates_pose(x))@held
    def predicted_tool(x):
        # Grasp deflection belongs to the hand frame. A single left/world
        # transform of the tool also rotates that deflection when the new goal
        # changes orientation, causing successive feedback fits to oscillate.
        return tracking@pose_matrix(coordinates_pose(x))@measured_held
    def constraints(x):
        q,v1=x[:6],x[6:]
        low,high=discrete_displacement_bounds(v0,v1,budget,dt,vmax,amax);m=tool(q)
        return np.r_[q-q0-low,q0+high-q,
            (.98*OBJECT_POSITION_TOLERANCE_M)**2-np.sum((m[:3,3]-source_target[:3,3])**2),
            (.98*OBJECT_ROTATION_TOLERANCE_RAD)**2-Rotation.from_matrix(source_target[:3,:3].T@m[:3,:3]).magnitude()**2]
    def objective(x):
        predicted=predicted_tool(x[:6])
        position=np.sum((predicted[:3,3]-source_target[:3,3])**2)/OBJECT_POSITION_TOLERANCE_M**2
        rotation=Rotation.from_matrix(source_target[:3,:3].T@predicted[:3,:3]).magnitude()**2/OBJECT_ROTATION_TOLERANCE_RAD**2
        return position+rotation+.01*np.sum((x[:6]-seed)**2)+.01*np.sum((x[6:]/vmax)**2)
    # The oracle fixes interaction pose/contact/time, not the human velocity.
    # Keeping its velocity fixed exhausted the reachable rotation envelope and
    # repeated the same failed boundary. Choose a canonical velocity together
    # with the pose; the subsequent physical Anchor must accept this entry.
    result=minimize(objective,np.r_[seed,np.clip(v1_seed,-vmax,vmax)],method='SLSQP',
                    bounds=[(None,None)]*6+list(zip(-vmax,vmax)),
                    constraints=[dict(type='ineq',fun=constraints)],options={'maxiter':120,'ftol':1e-10})
    if not result.success or min(constraints(result.x)) < -1e-8:
        raise RuntimeError('No feedback boundary proposal within original source/time conditions')
    q,v1=result.x[:6],result.x[6:];pose=coordinates_pose(q);eps=1e-5
    advanced=coordinates_pose(q+eps*v1)
    twist=np.r_[v1[:3],(Rotation.from_quat(advanced[[4,5,6,3]])*
        Rotation.from_quat(pose[[4,5,6,3]]).inv()).as_rotvec()/eps]
    proposal=dict(goal,wrist_xyz_wxyz=pose.tolist(),wrist_twist=twist.tolist(),
                  physically_acquired_state=False,state_origin='unvalidated_physical_feedback_boundary_proposal')
    predicted=predicted_tool(q)
    return proposal,dict(measured_tool_position_offset_m=offset.tolist(),
        measured_tool_rotation_offset_rad=float(Rotation.from_matrix(correction[:3,:3]).magnitude()),
        predicted_tool_position_error_m=float(np.linalg.norm(predicted[:3,3]-source_target[:3,3])),
        predicted_tool_rotation_error_rad=float(Rotation.from_matrix(source_target[:3,:3].T@predicted[:3,:3]).magnitude()),
        canonical_boundary_velocity_selected=True,nominal_planning_limits_unchanged=True,
        feedback_model='world_wrist_tracking_and_measured_hand_object_transform',
        source_condition_unchanged=True,physics_acceptance=False)


def quiet_boundary_target(start,goal,source_target,budget,dt,planning):
    """Choose one source-conditioned canonical proposal with a quiet exit.

    Human boundary velocity is not a robot canonical velocity. Pose/contact
    conditions remain unchanged; stopping must fit the original time budget.
    This proposal only becomes an entry after continuous physical acquisition
    and a fresh local core rollout.
    """
    from scipy.optimize import minimize
    from multipath.execution.mounted_scene import pose_matrix
    from multipath.planning.floating import coordinates_pose
    from multipath.validation.physics import OBJECT_POSITION_TOLERANCE_M,OBJECT_ROTATION_TOLERANCE_RAD
    q0=pose_coordinates(start['wrist_xyz_wxyz']);held=np.asarray(start['hand_object_transform'])
    v0=coordinate_velocity(start['wrist_xyz_wxyz'],start['wrist_twist'],q0)
    vmax=np.asarray(planning['velocity_limits'])*.98;amax=np.asarray(planning['acceleration_limits'])*.95
    low,high=discrete_displacement_bounds(v0,np.zeros(6),budget,dt,vmax,amax)
    if np.any(low>high):raise RuntimeError('No quiet boundary within source time')
    source_target=np.asarray(source_target)
    def tool(x):return pose_matrix(coordinates_pose(x))@held
    def constraints(x):
        m=tool(x)
        return np.r_[x-q0-low,q0+high-x,
            (.98*OBJECT_POSITION_TOLERANCE_M)**2-np.sum((m[:3,3]-source_target[:3,3])**2),
            (.98*OBJECT_ROTATION_TOLERANCE_RAD)**2-Rotation.from_matrix(source_target[:3,:3].T@m[:3,:3]).magnitude()**2]
    def objective(x):
        m=tool(x)
        # Prefer source position and small wrist rotations while preserving
        # a necessary acceleration envelope; cuRobo checks actual geometry.
        return 10*np.sum((m[:3,3]-source_target[:3,3])**2)/OBJECT_POSITION_TOLERANCE_M**2+np.sum((x[3:]-q0[3:])**2)/.35**2
    seed=q0+np.clip(pose_coordinates(goal['wrist_xyz_wxyz'],q0)-q0,low,high)
    fit=minimize(objective,seed,method='SLSQP',constraints=[dict(type='ineq',fun=constraints)],options={'maxiter':100,'ftol':1e-10})
    if not fit.success or min(constraints(fit.x)) < -1e-8:raise RuntimeError('No quiet source-conditioned boundary proposal')
    return dict(goal,wrist_xyz_wxyz=coordinates_pose(fit.x).tolist(),wrist_twist=np.zeros(6).tolist(),
        state_origin='unvalidated_quiet_source_conditioned_boundary',physically_acquired_state=False)
