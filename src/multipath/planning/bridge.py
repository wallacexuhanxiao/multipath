"""Cheap necessary bridge checks before cuRobo initialization."""
import numpy as np
from scipy.spatial.transform import Rotation
from multipath.planning.floating import pose_coordinates,carried_proxy


def hand_table_clearance(scene):
    """Exact reached hand/table boundary, shared by local synthesis and planning.

    A successful release is not enough if the next planner starts with a hand
    mesh inside the table. Use the same measured surfaces as FloatingPlanner;
    do not substitute a coarse sphere overlap for a physical collision.
    """
    from pxr import Usd,UsdGeom
    from multipath.planning.floating import mesh_box_intersects,usd_mesh_faces
    if not hasattr(scene,'anchor_hand_meshes'):
        scene.anchor_hand_meshes=[]
        for prim in Usd.PrimRange.Stage(scene.world.stage,Usd.TraverseInstanceProxies()):
            if not str(prim.GetPath()).startswith(scene.root+'/') or not prim.IsA(UsdGeom.Mesh):continue
            if 'collision' in str(prim.GetPath()).lower():continue
            mesh=UsdGeom.Mesh(prim)
            scene.anchor_hand_meshes.append((prim,np.asarray(mesh.GetPointsAttr().Get()),usd_mesh_faces(mesh)))
    cache=UsdGeom.XformCache();box=scene.table;b=np.asarray(box['pose']);half=np.asarray(box['dims'])/2
    rotation=Rotation.from_quat(b[[4,5,6,3]]);minimum=np.inf;collisions=[]
    for prim,points,faces in scene.anchor_hand_meshes:
        matrix=np.asarray(cache.GetLocalToWorldTransform(prim)).T
        vertices=points@matrix[:3,:3].T+matrix[:3,3]
        offset=np.abs(rotation.inv().apply(vertices-b[:3]))-half
        distance=np.linalg.norm(np.maximum(offset,0),axis=1)+np.minimum(offset.max(1),0)
        minimum=min(minimum,float(distance.min()))
        if mesh_box_intersects(vertices,faces,box):collisions.append(str(prim.GetPath()))
    if not np.isfinite(minimum):raise RuntimeError('Anchor boundary has no measured hand geometry')
    return dict(clearance_m=float(min(minimum,-1e-7) if collisions else minimum),
                collision_free=not collisions,intersecting_meshes=collisions,
                geometry='measured triangle surfaces against unchanged table')


def carried_object_clearance(scene,state):
    """Check table support clearance and actual carried/target mesh separation.

    A hand/tool contact while the table supports the bowl is not a transportable
    bridge boundary. This cheap exit check avoids accepting that local optimum.
    """
    tool=np.asarray(state['objects']['tool']['pose'])
    rotation=Rotation.from_quat(tool[[4,5,6,3]])
    # Coarse covering spheres extend beyond the bowl's real surface. Their
    # table overlap is a planning diagnostic, not physical penetration.
    vertices=rotation.apply(scene.meshes['tool'].vertices)+tool[:3];boxes={'table':scene.table}
    result={}
    for role,box in boxes.items():
        pose=np.asarray(box['pose']);local=Rotation.from_quat(pose[[4,5,6,3]]).inv().apply(vertices-pose[:3])
        offset=np.abs(local)-np.asarray(box['dims'])/2
        distance=np.linalg.norm(np.maximum(offset,0),axis=1)+np.minimum(np.max(offset,axis=1),0)
        from multipath.planning.floating import mesh_box_intersects
        collision=mesh_box_intersects(vertices,scene.meshes['tool'].faces,box)
        result[role]=float(min(np.min(distance),-1e-7) if collision else np.min(distance))
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
    """Measured world twist in the planner's ZYX coordinate chart.

    Use the same analytic Jacobian for boundary fitting and execution. Unequal
    finite-difference steps rejected feasible acceleration-envelope boundaries.
    """
    q=pose_coordinates(np.asarray(pose,dtype=float),near);twist=np.asarray(twist,dtype=float)
    jac=coordinate_angular_jacobian(q)
    angular=np.linalg.solve(jac,twist[3:]) if abs(np.cos(q[4]))>1e-8 else np.linalg.lstsq(jac,twist[3:],rcond=1e-8)[0]
    return np.r_[twist[:3],angular]


def coordinate_angular_jacobian(q):
    yaw,pitch=np.asarray(q)[3:5]
    return np.array([[0.,-np.sin(yaw),np.cos(yaw)*np.cos(pitch)],
                     [0.,np.cos(yaw),np.sin(yaw)*np.cos(pitch)],
                     [1.,0.,-np.sin(pitch)]])


def coordinates_twist(q,velocity):
    velocity=np.asarray(velocity,dtype=float)
    return np.r_[velocity[:3],coordinate_angular_jacobian(q)@velocity[3:]]


def bridge_precheck(start,goal,budget,planning):
    """Necessary conditions only; passing still requires planning and physics.

    Independent Anchor success is not a proof of fixed-time compatibility.
    These limits are the configured planning budgets, not hardware ratings.
    Tracking compensation can separate the commanded terminal state from
    the predicted physical handoff. Test the command's nominal feasibility;
    the actual handoff is still verified after continuous execution.
    """
    compensated=bool(goal.get('boundary_prediction_from_physical_tracking'))
    if compensated:
        goal=dict(goal,wrist_xyz_wxyz=goal['command_wrist_xyz_wxyz'],
            wrist_twist=goal['command_wrist_twist'])
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
                boundary_checked='compensating_command' if compensated else 'trajectory_boundary',
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


def free_exit_clearance_offset(scene,actual,geometry,obstacle_roles=None):
    """Resynthesize a released Anchor boundary from real collision feedback.

    The offset is a command proposal. Its size comes from the recorded hand
    cover and one physics step of measured surface motion, not a task name.
    """
    from multipath.planning.floating import sphere_world_clearance
    if not actual.get('physically_acquired_state') or actual['contact']['mode']!='free':
        raise ValueError('Free-exit feedback requires a real released state')
    spheres=geometry['spheres'];pose=np.asarray(actual['wrist_xyz_wxyz']);target=pose.copy()
    roles=set(obstacle_roles) if obstacle_roles is not None else {'table','tool','target'}
    if not roles or not roles<={'table','tool','target'}:raise ValueError('Unknown free-exit collision feedback role')
    boxes={'table':scene.table} if 'table' in roles else {}
    for role in ('tool','target'):
        if role not in roles:continue
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
        obstacle_roles=sorted(roles),physically_acquired_feedback=True,physics_acceptance=False,source_time_stretch=False)


def feedback_boundary_target(start,goal,planned_pose,actual_state,source_target,budget,dt,planning,*,clearance=None,clearance_gradient=None,final_coordinate_velocity=None,interior_scales=(.98,.95,.98),max_collision_seeds=7):
    """Develop one canonical boundary using a preceding physical rollout.

    Only the next oracle boundary conditions this fit. No relaxed reference
    trajectory is tracked and the fitted target is never a physical artifact.
    A new cuRobo plan and real execution must validate the proposal.
    """
    from scipy.optimize import minimize
    from time import monotonic
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
    # Leave room for spatial-twist/Euler conversion and the subsequent
    # discrete fit. An optimum exactly on the envelope failed that fit.
    scales=np.asarray(interior_scales,dtype=float)
    if scales.shape!=(3,) or not np.isfinite(scales).all() or np.any(scales<=0) or np.any(scales>1):
        raise ValueError('Fit interiors must stay within declared velocity, acceleration and source limits')
    vmax=np.asarray(planning['velocity_limits'])*scales[0];amax=np.asarray(planning['acceleration_limits'])*scales[1]
    v1_seed=coordinate_velocity(goal['wrist_xyz_wxyz'],
        goal.get('interaction_entry_twist_prior',goal['wrist_twist']),seed)
    velocity_bounds=list(zip(-vmax,vmax))
    if final_coordinate_velocity is not None:
        v1_seed=np.asarray(final_coordinate_velocity,dtype=float)
        if v1_seed.shape!=(6,) or not np.isfinite(v1_seed).all() or np.any(np.abs(v1_seed)>vmax):
            raise ValueError('Canonical terminal velocity exceeds the unchanged planning budget')
        velocity_bounds=list(zip(v1_seed,v1_seed))
    def tool(x):return pose_matrix(coordinates_pose(x))@held
    def predicted_tool(x):
        # Grasp deflection belongs to the hand frame. A single left/world
        # transform of the tool also rotates that deflection when the new goal
        # changes orientation, causing successive feedback fits to oscillate.
        return tracking@pose_matrix(coordinates_pose(x))@measured_held
    displacement_scale=np.maximum(vmax*budget,1e-6)
    def nominal_constraints(x):
        q,v1=x[:6],x[6:]
        low,high=discrete_displacement_bounds(v0,v1,budget,dt,vmax,amax);m=tool(q)
        position=np.sum((m[:3,3]-source_target[:3,3])**2)/OBJECT_POSITION_TOLERANCE_M**2
        rotation=Rotation.from_matrix(source_target[:3,:3].T@m[:3,:3]).magnitude()**2/OBJECT_ROTATION_TOLERANCE_RAD**2
        return np.r_[(q-q0-low)/displacement_scale,(q0+high-q)/displacement_scale,
                     scales[2]**2-position-rotation]
    geometry_cache={}
    def geometry(q):
        # Geometry depends on pose, not terminal velocity. Cache the paired
        # distance/gradient so SLSQP reuses the same float32 collision query.
        key=tuple(q)
        if key not in geometry_cache:
            if clearance_gradient is None:
                geometry_cache[key]=(float(clearance(coordinates_pose(q))),None)
            else:
                value,gradient=clearance_gradient(np.asarray(q)[None])
                geometry_cache[key]=(float(value[0]),np.asarray(gradient[0],dtype=float))
        return geometry_cache[key]
    def constraints(x):
        values=nominal_constraints(x)
        if clearance is not None:values=np.r_[values,(geometry(x[:6])[0]-1e-4)/OBJECT_POSITION_TOLERANCE_M]
        return values
    def constraint_jacobian(x):
        # Source pose and discrete time envelopes are float64. Only the GPU
        # collision row needs its existing autograd/ZYX kinematics gradient;
        # finite differences can lose precision in rounded GPU distances.
        epsilon=1e-6;jacobian=np.empty((13,len(x)))
        for i in range(len(x)):
            step=np.zeros_like(x);step[i]=epsilon
            jacobian[:,i]=(nominal_constraints(x+step)-nominal_constraints(x-step))/(2*epsilon)
        return np.vstack((jacobian,np.r_[geometry(x[:6])[1],np.zeros(6)]/OBJECT_POSITION_TOLERANCE_M))
    def objective(x):
        predicted=predicted_tool(x[:6])
        position=np.sum((predicted[:3,3]-source_target[:3,3])**2)/OBJECT_POSITION_TOLERANCE_M**2
        rotation=Rotation.from_matrix(source_target[:3,:3].T@predicted[:3,:3]).magnitude()**2/OBJECT_ROTATION_TOLERANCE_RAD**2
        # A fast boundary may satisfy the pose but make the next interaction
        # impossible to take over. Its local motion is a velocity preference,
        # not a mandatory human velocity or relaxed trajectory tracking gate.
        return position+rotation+.01*np.sum((x[:6]-seed)**2)+np.sum(((x[6:]-v1_seed)/vmax)**2)
    # The oracle fixes interaction pose/contact/time, not the human velocity.
    # Keeping its velocity fixed exhausted the reachable rotation envelope and
    # repeated the same failed boundary. Choose a canonical velocity together
    # with the pose; the subsequent physical Anchor must accept this entry.
    options={'maxiter':120,'ftol':1e-10}
    # Collision queries use float32 GPU geometry. The default 1e-8 numerical
    # difference can disappear; use a geometric step for this coupled fit.
    if clearance is not None:options['eps']=1e-4
    seeds=[seed]
    if clearance is not None:
        # Peer-hand obstacles make this nonconvex. Sample a few orientations
        # inside the existing source tolerance, around the carried object.
        # Select one canonical boundary; these are not trajectory solutions.
        for axis in np.eye(3):
            for sign in (-1,1):
                candidate=source_target.copy()
                candidate[:3,:3]=source_target[:3,:3]@Rotation.from_rotvec(
                    axis*sign*.8*OBJECT_ROTATION_TOLERANCE_RAD).as_matrix()
                wrist=candidate@np.linalg.inv(held)
                pose=np.r_[wrist[:3,3],Rotation.from_matrix(wrist[:3,:3]).as_quat()[[3,0,1,2]]]
                seeds.append(pose_coordinates(pose,q0))
    attempts=[];feasible=False;fit_begin=monotonic()
    condition=dict(type='ineq',fun=constraints)
    if clearance is not None and clearance_gradient is not None:
        condition['jac']=constraint_jacobian
    for candidate in seeds[:max_collision_seeds]:
        result=minimize(objective,np.r_[candidate,np.clip(v1_seed,-vmax,vmax)],method='SLSQP',
                        bounds=[(None,None)]*6+velocity_bounds,
                        constraints=[condition],options=options)
        minimum=float(min(constraints(result.x)))
        feasible=minimum>=-1e-8 and (result.success or clearance is not None)
        q,v=result.x[:6],result.x[6:]
        m=tool(q);low,high=discrete_displacement_bounds(v0,v,budget,dt,vmax,amax)
        details=dict(wrist_xyz_wxyz=coordinates_pose(q).tolist(),coordinate_velocity=v.tolist(),
            source_position_error_m=float(np.linalg.norm(m[:3,3]-source_target[:3,3])),
            source_rotation_error_rad=float(Rotation.from_matrix(source_target[:3,:3].T@m[:3,:3]).magnitude()),
            displacement_below_lower=np.maximum(0,low-(q-q0)).tolist(),
            displacement_above_upper=np.maximum(0,q-q0-high).tolist(),
            source_condition_margin=float(nominal_constraints(result.x)[-1]),
            collision_gradient=None if clearance_gradient is None else geometry(q)[1].tolist())
        attempts.append(dict(solver_success=bool(result.success),solver_message=str(result.message),
            minimum_constraint=minimum,feasible=bool(feasible),
            clearance_m=None if clearance is None else geometry(result.x[:6])[0],candidate=details))
        if clearance is not None:
            print('BOUNDARY_FIT_ATTEMPT',len(attempts),attempts[-1],
                  'elapsed_s',round(monotonic()-fit_begin,3),flush=True)
        if feasible:break
    if not feasible:
        best=max(range(len(attempts)),key=lambda i:attempts[i]['minimum_constraint'])
        error=RuntimeError('No canonical boundary found within source/time/collision conditions')
        error.boundary_fit_report=dict(attempts=attempts,source_time_stretch=False,physics_acceptance=False,
            collision_gradient_used=clearance_gradient is not None,collision_queries=len(geometry_cache),
            best_candidate=attempts[best]['candidate'],source_target=source_target.tolist(),
            source_budget_s=float(budget),entry_pose=start['wrist_xyz_wxyz'],entry_twist=start['wrist_twist'],
            constraint_scaling='positive normalization only; original feasible set retained')
        error.boundary_fit_report['elapsed_s']=monotonic()-fit_begin
        raise error
    q,v1=result.x[:6],result.x[6:];pose=coordinates_pose(q)
    twist=coordinates_twist(q,v1)
    proposal=dict(goal,wrist_xyz_wxyz=pose.tolist(),wrist_twist=twist.tolist(),
                  physically_acquired_state=False,state_origin='unvalidated_physical_feedback_boundary_proposal')
    predicted=predicted_tool(q)
    return proposal,dict(measured_tool_position_offset_m=offset.tolist(),
        measured_tool_rotation_offset_rad=float(Rotation.from_matrix(correction[:3,:3]).magnitude()),
        predicted_tool_position_error_m=float(np.linalg.norm(predicted[:3,3]-source_target[:3,3])),
        predicted_tool_rotation_error_rad=float(Rotation.from_matrix(source_target[:3,:3].T@predicted[:3,:3]).magnitude()),
        canonical_boundary_velocity_selected=True,nominal_planning_limits_unchanged=True,
        fixed_terminal_coordinate_velocity=final_coordinate_velocity is not None,
        interior_scales=scales.tolist(),
        feedback_model='world_wrist_tracking_and_measured_hand_object_transform',
        source_condition_unchanged=True,physics_acceptance=False,
        collision_boundary_fitted=clearance is not None,boundary_fit_attempts=attempts,
        collision_gradient_used=clearance_gradient is not None,collision_queries=len(geometry_cache),
        boundary_fit_elapsed_s=monotonic()-fit_begin)


def lookahead_grasp_boundary(start,previous_exit,source_exit,source_entry,tail_budget,bridge_budget,dt,planning,*,exit_condition=None):
    """Choose an Anchor exit together with its next oracle entry.

    These are command proposals from an acquired grasp, not physical states.
    Both intervals retain their original source budgets and object conditions.
    Unlike a quiet exit, the intermediate velocity is selected for both sides.
    Collision and closed-loop acceptance still require fresh real execution.
    """
    from scipy.optimize import minimize
    from multipath.planning.floating import coordinates_pose
    from multipath.execution.mounted_scene import pose_matrix
    from multipath.validation.physics import OBJECT_POSITION_TOLERANCE_M,OBJECT_ROTATION_TOLERANCE_RAD
    if not start.get('physically_acquired_state'):
        raise ValueError('Lookahead exit requires a physically acquired grasp')
    held=np.asarray(start['hand_object_transform'])
    q0=pose_coordinates(start['wrist_xyz_wxyz'])
    v0=coordinate_velocity(start['wrist_xyz_wxyz'],start['wrist_twist'],q0)
    vmax=np.asarray(planning['velocity_limits']);amax=np.asarray(planning['acceleration_limits'])
    sources=[np.asarray(source_exit),np.asarray(source_entry)];desired=[]
    for source in sources:
        wrist=source@np.linalg.inv(held)
        desired.append(pose_coordinates(np.r_[wrist[:3,3],Rotation.from_matrix(wrist[:3,:3]).as_quat()[[3,0,1,2]]],q0))
    def source_score(q,source):
        tool=pose_matrix(coordinates_pose(q))@held
        return (np.sum((tool[:3,3]-source[:3,3])**2)/OBJECT_POSITION_TOLERANCE_M**2+
            Rotation.from_matrix(source[:3,:3].T@tool[:3,:3]).magnitude()**2/OBJECT_ROTATION_TOLERANCE_RAD**2)
    def conditions(x):
        qout,vout,qin,vin=x.reshape(4,6)
        result=[]
        for a,b,va,vb,budget in ((q0,qout,v0,vout,tail_budget),(qout,qin,vout,vin,bridge_budget)):
            low,high=discrete_displacement_bounds(va,vb,budget,dt,vmax,amax)
            scale=np.maximum(vmax*budget,1e-6)
            result.extend(((b-a-low)/scale,(a+high-b)/scale))
        return np.r_[*result,1-source_score(qout,sources[0]),1-source_score(qin,sources[1]),
            [] if exit_condition is None else [exit_condition(qout)/OBJECT_POSITION_TOLERANCE_M]]
    # A source pose specifies interaction semantics; its velocity is not a
    # required human command. Fit the shared intermediate velocity freely.
    def objective(x):
        qout,vout,qin,vin=x.reshape(4,6)
        return (source_score(qout,sources[0])+source_score(qin,sources[1])+
                .01*np.sum((vout/vmax)**2)+.02*np.sum((vin/vmax)**2))
    old=pose_coordinates(previous_exit['wrist_xyz_wxyz'],q0)
    old_v=coordinate_velocity(previous_exit['wrist_xyz_wxyz'],previous_exit['wrist_twist'],q0)
    through=np.clip((desired[1]-desired[0])/bridge_budget,-vmax,vmax)
    bounds=[(None,None)]*6+list(zip(-vmax,vmax))+[(None,None)]*6+list(zip(-vmax,vmax))
    attempts=[]
    for velocity in (through,old_v,np.clip((desired[1]-q0)/(tail_budget+bridge_budget),-vmax,vmax)):
        seed=np.r_[old,np.clip(velocity,-vmax,vmax),desired[1],through]
        fit=minimize(objective,seed,method='SLSQP',bounds=bounds,
            constraints=[dict(type='ineq',fun=conditions)],options={'maxiter':180,'ftol':1e-10})
        minimum=float(conditions(fit.x).min())
        attempts.append(dict(solver_success=bool(fit.success),message=str(fit.message),minimum_constraint=minimum))
        if minimum>=-1e-8:break
    else:raise RuntimeError(f'No nominal two-interval grasp boundary within source time: {attempts}')
    qout,vout,qin,vin=fit.x.reshape(4,6)
    return coordinates_pose(qout),vout,dict(method='joint_anchor_exit_and_next_oracle_entry',
        anchor_tail_budget_s=float(tail_budget),relaxed_budget_s=float(bridge_budget),
        exit_source_score=float(source_score(qout,sources[0])),entry_source_score=float(source_score(qin,sources[1])),
        proposed_next_wrist=coordinates_pose(qin).tolist(),proposed_next_coordinate_velocity=vin.tolist(),
        next_source_object_boundary=sources[1].tolist(),
        proposed_exit_coordinate_velocity=vout.tolist(),attempts=attempts,
        original_source_conditions=True,source_time_stretch=False,physical_acceptance=False,
        collision_acceptance=False,state_assignment=False,exit_separation_condition=exit_condition is not None)


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
