"""Continuous takeover of a floating wrist's existing PD drive response."""
import numpy as np
from scipy.spatial.transform import Rotation


def feedback_wrench(pose,twist,actual,velocity,config):
    pose=np.asarray(pose);actual=np.asarray(actual)
    rotation=Rotation.from_quat(pose[[4,5,6,3]])*Rotation.from_quat(actual[[4,5,6,3]]).inv()
    error=rotation.as_quat();error*=1 if error[3]>=0 else -1
    force=config['wrist_kp']*(pose[:3]-actual[:3])+config['wrist_kd']*(np.asarray(twist)[:3]-velocity[:3])
    torque=2*config['wrist_kp_rot']*error[:3]+config['wrist_kd_rot']*(np.asarray(twist)[3:]-velocity[3:])
    return np.r_[force,torque]


def takeover_bias(start,first_command,config,new_feedforward=None):
    """Preserve drive response while keeping measured q/dq planner boundaries.

    This is controller memory, not a pose assignment or time extension. Both
    old and new terms are evaluated at the same actual state.
    """
    pose=np.asarray(start['wrist_xyz_wxyz']);twist=np.asarray(start['wrist_twist'])
    previous=feedback_wrench(start.get('command_wrist',pose),start.get('command_twist',twist),pose,twist,config)
    previous+=np.asarray(start.get('controller_wrench_bias',np.zeros(6)))
    current=feedback_wrench(first_command[:7],first_command[19:25],pose,twist,config)
    return previous-current-(np.zeros(6) if new_feedforward is None else new_feedforward)


def carried_gravity_wrench(scene):
    """Support an actually grasped object's mass at its measured COM."""
    tool=scene.objects['tool'];p,q=tool.get_world_pose();com,_=tool.get_com()
    center=np.asarray(p).reshape(3)+Rotation.from_quat(np.asarray(q).reshape(4)[[1,2,3,0]]).apply(np.asarray(com).reshape(3))
    wrist,_=scene.root_motion();root_com=np.asarray(scene.av.get_coms())[0,0,:3]
    root_center=wrist[:3]+Rotation.from_quat(wrist[[4,5,6,3]]).apply(root_com)
    force=np.array([0.,0.,float(tool.get_mass())*9.81])
    return np.r_[force,np.cross(center-root_center,force)]


def supported_contact_wrench(scene):
    """Retain measured grasp reaction when the environment supports the load.

    Native normal forces act at their measured contact points. Any remaining
    per-link force uses the link origin. Fresh physics still decides success.
    """
    paths=scene.contacts.prim_paths
    force=np.asarray(scene.contacts.get_contact_force_matrix(dt=scene.dt)).reshape(len(paths),-1,3).sum(1)
    positions,_=scene.contacts.get_world_poses();pose,_=scene.root_motion()
    center=pose[:3]+Rotation.from_quat(pose[[4,5,6,3]]).apply(np.asarray(scene.av.get_coms())[0,0,:3])
    normal_force,points,normals,_,counts,starts=scene.contacts.get_contact_force_data(dt=scene.dt)
    normal_force=np.asarray(normal_force).reshape(-1);points=np.asarray(points);normals=np.asarray(normals)
    counts=np.asarray(counts).reshape(len(paths),-1);starts=np.asarray(starts).reshape(len(paths),-1)
    native_moment=np.zeros(3);normal_by_link=np.zeros_like(force)
    for link in range(len(paths)):
        for count,start in zip(counts[link],starts[link]):
            indices=np.arange(int(start),int(start)+int(count))
            values=normal_force[indices,None]*normals[indices]
            normal_by_link[link]+=values.sum(0)
            native_moment+=np.cross(points[indices]-center,values).sum(0)
    residual=force-normal_by_link
    moment=native_moment+np.cross(np.asarray(positions)-center,residual).sum(0)
    scene.last_support_contact_wrench=dict(force_n=force.sum(0).tolist(),moment_nm=moment.tolist(),
        native_point_normal_moment_nm=native_moment.tolist(),
        link_origin_moment_nm=np.cross(np.asarray(positions)-center,force).sum(0).tolist(),
        remaining_link_force_n=residual.sum(0).tolist(),normal_force_at_native_contact_points=True)
    return -np.r_[force.sum(0),moment]


def measured_force_balance(scene):
    """Read the early handoff's physical force balance without a new gate."""
    masses=np.asarray(scene.contacts.get_masses()).reshape(-1)
    velocities=np.asarray(scene.contacts._physics_view.get_velocities()).reshape(-1,6)
    filtered=np.asarray(scene.contacts.get_contact_force_matrix(dt=scene.dt)).reshape(len(masses),-1,3).sum(1)
    total=np.asarray(scene.contacts.get_net_contact_forces(dt=scene.dt)).reshape(-1,3)
    return dict(time_s=scene.ticks*scene.dt,hand_mass_kg=float(masses.sum()),
        hand_com_velocity_m_s=(np.sum(masses[:,None]*velocities[:,:3],axis=0)/masses.sum()).tolist(),
        commanded_wrench=np.asarray(scene.last_wrench).tolist(),
        filtered_object_contact_force_n=filtered.sum(0).tolist(),
        all_contact_force_n=total.sum(0).tolist(),
        other_contact_force_n=(total-filtered).sum(0).tolist(),
        diagnostic_only=True)


def support_hold_commands(scene,start,ticks):
    """Continue actual stabilizing drive memory within the same source budget."""
    from multipath.anchors.floating import stopping_context
    program=getattr(scene,'support_hold_program',None)
    scene.last_support_hold_wrenches=None
    if program is not None:
        commands=np.asarray(program['commands']);wrenches=np.asarray(program['wrenches'])
        take=min(ticks,len(commands))
        result=np.concatenate((commands[:take],np.repeat(commands[-1:],ticks-take,axis=0)))
        scene.last_support_hold_wrenches=np.concatenate((wrenches[:take],np.repeat(wrenches[-1:],ticks-take,axis=0)))
        scene.support_hold_program=None
        return result
    last=np.r_[start['command_wrist'],start['command_hand_q'],start['command_twist']]
    return stopping_context(last,ticks,scene.dt)


def bandwidth_handoff_profile(ticks,dt,mass,inertia_world,config):
    """Fade drive memory on the model's response scale, inside source time.

    Using the whole bridge budget as the controller's response time leaves
    a long displacement impulse. These scales depend on mass/inertia/gains,
    not on the task, source frames or a hand-tuned delay.
    """
    durations=np.r_[np.full(3,np.sqrt(mass/config['wrist_kp'])),
        np.sqrt(np.maximum(np.diag(inertia_world),1e-12)/config['wrist_kp_rot'])]
    durations=np.minimum(durations,max(dt,(ticks-1)*dt))
    phase=np.clip(np.arange(ticks)[:,None]*dt/durations,0.,1.)
    return 1-3*phase**2+2*phase**3


def supported_payload_wrenches(scene,start,commands,hand_model):
    """The table supports normal load, not tangential/yaw payload inertia."""
    from multipath.execution.transport_dynamics import TransportDynamics
    payload_model=TransportDynamics(scene,True)
    mass=payload_model.mass-hand_model.mass
    if mass<=0:raise ValueError('Invalid measured supported payload mass')
    extra=payload_model.commands(scene,start,commands)-hand_model.commands(scene,start,commands)
    table=np.asarray(scene.table['pose'])
    normal=Rotation.from_quat(table[[4,5,6,3]]).apply([0.,0.,1.])
    center=(payload_model.mass*payload_model.center-hand_model.mass*hand_model.center)/mass
    lever=Rotation.from_quat(np.asarray(commands)[:,[4,5,6,3]]).apply(center-hand_model.root_com)
    force=extra[:,:3]-np.outer(extra[:,:3]@normal,normal)
    intrinsic=extra[:,3:]-np.cross(lever,extra[:,:3])
    torque=np.outer(intrinsic@normal,normal)+np.cross(lever,force)
    scene.last_supported_payload_dynamics=dict(payload_mass_kg=mass,
        support_normal=normal.tolist(),normal_load_supplied_by_environment=True,
        tangential_inertia_retained=True,yaw_inertia_retained=True,
        maximum_added_force_n=float(np.max(np.linalg.norm(force,axis=1))),
        maximum_added_moment_nm=float(np.max(np.linalg.norm(torque,axis=1))),
        physical_validation_required=True)
    return np.c_[force,torque]


def segment_wrenches(scene,start,commands,*,transport=False,baseline_commands=None,free_motion=False,supported_object=None):
    """Keep the established takeover; validate new payload terms in physics."""
    from multipath.execution.transport_dynamics import TransportDynamics
    carry=start['contact']['mode']=='grasped'
    if supported_object is None:supported_object=getattr(scene,'surface_supported_object',False)
    # A declared surface-supported object still has native grasp constraints,
    # but its gravity is supported by the environment. Treating its full
    # mass/COM as a rigid airborne payload applies a spurious wrist moment.
    # The actual coupled rollout, not this approximate model, is acceptance.
    model=TransportDynamics(scene,carry and not supported_object)
    feedforward=model.commands(scene,start,commands) if (transport and carry) or (free_motion and not carry) else np.zeros((len(commands),6))
    if transport and carry and supported_object:
        feedforward+=supported_payload_wrenches(scene,start,commands,model)
        # Fading all previous drive response removes the wrist's contact
        # preload as well as its transient tracking error. A hand-only mass
        # model supplies no replacement for that measured reaction.
        feedforward+=supported_contact_wrench(scene)
    if baseline_commands is not None and not carry and not free_motion:
        # Spatial detours accelerate the free assembly about its off-center
        # COM. Compensate only the difference from the already valid motion;
        # its existing gravity/tracking bias remains part of the baseline.
        feedforward+=model.commands(scene,start,commands)-model.commands(scene,start,baseline_commands)
    bias=takeover_bias(start,commands[0],scene.cfg['controller'],new_feedforward=feedforward[0])
    rotation=Rotation.from_quat(np.asarray(start['wrist_xyz_wxyz'])[[4,5,6,3]]).as_matrix()
    inertia=rotation@model.inertia@rotation.T;cfg=scene.cfg['controller']
    profile=bandwidth_handoff_profile(len(commands),scene.dt,model.mass,inertia,cfg)
    origin='measured_response_bandwidth'
    scene.last_drive_memory_profile=dict(method=origin,model=model.report(),
        duration_s=len(commands)*scene.dt,source_time_stretch=False,physical_validation_required=True)
    if transport and carry and supported_object:
        scene.last_drive_memory_profile['supported_payload_dynamics']=scene.last_supported_payload_dynamics
    return feedforward+bias*profile


def balanced_force_profile(ticks,dt,mass,kp,kd):
    """Smooth takeover with zero terminal position/velocity footprint.

    The linear closed-loop PD model sizes this input, not source-specific
    tuning. Actual coupled hand/object execution must still validate it.
    """
    from scipy.linalg import expm
    dynamics=np.array([[0.,1.,0.],[-kp/mass,-kd/mass,1/mass],[0.,0.,0.]])
    transition=expm(dynamics*dt);a=transition[:2,:2];b=transition[:2,2]
    response=np.stack([np.linalg.matrix_power(a,ticks-1-k)@b for k in range(ticks)],axis=1)
    unit=np.eye(ticks);d2=np.diff(unit,n=2,axis=0)
    constraints=np.vstack((unit[0],unit[1]-unit[0],unit[-1],unit[-1]-unit[-2],response))
    rhs=np.r_[1.,0.,0.,0.,0.,0.]
    cost=d2.T@d2+np.eye(ticks)*1e-6
    inverse=np.linalg.solve(cost,constraints.T)
    profile=inverse@np.linalg.solve(constraints@inverse,rhs)
    if np.max(np.abs(constraints@profile-rhs))>1e-7:raise RuntimeError('Source budget too short for continuous drive takeover')
    return profile
