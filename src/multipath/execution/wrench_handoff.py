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
