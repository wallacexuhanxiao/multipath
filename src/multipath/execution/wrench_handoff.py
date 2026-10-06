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


def takeover_bias(start,first_command,config):
    """Preserve drive response while keeping measured q/dq planner boundaries.

    This is controller memory, not a pose assignment or time extension. Both
    old and new terms are evaluated at the same actual state.
    """
    pose=np.asarray(start['wrist_xyz_wxyz']);twist=np.asarray(start['wrist_twist'])
    previous=feedback_wrench(start.get('command_wrist',pose),start.get('command_twist',twist),pose,twist,config)
    current=feedback_wrench(first_command[:7],first_command[19:25],pose,twist,config)
    return previous-current


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
