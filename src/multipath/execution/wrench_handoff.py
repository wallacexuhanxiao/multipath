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
