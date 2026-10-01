"""Bounded finger preload driven by measured object motion in the hand frame."""
import numpy as np


def update_preload(current, position_drift, rotation_drift, dt, *,
                   position_deadband=.01, rotation_deadband=.12,
                   max_preload=.10, closing_rate=.15):
    """Increase preload only after slip; reset by creating a new grasp state.

    This is a position-target adapter, not an attachment or object-pose servo.
    The caller chooses closing joints and clamps targets to actuator limits.
    """
    if dt<=0 or min(position_deadband,rotation_deadband,max_preload,closing_rate)<=0:
        raise ValueError('Grip feedback parameters must be positive')
    if not np.isfinite([current,position_drift,rotation_drift,dt]).all():
        raise ValueError('Nonfinite grip feedback measurement')
    demand=np.clip(max(position_drift/position_deadband-1,
                       rotation_drift/rotation_deadband-1),0,1)
    return float(np.clip(current+closing_rate*dt*demand,0,max_preload))
