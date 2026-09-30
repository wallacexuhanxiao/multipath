"""Smooth time parameterization of a measured BC wrist joint path."""
import numpy as np
from scipy.interpolate import CubicSpline


def retime_bc(arm_q, finger_q, fps=30., dt=1/120):
    arm_q, finger_q = np.asarray(arm_q), np.asarray(finger_q)
    t = np.arange(len(arm_q)) / fps
    spline = CubicSpline(t, arm_q, bc_type='clamped')
    dense = np.linspace(0,t[-1], max(100, len(t)*20))
    velocity = np.array([2.175]*4+[2.61]*3)*.7
    acceleration = np.full(7, 5.)
    jerk = np.full(7, 200.)
    factor = max(1., np.max(np.abs(spline(dense,1))/velocity),
                 np.sqrt(np.max(np.abs(spline(dense,2))/acceleration)),
                 np.cbrt(np.max(np.abs(spline(dense,3))/jerk)))
    steps = int(np.ceil(t[-1]*factor/dt))
    query = np.linspace(0,t[-1],steps+1)
    fingers = np.stack([np.interp(query,t,finger_q[:,i]) for i in range(finger_q.shape[1])],axis=1)
    return spline(query), fingers, {'time_scale':float(steps*dt/t[-1]),'duration_s':steps*dt,
                                    'source_duration_s':float(t[-1])}
