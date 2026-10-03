"""Mounted command projection with discrete braking room at joint limits.

This is an arm execution adaptation, not an extra PPO reward or a claim of
closed-loop feasibility. Actual PhysX velocity checks remain authoritative.
"""
import numpy as np


def stopping_speed(distance, acceleration, dt, vmax):
    # For next-step speed v, exact distance under maximum discrete braking:
    # dt * sum(max(v - k*a*dt, 0)). Include the next commanded step itself.
    lo=np.zeros_like(distance);hi=np.asarray(vmax,dtype=float).copy()
    for _ in range(32):
        mid=(lo+hi)/2
        n=np.ceil(mid/(acceleration*dt))
        travel=dt*(n*mid-acceleration*dt*n*(n-1)/2)
        safe=travel<=np.maximum(distance,0.)
        lo=np.where(safe,mid,lo);hi=np.where(safe,hi,mid)
    return lo


def project_arm_command(proposed,q,dq,dt,limits,vmax,acceleration):
    q=np.asarray(q);dq=np.asarray(dq);acceleration=np.broadcast_to(acceleration,q.shape)
    upper_speed=stopping_speed(limits[:,1]-q,acceleration,dt,vmax)
    lower_speed=stopping_speed(q-limits[:,0],acceleration,dt,vmax)
    low=np.maximum(limits[:,0],q+dt*np.maximum(-lower_speed,dq-acceleration*dt))
    high=np.minimum(limits[:,1],q+dt*np.minimum(upper_speed,dq+acceleration*dt))
    if np.any(low>high+1e-9):
        raise RuntimeError('Residual commands cannot satisfy local actuator boundary')
    high=np.maximum(low,high)  # only floating-point roundoff after the check
    return np.clip(proposed,low,high),low,high
