"""Fit an existing geometric path to an immutable source budget, or reject."""
import numpy as np


def physics_ticks_per_frame(fps, dt):
    if not np.isfinite([fps,dt]).all() or fps<=0 or dt<=0:
        raise ValueError('Invalid source/physics clock')
    ticks=round(1/(fps*dt))
    if ticks<1 or abs(ticks*fps*dt-1)>1e-8:
        raise ValueError('Physics/source clocks not commensurate')
    return ticks


def fixed_budget_path(raw, raw_dt, ticks, dt, velocity_limits, acceleration_limits,
                      initial_velocity=None, final_velocity=None):
    raw = np.asarray(raw)
    if raw.ndim!=2 or len(raw)<2 or not np.isfinite(raw).all():
        raise ValueError('Invalid planner positions')
    if ticks<1 or not np.isfinite([raw_dt,dt]).all() or raw_dt<=0 or dt<=0:
        raise ValueError('Invalid planner/source clock')
    duration = (len(raw) - 1) * float(raw_dt)
    budget = ticks * dt
    # cuRobo's optimized duration is a proposal, not a proof of minimum time.
    # Compression is allowed ONLY if the resampled physical limits below pass.
    # Include both boundary states; the executor applies entries 1..ticks.
    path = np.stack([np.interp(np.linspace(0, len(raw)-1, ticks+1),
                              np.arange(len(raw)), raw[:,j]) for j in range(raw.shape[1])], axis=1)
    velocity = np.diff(path, axis=0)/dt
    ends = [np.zeros(raw.shape[1]) if v is None else np.asarray(v)
            for v in (initial_velocity, final_velocity)]
    acceleration = np.diff(np.vstack((ends[0], velocity, ends[1])), axis=0)/dt
    vr = float(np.max(np.abs(velocity)/velocity_limits))
    ar = float(np.max(np.abs(acceleration)/acceleration_limits))
    if vr > 1.0001 or ar > 1.0001:
        raise RuntimeError(f'Fixed source budget violates dynamics: velocity={vr:.3f}, acceleration={ar:.3f}')
    return path, {'budget_s':budget, 'curobo_duration_s':duration,
                  'time_scale':budget/max(duration,dt), 'command_velocity_ratio':vr, 'command_acceleration_ratio':ar}


def parameterize_path(raw,ticks,dt,velocity_limits,acceleration_limits,
                      initial_velocity=None,final_velocity=None):
    """Forward/backward reachability in squared path speed; source budget fixed.

    qdot = q'(s)*sdot; qddot = q'(s)*sddot + q''(s)*sdot**2.
    Adjacent squared speeds make these inequalities linear. Both ends of
    every geometric interval are constrained, then the executed samples are
    checked again. The caller must collision-check the interpolated path.
    """
    from scipy.interpolate import CubicSpline
    from scipy.optimize import linprog
    raw=np.asarray(raw)
    distance=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(raw,axis=0),axis=1))]
    keep=np.r_[True,np.diff(distance)>1e-9]
    if distance[-1]<1e-9:
        return fixed_budget_path(raw,dt,ticks,dt,velocity_limits,acceleration_limits,initial_velocity,final_velocity)
    spline=CubicSpline(distance[keep]/distance[-1],raw[keep],axis=0)
    s=np.linspace(0,1,max(4*len(raw),481));ds=s[1]-s[0]
    deriv=spline(s,1);curvature=spline(s,2)
    vlim=np.broadcast_to(np.asarray(velocity_limits),raw.shape[1])*.98
    alim=np.broadcast_to(np.asarray(acceleration_limits),raw.shape[1])*.95
    cap=np.min((vlim/np.maximum(np.abs(deriv),1e-9))**2,axis=1)
    def boundary_speed(velocity,d):
        if velocity is None:return 0.
        speed=float(np.dot(velocity,d)/max(np.dot(d,d),1e-12))
        if speed < -1e-5 or np.max(np.abs(d*max(speed,0)-velocity))>.05:
            raise RuntimeError('Boundary velocity is incompatible with the geometric path')
        return max(speed,0)**2
    first=boundary_speed(initial_velocity,deriv[0]);last=boundary_speed(final_velocity,deriv[-1])
    constraints=[]
    for i in range(len(s)-1):
        at_start=np.stack((-deriv[i]/(2*ds)+curvature[i],deriv[i]/(2*ds)),axis=1)
        at_end=np.stack((-deriv[i+1]/(2*ds),deriv[i+1]/(2*ds)+curvature[i+1]),axis=1)
        matrix=np.vstack((at_start,-at_start,at_end,-at_end))
        constraints.append((matrix,np.tile(alim,4)))
    reachable=cap.copy();reachable[-1]=last
    for i in range(len(s)-2,-1,-1):
        matrix,bounds=constraints[i]
        solution=linprog([-1.,0.],A_ub=matrix,b_ub=bounds,
                         bounds=[(0,cap[i]),(last,last) if i==len(s)-2 else (0,reachable[i+1])],method='highs')
        if not solution.success:raise RuntimeError('No acceleration-feasible terminal connection')
        reachable[i]=solution.x[0]
    if first>reachable[0]+1e-6:raise RuntimeError('Initial velocity cannot connect to terminal boundary')
    squared=np.zeros(len(s));squared[0]=first
    for i,(matrix,bounds) in enumerate(constraints):
        solution=linprog([0.,-1.],A_ub=matrix,b_ub=bounds,
                         bounds=[(squared[i],squared[i]),(last,last) if i==len(s)-2 else (0,reachable[i+1])],method='highs')
        if not solution.success:raise RuntimeError('Path speed propagation failed')
        squared[i+1]=max(0.,solution.x[1])
    speeds=np.sqrt(squared)
    stage_time=2*ds/np.maximum(speeds[:-1]+speeds[1:],1e-12)
    clock=np.r_[0.,np.cumsum(stage_time)];duration=float(clock[-1]);budget=ticks*dt
    if duration>budget+1e-6:
        raise RuntimeError(f'Constrained path needs {duration:.4f}s; source budget {budget:.4f}s')
    query=np.linspace(0,duration,ticks+1)
    idx=np.clip(np.searchsorted(clock,query,side='right')-1,0,len(stage_time)-1)
    elapsed=query-clock[idx];acc=(squared[1:]-squared[:-1])/(2*ds)
    phase=s[idx]+speeds[idx]*elapsed+.5*acc[idx]*elapsed**2
    path=spline(np.clip(phase,0,1));path[0]=raw[0];path[-1]=raw[-1]
    _,check=fixed_budget_path(path,dt,ticks,dt,velocity_limits,acceleration_limits,initial_velocity,final_velocity)
    check.update(parameterization='squared_path_speed_reachability',minimum_computed_duration_s=duration)
    return path,check
