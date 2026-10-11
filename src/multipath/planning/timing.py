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


def boundary_fitted_path(raw,ticks,dt,velocity_limits,acceleration_limits,
                         initial_velocity=None,final_velocity=None):
    """Fit cuRobo positions with fixed source time and two moving boundaries.

    Minimize L1 deviation from the resampled geometric plan under linear joint
    velocity/acceleration constraints. The caller must collision-check every
    resulting configuration; changing the geometric fit is not a physics pass.
    """
    from scipy.optimize import linprog
    raw=np.asarray(raw);n=ticks+1
    if ticks<3:raise RuntimeError('Moving boundary fitting requires at least three physics intervals')
    desired=np.stack([np.interp(np.linspace(0,len(raw)-1,n),np.arange(len(raw)),raw[:,j]) for j in range(raw.shape[1])],axis=1)
    first=np.zeros(raw.shape[1]) if initial_velocity is None else np.asarray(initial_velocity)
    last=np.zeros(raw.shape[1]) if final_velocity is None else np.asarray(final_velocity)
    identity=np.eye(n);d1=np.diff(identity,axis=0);d2=np.diff(identity,n=2,axis=0)
    matrix=np.vstack((np.c_[d1,np.zeros_like(d1)],np.c_[-d1,np.zeros_like(d1)],
                      np.c_[d2,np.zeros_like(d2)],np.c_[-d2,np.zeros_like(d2)],
                      np.c_[identity,-identity],np.c_[-identity,-identity]))
    path=np.empty_like(desired)
    for j in range(raw.shape[1]):
        rhs=np.r_[np.full(2*(n-1),velocity_limits[j]*dt),np.full(2*(n-2),acceleration_limits[j]*dt**2),desired[:,j],-desired[:,j]]
        bounds=[(None,None)]*n+[(0,None)]*n
        for k,value in [(0,raw[0,j]),(1,raw[0,j]+first[j]*dt),
                        (n-2,raw[-1,j]-last[j]*dt),(n-1,raw[-1,j])]:bounds[k]=(value,value)
        result=linprog(np.r_[np.zeros(n),np.ones(n)],A_ub=matrix,b_ub=rhs,bounds=bounds,method='highs')
        if not result.success:raise RuntimeError(f'No fixed-time boundary trajectory in coordinate {j}')
        path[:,j]=result.x[:n]
    _,check=fixed_budget_path(path,dt,ticks,dt,velocity_limits,acceleration_limits,first,last)
    check.update(parameterization='endpoint_constrained_geometric_fit',
        geometry_max_adjustment=float(np.max(np.abs(path-desired))),boundary_velocities_preserved=True)
    return path,check


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


def collision_fitted_path(path,dt,velocity_limits,acceleration_limits,
                          initial_velocity,final_velocity,clearance_gradient,
                          boundary_condition=None,preserved_coordinates=None,
                          progress_callback=None):
    """Couple wrist position and orientation to fixed-time collision fitting.

    The physical entry remains fixed. A verified exit remains fixed too; an
    unvalidated canonical proposal may move inside its oracle pose condition.
    Linearize the cuRobo signed-distance
    field with an elastic collision penalty, then query the actual field.
    Slack is only an optimizer device: final clearance has no slack. A hard
    linearization at an initially penetrating pose can itself be infeasible.
    The caller still checks geometry and executes real physics.
    """
    from scipy.optimize import linprog
    from scipy.sparse import block_diag,csr_matrix,hstack,vstack
    path=np.asarray(path).copy();desired=path.T.reshape(-1);n,dimension=path.shape
    eye=np.eye(n);d1=np.diff(eye,axis=0);d2=np.diff(eye,n=2,axis=0)
    per_axis=np.vstack((d1,-d1,d2,-d2));dynamics=block_diag([per_axis]*dimension,format='csr')
    identity=csr_matrix(np.eye(dimension*n));zero=csr_matrix(dynamics.shape)
    matrix=vstack((hstack((dynamics,zero)),hstack((identity,-identity)),hstack((-identity,-identity))),format='csr')
    end_count=dimension if boundary_condition is not None else 0
    matrix=hstack((matrix,csr_matrix((matrix.shape[0],n+end_count))),format='csr')
    rhs=np.r_[np.concatenate([np.r_[np.full(2*(n-1),velocity_limits[j]*dt),
        np.full(2*(n-2),acceleration_limits[j]*dt**2)] for j in range(dimension)]),desired,-desired]
    bounds=[(None,None)]*(dimension*n)+[(0,None)]*(dimension*n+n+end_count)
    if end_count:
        terminal=np.zeros((dimension,dimension*n))
        for j in range(dimension):terminal[j,j*n+n-2:j*n+n]=[-1/dt,1/dt]
        zeros=csr_matrix((dimension,dimension*n+n));slack=csr_matrix(np.eye(dimension))
        matrix=vstack((matrix,hstack((csr_matrix(terminal),zeros,-slack)),
                       hstack((-csr_matrix(terminal),zeros,-slack))),format='csr')
        rhs=np.r_[rhs,final_velocity,-np.asarray(final_velocity)]
    fixed=[0,1,n-2,n-1] if boundary_condition is None else [0,1]
    for j in range(dimension):
        for k in fixed:bounds[j*n+k]=(path[k,j],path[k,j])
    equality=None;equality_rhs=None
    if preserved_coordinates is not None:
        # Preserve linear coordinates of the cuRobo proposal, e.g. a table
        # supporter's height/orientation while its in-plane route is fitted.
        preserved=np.asarray(preserved_coordinates)
        equality=hstack((csr_matrix(np.kron(preserved,np.eye(n))),
            csr_matrix((len(preserved)*n,dimension*n+n+end_count))),format='csr')
        equality_rhs=(path@preserved.T).T.reshape(-1)
    # A trust step is two physics intervals of coordinate travel, not an
    # execution delay. Keep the nonlinear distance approximation local.
    trust=2*dt*np.asarray(velocity_limits)
    weights=np.repeat(1/np.asarray(velocity_limits),n)
    objective=np.r_[np.zeros(dimension*n),weights,np.full(n,1e3),
                    1/np.asarray(velocity_limits) if end_count else np.empty(0)]
    required=np.full(n,1e-4)
    # Fixed physical boundaries cannot be moved to acquire an extra solver
    # margin. Require nonpenetration there; retain the interior path margin.
    # A valid 28 micrometre entry otherwise made every LP impossible to finish.
    required[fixed]=0.
    def nonlinear_fallback(reason):
        if boundary_condition is None:raise RuntimeError(reason)
        return nonlinear_collision_fit(path,desired.reshape(dimension,n).T,dt,
            velocity_limits,acceleration_limits,initial_velocity,final_velocity,
            clearance_gradient,boundary_condition,required,reason,
            preserved_coordinates=preserved_coordinates,progress_callback=progress_callback)
    for iteration in range(20):
        clearance,normal=clearance_gradient(path)
        print('COLLISION_TIME_FIT',iteration,float(np.min(clearance)),
              float(np.max(np.linalg.norm(normal[:,:3],axis=1))),
              'fixed',clearance[[0,1,n-2,n-1]].tolist(),flush=True)
        if np.min(clearance-required)>=-1e-7:
            if boundary_condition is not None and np.min(boundary_condition(path[-1])) < -1e-7:
                raise RuntimeError('Collision fit violates unchanged oracle boundary condition')
            end_velocity=final_velocity if boundary_condition is None else (path[-1]-path[-2])/dt
            _,check=fixed_budget_path(path,dt,n-1,dt,velocity_limits,acceleration_limits,initial_velocity,end_velocity)
            check.update(parameterization='collision_constrained_source_time_fit',
                collision_fit_iterations=iteration,minimum_planning_clearance_m=float(np.min(clearance)),
                fixed_boundary_clearance_requirement_m=0.,interior_clearance_requirement_m=1e-4,
                boundary_velocities_preserved=boundary_condition is None,
                physical_entry_velocity_preserved=True,canonical_boundary_jointly_fitted=boundary_condition is not None)
            return path,check
        geometry=hstack([csr_matrix(-normal[:,j,None]*eye) for j in range(dimension)],format='csr')
        geometry=hstack((geometry,csr_matrix((n,dimension*n)),-csr_matrix(eye),csr_matrix((n,end_count))),format='csr')
        geometric_rhs=clearance-required-np.sum(normal*path,axis=1)
        if boundary_condition is not None:
            value=np.atleast_1d(boundary_condition(path[-1]));derivative=np.empty((len(value),dimension));epsilon=1e-5
            for j in range(dimension):
                offset=np.zeros(dimension);offset[j]=epsilon
                derivative[:,j]=(np.asarray(boundary_condition(path[-1]+offset))-
                    np.asarray(boundary_condition(path[-1]-offset)))/(2*epsilon)
            row=np.zeros((len(value),2*dimension*n+n+end_count))
            row[:,np.arange(dimension)*n+n-1]=-derivative
            geometry=vstack((geometry,csr_matrix(row)),format='csr')
            geometric_rhs=np.r_[geometric_rhs,value-derivative@path[-1]]
        local_bounds=bounds.copy()
        for j in range(dimension):
            for k in range(2,n if boundary_condition is not None else n-2):
                local_bounds[j*n+k]=(path[k,j]-trust[j],path[k,j]+trust[j])
        result=linprog(objective,
            A_ub=vstack((matrix,geometry),format='csr'),b_ub=np.r_[rhs,geometric_rhs],
            A_eq=equality,b_eq=equality_rhs,bounds=local_bounds,method='highs')
        if not result.success:return nonlinear_fallback('Source-time collision fit lost nominal boundary feasibility')
        proposed=result.x[:dimension*n].reshape(dimension,n).T
        violation=float(np.maximum(required-clearance,0).sum())
        for fraction in [1.,.5,.25,.125]:
            trial=path+fraction*(proposed-path);trial_clearance,_=clearance_gradient(trial)
            source_ok=boundary_condition is None or np.min(boundary_condition(trial[-1]))>=-1e-7
            if source_ok and np.maximum(required-trial_clearance,0).sum()<violation-1e-8:
                path=trial;break
        else:return nonlinear_fallback('Source-time collision fit cannot reduce physical clearance violation')
    return nonlinear_fallback('Collision-constrained source-time fit did not converge')


def nonlinear_collision_fit(seed,desired,dt,velocity_limits,acceleration_limits,
                            initial_velocity,final_velocity,field,condition,required,reason,
                            preserved_coordinates=None,progress_callback=None):
    """Resolve an observed SCP dead end with the actual nonlinear constraints.

    A tangent-plane endpoint approximation can propose a point outside the
    source ellipsoid for every nonzero line-search step. Jointly fit the
    remaining path and one canonical exit using actual distance/pose queries.
    This is fixed-time fitting of the cuRobo proposal, not simulator execution.
    """
    from scipy.optimize import minimize,LinearConstraint
    from scipy.linalg import null_space
    from time import monotonic
    seed=np.asarray(seed);n,d=seed.shape;velocity=np.asarray(velocity_limits)
    started=monotonic();query_count=0;query_seconds=0.;iterations=0;phase='elastic_collision_search'
    center=seed.reshape(-1);cache={}
    # Parameterize only legal movement. Entry and table-support coordinates
    # remain invariant by construction, rather than dense redundant equalities.
    basis=np.eye(d) if preserved_coordinates is None else null_space(np.asarray(preserved_coordinates))
    if preserved_coordinates is not None and np.max(np.abs(
            (seed-desired)@np.asarray(preserved_coordinates).T))>1e-8:
        raise RuntimeError('Physical entry/path violates preserved coordinates')
    width=basis.shape[1]
    steps=basis*(dt/np.linalg.norm(basis/velocity[:,None],axis=0))
    mapping=np.zeros((n*d,(n-2)*width))
    for k in range(2,n):mapping[k*d:(k+1)*d,(k-2)*width:(k-1)*width]=steps
    def unpack(x):return (center+mapping@x).reshape(n,d)
    def query(x):
        nonlocal query_count,query_seconds
        if 'x' not in cache or not np.array_equal(x,cache['x']):
            before=monotonic()
            c,g=field(unpack(x));cache.update(x=x.copy(),clearance=c,gradient=g)
            query_count+=1;query_seconds+=monotonic()-before
        return cache['clearance'],cache['gradient']
    unit=np.eye(n);v=np.kron(np.diff(unit,axis=0),np.eye(d));a=np.kron(np.diff(unit,n=2,axis=0),np.eye(d))
    dynamics=np.vstack((v,a));limit=np.r_[np.tile(velocity*dt,n-1),np.tile(np.asarray(acceleration_limits)*dt**2,n-2)]
    coefficients=(dynamics@mapping)/limit[:,None];constant=dynamics@center/limit
    variable_rows=np.max(np.abs(coefficients),axis=1)>1e-12
    if np.max(np.abs(constant[~variable_rows]),initial=0)>1.0001:
        raise RuntimeError('Immutable physical entry/support coordinates violate nominal dynamics')
    linear=LinearConstraint(coefficients[variable_rows],
        -1-constant[variable_rows],1-constant[variable_rows])
    def objective(x):
        p=unpack(x);delta=(p-desired)/velocity
        terminal=((p[-1]-p[-2])/dt-final_velocity)/velocity
        value=.5*np.sum(delta**2)/n+.5*np.sum(terminal**2)
        gradient=(p-desired)/(velocity**2*n)
        gradient[-1]+=terminal/(velocity*dt);gradient[-2]-=terminal/(velocity*dt)
        return value,gradient.reshape(-1)@mapping
    clearance_scale=dt*float(np.max(velocity[:3]))
    def collision(x):return (query(x)[0]-required)/clearance_scale
    def collision_jac(x):
        gradient=query(x)[1];matrix=np.zeros((n,n*d))
        for k in range(n):matrix[k,k*d:(k+1)*d]=gradient[k]
        return matrix@mapping/clearance_scale
    def feasibility_objective(x):
        value,gradient=objective(x);residual=np.minimum(collision(x),0.)
        # As in the elastic SCP, allow an infeasible collision linearization
        # during search. Exact nonpenetration remains mandatory at acceptance.
        # Hard collision rows can give SLSQP no step at the colliding seed.
        penalty=1e3
        # Rescale the entire objective, preserving the penalty ratio while
        # preventing enormous first QP steps from a colliding initialization.
        return (value/penalty+.5*np.dot(residual,residual),
                gradient/penalty+residual@collision_jac(x))
    def boundary(x):return np.atleast_1d(condition(unpack(x)[-1]))
    def boundary_jac(x):
        p=unpack(x)[-1];gradient=np.zeros((len(boundary(x)),n*d));epsilon=1e-5
        for j in range(d):
            offset=np.zeros(d);offset[j]=epsilon
            gradient[:,-d+j]=(np.asarray(condition(p+offset))-np.asarray(condition(p-offset)))/(2*epsilon)
        return gradient@mapping
    print('NONLINEAR_COLLISION_FIT_BEGIN',reason,flush=True)
    constraints=[linear,{'type':'ineq','fun':boundary,'jac':boundary_jac}]
    class FeasiblePath(Exception):
        def __init__(self,x):self.x=x.copy()
    def progress(x):
        nonlocal iterations
        iterations+=1;p=unpack(x);clearance=query(x)[0]
        source_values=np.atleast_1d(condition(p[-1]));source_value=float(source_values.min())
        ratios=np.abs(dynamics@p.reshape(-1))/limit
        report=dict(iteration=iterations,phase=phase,elapsed_s=monotonic()-started,
            field_queries=query_count,field_query_seconds=query_seconds,
            clearance_residual_m=float(np.min(clearance-required)),
            source_condition=source_value,nominal_dynamics_ratio=float(ratios.max()))
        report['source_conditions']=source_values.tolist()
        preserved_error=0. if preserved_coordinates is None else float(np.max(np.abs(
            (p-desired)@np.asarray(preserved_coordinates).T)))
        report['preserved_coordinate_error']=preserved_error
        print('NONLINEAR_COLLISION_FIT_PROGRESS',report,flush=True)
        if progress_callback is not None:progress_callback(p,report)
        if (report['clearance_residual_m']>=-1e-7 and source_value>=-1e-6
                and ratios.max()<=1.0001 and preserved_error<=1e-8):
            raise FeasiblePath(x)
    try:
        result=minimize(feasibility_objective,np.zeros(mapping.shape[1]),jac=True,method='SLSQP',
            constraints=constraints,callback=progress,options={'maxiter':80,'ftol':1e-9})
        solution=result.x;message=str(result.message);iterations=int(result.nit)
    except FeasiblePath as result:
        solution=result.x;message='Feasible path reached; geometric polishing stopped'
    # The elastic objective can balance a tiny violation against its reference
    # regularizer. Restore exact feasibility around the resulting candidate,
    # without pulling it back toward the rejected original geometry/velocity.
    candidate=unpack(solution);clearance=query(solution)[0]
    ratios=np.abs(dynamics@candidate.reshape(-1))/limit
    if (np.min(clearance-required)<-1e-7 or np.min(condition(candidate[-1]))<-1e-6
            or ratios.max()>1.0001):
        phase='collision_feasibility_restoration';origin=solution.copy()
        def proximal(x):return .5*np.sum((x-origin)**2),x-origin
        try:
            restored=minimize(proximal,origin,jac=True,method='SLSQP',constraints=[*constraints,
                {'type':'ineq','fun':collision,'jac':collision_jac}],callback=progress,
                options={'maxiter':40,'ftol':1e-10})
            solution=restored.x;message='Feasibility restoration: '+str(restored.message)
        except FeasiblePath as restored:
            solution=restored.x;message='Feasible path restored; geometric polishing stopped'
    path=unpack(solution);clearance=query(solution)[0];source_value=float(np.min(condition(path[-1])))
    preserved_error=0. if preserved_coordinates is None else float(np.max(np.abs(
        (path-desired)@np.asarray(preserved_coordinates).T)))
    print('NONLINEAR_COLLISION_FIT_END',message,'iterations',iterations,
          'clearance',float(np.min(clearance)),'source_condition',float(source_value),flush=True)
    # The CUDA distance field and optimizer use finite precision. This is a
    # normalized numerical residual, not a wider source-pose acceptance gate;
    # the actual Anchor boundary retains the unchanged physical C.4 check.
    source_solver_tolerance=1e-6
    if (not np.isfinite(path).all() or np.min(clearance-required)<-1e-7
            or source_value<-source_solver_tolerance or preserved_error>1e-8):
        raise RuntimeError(f'Nonlinear source-time collision fit rejected: {message}; '
                           f'clearance_residual={float(np.min(clearance-required))}; source_condition={source_value}')
    _,check=fixed_budget_path(path,dt,n-1,dt,velocity_limits,acceleration_limits,
        initial_velocity,(path[-1]-path[-2])/dt)
    check.update(parameterization='nonlinear_collision_constrained_source_time_fit',
        collision_fit_iterations=iterations,minimum_planning_clearance_m=float(np.min(clearance)),
        canonical_boundary_jointly_fitted=True,physical_entry_velocity_preserved=True,
        boundary_velocities_preserved=False,source_condition=float(source_value),
        source_solver_residual_tolerance=source_solver_tolerance,
        fixed_boundary_clearance_requirement_m=0.,interior_clearance_requirement_m=1e-4,
        preceding_scp_failure=reason)
    return path,check
