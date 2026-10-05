"""Replay stage of local EgoEngine-style synthesis on the mounted embodiment.

Padded human retarget proposals are consumed locally. Core commands are derived
using that context; no commands or targets for relaxed intervals are returned.
Replay failures are explicit refinement requests, never successful artifacts.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def _bounded_quadratic_fit(hessian,linear,matrix,low,high,lower,upper):
    """Solve the existing convex fit directly instead of iterative SLSQP.

    DAQP is already used by the EgoEngine retarget environment. All original
    linear constraints and bounds are retained; no soft constraints are used.
    """
    import daqp
    n=len(linear)
    a=np.vstack((np.eye(n),matrix)).astype(np.float64)
    lo=np.r_[lower,low].astype(np.float64);hi=np.r_[upper,high].astype(np.float64)
    sense=np.where(abs(hi-lo)<1e-12,5,0).astype(np.int32)
    x,_,status,_=daqp.solve(np.ascontiguousarray(hessian,dtype=np.float64),
        np.ascontiguousarray(linear,dtype=np.float64),np.ascontiguousarray(a),
        hi,lo,sense,primal_tol=1e-8,dual_tol=1e-8,iter_limit=10000)
    if status!=1:raise RuntimeError(f'Local quadratic fit failed: DAQP status {status}')
    return x


def fit_hand_startup(desired,start_q,start_dq,dt,limits,velocity_limits,*,startup_duration=None,frame_weights=None):
    """Fit XHand commands within physical joint/speed bounds from measured q,dq.

    When context duration is given, allocate that duration to ramp toward the
    URDF speed limit. This is a synthesis slew budget, not a hardware rating.
    The first interval continues measured velocity without adding source time.
    """
    from scipy.sparse import diags
    weights=np.ones(len(desired)) if frame_weights is None else np.asarray(frame_weights)
    n=len(desired);d=diags([np.ones(n),-np.ones(n-1)],[0,-1],shape=(n,n)).toarray()
    dd=d@d;result=np.empty_like(desired)
    for j in range(desired.shape[1]):
        step=velocity_limits[j]*dt
        low=np.full(n,-1.);high=np.ones(n)
        low[0]=high[0]=start_dq[j]/velocity_limits[j]
        target=desired[:,j]-start_q[j]
        smooth=dd/step;constant=np.zeros(n);constant[0]=-start_dq[j]/velocity_limits[j]
        matrices=[d/step];lows=[low];highs=[high]
        if startup_duration is not None:
            acceleration=velocity_limits[j]/max(dt,startup_duration)
            ac=np.zeros(n);ac[0]=-start_dq[j]/(acceleration*dt)
            matrices.append(dd/(acceleration*dt**2));lows.append(-1-ac);highs.append(1-ac)
        # .5 * weighted tracking error + .005 * normalized acceleration error.
        hessian=np.diag(weights)+.01*smooth.T@smooth
        linear=-weights*target+.01*smooth.T@constant
        solution=_bounded_quadratic_fit(hessian,linear,np.vstack(matrices),
            np.concatenate(lows),np.concatenate(highs),
            limits[j,0]-start_q[j]+np.zeros(n),limits[j,1]-start_q[j]+np.zeros(n))
        result[:,j]=solution+start_q[j]
    speed=np.diff(np.vstack((start_q,result)),axis=0)/dt
    if np.max(abs(speed)/velocity_limits)>1.00001:raise RuntimeError('XHand fitted velocity exceeds URDF limit')
    return result


def hand_limits_from_urdf(scene):
    import xml.etree.ElementTree as ET
    from multipath.xhand import RIGHT_XHAND_FINGER_JOINT_NAMES
    joints={j.get('name'):j.find('limit') for j in ET.parse(scene.args.xhand_urdf).getroot().findall('joint')}
    return np.array([float(joints[name].get('velocity')) for name in RIGHT_XHAND_FINGER_JOINT_NAMES])


def local_wrist_targets(anchor,proposal,held_transform=None,entry_offset=None,*,padded=False):
    """Local Cartesian/finger prior shared by Replay and residual refinement."""
    q=np.asarray(proposal['wrist_hand_q']).copy()
    # Three-frame local smoothing includes pre/post context at core boundaries.
    hand_context=np.pad(q[:,7:],((1,1),(0,0)),mode='edge')
    q[:,7:]=(hand_context[:-2]+2*hand_context[1:-1]+hand_context[2:])/4
    commands=[]
    start,end=(anchor.synth_start,anchor.synth_end) if padded else (anchor.core_start,anchor.core_end)
    for frame in range(start,end):
        i=min(frame+1-anchor.synth_start,len(q)-1)
        wrist=q[i,:7].copy()
        if entry_offset is not None:
            phase=np.clip((frame+1-start)/max(1,anchor.core_start-anchor.synth_start),0,1)
            wrist[:3]+=(1-3*phase*phase+2*phase*phase*phase)*entry_offset
        if held_transform is not None:
            tool=proposal['tool_pose'][i]
            goal=tool@np.linalg.inv(held_transform)
            wrist=np.r_[goal[:3,3],Rotation.from_matrix(goal[:3,:3]).as_quat()[[3,0,1,2]]]
        commands.append(np.r_[wrist,q[i,7:]])
    return np.asarray(commands)


def local_commands(scene,anchor,proposal,held_transform=None,entry_offset=None,*,padded=False):
    targets=local_wrist_targets(anchor,proposal,held_transform,entry_offset,padded=padded)
    seed=np.asarray(scene.state()['arm_q']);commands=[]
    for target in targets:
        seed=scene.solve(target[:7],seed)
        commands.append(np.r_[seed,target[7:]])
    return np.asarray(commands)


def fit_arm_startup(desired,start_q,start_dq,dt,velocity_limits,acceleration_limits,joint_limits,*,startup_duration=None,frame_weights=None):
    """Local arm fit with measured entry velocity and a context-timed ramp.

    An acceleration envelope reduces simultaneous arm/hand startup demand.
    It is a command constraint, not a proof of closed-loop physical feasibility.
    """
    from scipy.sparse import diags
    n=len(desired)
    weights=np.ones(n) if frame_weights is None else np.asarray(frame_weights)
    difference=diags([np.ones(n),-np.ones(n-1)],[0,-1],shape=(n,n)).toarray()
    second=difference@difference
    result=np.empty_like(desired)
    for j in range(7):
        # Optimize displacement from measured entry. Normalize constraint rows
        # so 1/dt^2 does not swamp SLSQP's numerical line search.
        vmax=velocity_limits[j]*.98;amax=float(np.broadcast_to(acceleration_limits,7)[j])*.95
        aconst=np.zeros(n);aconst[0]=-start_dq[j]/(amax*dt)
        matrix=np.vstack((difference/(dt*vmax),second/(dt**2*amax)))
        low=np.r_[-np.ones(n),-np.ones(n)-aconst];high=np.r_[np.ones(n),np.ones(n)-aconst]
        if startup_duration is not None:
            # Preserve measured entry velocity at the first interval; smoothly
            # open acceleration authority inside the existing synthesis context.
            phase=np.clip(np.arange(n)*dt/max(dt,startup_duration),0,1)
            envelope=phase**2*(3-2*phase)
            low[n:]=-envelope-aconst;high[n:]=envelope-aconst
        target=desired[:,j]-start_q[j]
        solution=_bounded_quadratic_fit(np.diag(weights),-weights*target,matrix,low,high,
            np.full(n,joint_limits[j,0]-start_q[j]),np.full(n,joint_limits[j,1]-start_q[j]))
        result[:,j]=solution+start_q[j]
    return result


def fit_timed_controls(arm_targets,hand_targets,start_q,start_dq,start_hand,
                       fps,dt,velocity_limits,acceleration_limits,joint_limits,
                       *,start_hand_dq=None,hand_joint_limits=None,hand_velocity_limits=None,hand_startup_duration=None,core_interval_s=None):
    """Mounted retargeting with velocity/acceleration constraints at physics rate.

    This is kinematic trajectory fitting, not physical MPC or a success gate.
    Only the subsequent actual object/contact rollout validates the interaction.
    """
    tpf=int(round(1/(fps*dt)))
    if tpf<1 or abs(tpf*fps*dt-1)>1e-8:raise ValueError("Physics/source clocks not commensurate")
    n=len(arm_targets)*tpf
    clock=np.arange(len(arm_targets)+1)/fps;query=np.arange(1,n+1)*dt
    # Context supports synthesis; only the oracle core must track closely.
    # This changes fitting priority, never the source clock or hard bounds.
    core=np.ones(n,dtype=bool) if core_interval_s is None else ((query>core_interval_s[0])&(query<=core_interval_s[1]))
    weights=np.where(core,1.,.05)
    reference=np.vstack((start_q,arm_targets))
    desired=np.stack([np.interp(query,clock,reference[:,j]) for j in range(7)],axis=1)
    hand_ref=np.vstack((start_hand,hand_targets))
    hand=np.stack([np.interp(query,clock,hand_ref[:,j]) for j in range(hand_ref.shape[1])],axis=1)
    if hand_joint_limits is not None:
        hand=fit_hand_startup(hand,start_hand,start_hand_dq,dt,hand_joint_limits,hand_velocity_limits,startup_duration=hand_startup_duration,frame_weights=weights)
    result=fit_arm_startup(desired,start_q,start_dq,dt,velocity_limits,acceleration_limits,joint_limits,
                           startup_duration=hand_startup_duration,frame_weights=weights)
    velocity=np.diff(np.vstack((start_q,result)),axis=0)/dt
    acceleration=np.diff(np.vstack((start_dq,velocity)),axis=0)/dt
    vr=float(np.max(np.abs(velocity)/velocity_limits));ar=float(np.max(np.abs(acceleration)/acceleration_limits))
    if vr>1.0001 or ar>1.0001:raise RuntimeError('Local retarget optimizer violated dynamics')
    return np.c_[result,hand,velocity],{'method':'continuous_IK_and_constrained_local_retarget',
                    'control_dt':dt,'startup_context_s':hand_startup_duration,'target_joint_rmse_rad':float(np.sqrt(np.mean((desired-result)**2))),
                    'command_velocity_ratio':vr,'command_acceleration_ratio':ar,
                    'fitting_context_weight':.05,'core_interval_s':core_interval_s,
                    'core_arm_target_rmse_rad':float(np.sqrt(np.mean((desired[core]-result[core])**2))) if core.any() else None}
