"""Replay stage of local EgoEngine-style synthesis on the mounted embodiment.

Padded human retarget proposals are consumed locally. Core commands are derived
using that context; no commands or targets for relaxed intervals are returned.
Replay failures are explicit refinement requests, never successful artifacts.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def local_commands(scene,anchor,proposal,held_transform=None,entry_offset=None,*,padded=False):
    q=np.asarray(proposal['wrist_hand_q']).copy()
    # Three-frame local smoothing includes pre/post context at core boundaries.
    hand_context=np.pad(q[:,7:],((1,1),(0,0)),mode='edge')
    q[:,7:]=(hand_context[:-2]+2*hand_context[1:-1]+hand_context[2:])/4
    seed=np.asarray(scene.state()['arm_q']); commands=[]
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
        seed=scene.solve(wrist,seed)
        commands.append(np.r_[seed,q[i,7:]])
    return np.asarray(commands)


def fit_timed_controls(arm_targets,hand_targets,start_q,start_dq,start_hand,
                       fps,dt,velocity_limits,acceleration_limits,joint_limits):
    """Mounted retargeting with velocity/acceleration constraints at physics rate.

    This is kinematic trajectory fitting, not physical MPC or a success gate.
    Only the subsequent actual object/contact rollout validates the interaction.
    """
    from scipy.optimize import minimize, LinearConstraint, Bounds
    from scipy.sparse import diags
    tpf=int(round(1/(fps*dt)));n=len(arm_targets)*tpf
    clock=np.arange(len(arm_targets)+1)/fps;query=np.arange(1,n+1)*dt
    reference=np.vstack((start_q,arm_targets))
    desired=np.stack([np.interp(query,clock,reference[:,j]) for j in range(7)],axis=1)
    hand_ref=np.vstack((start_hand,hand_targets))
    hand=np.stack([np.interp(query,clock,hand_ref[:,j]) for j in range(hand_ref.shape[1])],axis=1)
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
        target=desired[:,j]-start_q[j]
        opt=minimize(lambda q:.5*np.sum((q-target)**2),np.zeros(n),
                     jac=lambda q:q-target,method='SLSQP',
                     bounds=Bounds(joint_limits[j,0]-start_q[j],joint_limits[j,1]-start_q[j]),
                     constraints=[LinearConstraint(matrix,low,high)],
                     options={'maxiter':150,'ftol':1e-8})
        if not opt.success:raise RuntimeError(f'Local dynamic retarget failed joint {j}: {opt.message}')
        result[:,j]=opt.x+start_q[j]
    velocity=np.diff(np.vstack((start_q,result)),axis=0)/dt
    acceleration=np.diff(np.vstack((start_dq,velocity)),axis=0)/dt
    vr=float(np.max(np.abs(velocity)/velocity_limits));ar=float(np.max(np.abs(acceleration)/acceleration_limits))
    if vr>1.0001 or ar>1.0001:raise RuntimeError('Local retarget optimizer violated dynamics')
    return np.c_[result,hand,velocity],{'method':'continuous_IK_and_constrained_local_retarget',
                    'control_dt':dt,'target_joint_rmse_rad':float(np.sqrt(np.mean((desired-result)**2))),
                    'command_velocity_ratio':vr,'command_acceleration_ratio':ar}
