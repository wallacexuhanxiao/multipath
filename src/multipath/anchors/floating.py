"""Local floating-hand Anchor controls and physical padded/core acceptance."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from multipath.anchors.mpc import WindowEvaluation


def controls_from_poses(poses,hands,dt):
    poses=np.asarray(poses);hands=np.asarray(hands)
    velocity=np.diff(poses[:,:3],axis=0)/dt
    rot=Rotation.from_quat(poses[:,[4,5,6,3]])
    omega=(rot[1:]*rot[:-1].inv()).as_rotvec()/dt
    return np.c_[poses[1:],hands[1:],velocity,omega]


def local_controls(scene,anchor,data,start):
    """Only local source data; bridge state mismatch blends out inside padding."""
    fps=scene.manifest['fps'];tpf=round(1/(fps*scene.dt));frames=np.arange(anchor.synth_start,anchor.synth_end+1)
    # Stored proposals contain [synth_start,synth_end); terminal uses last local target.
    reference=np.asarray(data['wrist_hand_q']);ref=np.vstack((reference,reference[-1]))
    sample=np.arange((len(frames)-1)*tpf+1)/tpf
    source=np.arange(len(ref))
    pose=np.empty((len(sample),7));pose[:,:3]=np.stack([np.interp(sample,source,ref[:,j]) for j in range(3)],axis=1)
    pose[:,3:]=Slerp(source,Rotation.from_quat(ref[:,[4,5,6,3]]))(sample).as_quat()[:,[3,0,1,2]]
    hand=np.stack([np.interp(sample,source,ref[:,j]) for j in range(7,19)],axis=1)
    padding=max(1,(anchor.core_start-anchor.synth_start)*tpf)
    weight=1-np.clip(np.arange(len(sample))/padding,0,1);weight=weight**2*(3-2*weight)
    pose[:,:3]+=weight[:,None]*(np.asarray(start['wrist_xyz_wxyz'][:3])-pose[0,:3])
    delta=Rotation.from_quat(np.asarray(start['wrist_xyz_wxyz'])[[4,5,6,3]])*Rotation.from_quat(pose[0,[4,5,6,3]]).inv()
    rotation=Rotation.from_rotvec(weight[:,None]*delta.as_rotvec())*Rotation.from_quat(pose[:,[4,5,6,3]])
    pose[:,3:]=rotation.as_quat()[:,[3,0,1,2]]
    hand+=weight[:,None]*(np.asarray(start['hand_q'])-hand[0])
    hand=np.clip(hand,scene.hand_limits[:,0],scene.hand_limits[:,1])
    # Finger slew is constrained; no arm IK or joint-space projection exists.
    for k in range(1,len(hand)):
        hand[k]=np.clip(hand[k],hand[k-1]-scene.hand_velocity*scene.dt,hand[k-1]+scene.hand_velocity*scene.dt)
    return controls_from_poses(pose,hand,scene.dt)


def validate_commands(scene,commands,start):
    c=np.asarray(commands)
    if c.ndim!=2 or c.shape[1]!=25 or not np.isfinite(c).all():return {'feasible':False,'reason':'invalid floating controls'}
    q=np.vstack((start['hand_q'],c[:,7:19]));ratio=float(np.max(np.abs(np.diff(q,axis=0)/scene.dt)/scene.hand_velocity))
    in_bounds=bool(np.all(c[:,7:19]>=scene.hand_limits[:,0]-1e-6) and np.all(c[:,7:19]<=scene.hand_limits[:,1]+1e-6))
    return {'feasible':bool(in_bounds and ratio<=1.0001),'hand_velocity_ratio':ratio,'hand_joint_bounds':in_bounds}


def rollout(scene,anchor,data,commands,start,out,mode='LOCAL_PADDED',core_only=False,mesh=True):
    scene.begin_trial(out,start,mesh=mesh);tpf=round(1/(scene.dt*scene.manifest['fps']))
    objective=WindowEvaluation(scene,anchor,data,start);entry=None;exit_state=None;failure=None
    start_frame=anchor.core_start if core_only else anchor.synth_start
    resets=scene.restore_count
    if core_only:entry=scene.snapshot()
    try:
        for k,c in enumerate(commands):
            frame=start_frame+(k+1)/tpf;scene.step_control(c,mode,frame)
            if (k+1)%tpf:continue
            if frame==anchor.core_start:entry=scene.snapshot()
            if frame==anchor.core_end:exit_state=scene.snapshot()
            if not objective.observe(frame,c):
                failure=objective.failure;break
    except (RuntimeError,ValueError) as error:failure={'reason':str(error),'source_frame':scene.ticks*scene.dt*scene.manifest['fps']}
    finally:scene.save()
    result={'success':failure is None,'failure':failure,'core_entry_acquired':entry is not None,'core_exit_acquired':exit_state is not None,
        'state_resets_within_rollout':scene.restore_count-resets,'source_fps':scene.manifest['fps'],'control_dt':scene.dt,
        'embodiment':scene.embodiment,'command_format':'wrist_xyz_wxyz_hand12_twist6','actual_end_frame':scene.ticks*scene.dt*scene.manifest['fps']}
    (Path(out)/'local_result.json').write_text(json.dumps(result,indent=2))
    return result,entry,exit_state
