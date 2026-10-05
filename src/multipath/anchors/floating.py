"""Local floating-hand Anchor controls and physical padded/core acceptance."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from multipath.anchors.mpc import WindowEvaluation
from multipath.validation.physics import object_tracking_parameters
from multipath.planning.timing import physics_ticks_per_frame


def validate_local_data(anchor,data):
    frames=np.asarray(data['source_frames']);count=anchor.synth_end-anchor.synth_start
    if not np.array_equal(frames,np.arange(anchor.synth_start,anchor.synth_end)):
        raise ValueError('Local source frames do not match the synthesis interval')
    for key,shape in [('wrist_hand_q',(count,19)),('tool_pose',(count,4,4))]:
        values=np.asarray(data[key])
        if values.shape!=shape or not np.isfinite(values).all():
            raise ValueError(f'Invalid local source array: {key}')


def _entry_curve(reference,actual,velocity,dt,padding):
    """Position and first discrete derivative match the physical entry.

    Hermite correction vanishes with zero derivative at the core boundary;
    no physics steps or source time are added.
    """
    reference=np.asarray(reference);phase=np.clip(np.arange(len(reference))/padding,0,1)
    weight=1-phase**2*(3-2*phase);duration=padding*dt
    tangent=duration*(phase-2*phase**2+phase**3)
    offset=np.asarray(actual)-reference[0]
    # Select the Hermite tangent to match the *discrete* first command slope.
    slope=(np.asarray(actual)+dt*np.asarray(velocity)-reference[1]-weight[1]*offset)/tangent[1]
    return reference+weight[:,None]*offset+tangent[:,None]*slope


def controls_from_poses(poses,hands,dt):
    poses=np.asarray(poses);hands=np.asarray(hands)
    velocity=np.diff(poses[:,:3],axis=0)/dt
    rot=Rotation.from_quat(poses[:,[4,5,6,3]])
    omega=(rot[1:]*rot[:-1].inv()).as_rotvec()/dt
    return np.c_[poses[1:],hands[1:],velocity,omega]


def local_controls(scene,anchor,data,start):
    """Only local source data; bridge state mismatch blends out inside padding."""
    fps=scene.manifest['fps'];tpf=physics_ticks_per_frame(fps,scene.dt);frames=np.arange(anchor.synth_start,anchor.synth_end+1)
    # Stored proposals contain [synth_start,synth_end); terminal uses last local target.
    reference=np.asarray(data['wrist_hand_q']);ref=np.vstack((reference,reference[-1]))
    if anchor.entry_contact==anchor.exit_contact=='grasped':
        # The robot's acquired grasp need not equal the human retarget's grasp.
        # Condition local wrist targets on the source object interaction and
        # the actual hand-object transform. Entry blending below ends inside
        # padding; it must not postpone the canonical source boundary into CORE.
        held=np.asarray(start['hand_object_transform'])
        objects=np.asarray(data['tool_pose']);objects=np.concatenate((objects,objects[-1:]))
        wrists=objects@np.linalg.inv(held)
        ref=ref.copy();ref[:,:3]=wrists[:,:3,3]
        ref[:,3:7]=Rotation.from_matrix(wrists[:,:3,:3]).as_quat()[:,[3,0,1,2]]
    sample=np.arange((len(frames)-1)*tpf+1)/tpf
    source=np.arange(len(ref))
    pose=np.empty((len(sample),7));pose[:,:3]=np.stack([np.interp(sample,source,ref[:,j]) for j in range(3)],axis=1)
    pose[:,3:]=Slerp(source,Rotation.from_quat(ref[:,[4,5,6,3]]))(sample).as_quat()[:,[3,0,1,2]]
    hand=np.stack([np.interp(sample,source,ref[:,j]) for j in range(7,19)],axis=1)
    padding=(anchor.core_start-anchor.synth_start)*tpf
    if padding<2:
        # No available context: verify exact entry instead of moving the core.
        from multipath.execution.mounted_scene import pose_matrix
        if not np.allclose(pose_matrix(pose[0]),pose_matrix(start['wrist_xyz_wxyz']),atol=1e-5):
            raise ValueError('Local entry mismatch has no synthesis context')
        return controls_from_poses(pose,hand,scene.dt)
    weight=1-np.clip(np.arange(len(sample))/padding,0,1);weight=weight**2*(3-2*weight)
    twist=np.asarray(start.get('wrist_twist',np.zeros(6)))
    pose[:,:3]=_entry_curve(pose[:,:3],start['wrist_xyz_wxyz'][:3],twist[:3],scene.dt,padding)
    delta=Rotation.from_quat(np.asarray(start['wrist_xyz_wxyz'])[[4,5,6,3]])*Rotation.from_quat(pose[0,[4,5,6,3]]).inv()
    rotation=Rotation.from_rotvec(weight[:,None]*delta.as_rotvec())*Rotation.from_quat(pose[:,[4,5,6,3]])
    initial=Rotation.from_quat(np.asarray(start['wrist_xyz_wxyz'])[[4,5,6,3]])
    first=(Rotation.from_rotvec(twist[3:]*scene.dt)*initial)*rotation[1].inv()
    phase=np.clip(np.arange(len(sample))/padding,0,1);tangent=phase-2*phase**2+phase**3
    rotation=Rotation.from_rotvec(tangent[:,None]/tangent[1]*first.as_rotvec())*rotation
    pose[:,3:]=rotation.as_quat()[:,[3,0,1,2]]
    hand_start=hand_command_boundary(start)
    preserve_preload='command_hand_q' in start
    hand_start_velocity=np.zeros(12) if preserve_preload else start.get('hand_dq',np.zeros(12))
    hand=_entry_curve(hand,hand_start,hand_start_velocity,scene.dt,padding)
    if anchor.entry_contact==anchor.exit_contact=='grasped' and preserve_preload:
        # Keep contact-loaded joints from releasing their acquired preload,
        # while allowing the retarget to strengthen or adapt the grasp. Holding
        # all fingers rigidly discarded useful closing motion in physical replay.
        preload=hand_start-np.asarray(start['hand_q'])
        positive=preload>1e-6;negative=preload<-1e-6
        hand[:,positive]=np.maximum(hand[:,positive],hand_start[positive])
        hand[:,negative]=np.minimum(hand[:,negative],hand_start[negative])
    hand=np.clip(hand,scene.hand_limits[:,0],scene.hand_limits[:,1])
    # Finger slew is constrained; no arm IK or joint-space projection exists.
    for k in range(1,len(hand)):
        hand[k]=np.clip(hand[k],hand[k-1]-scene.hand_velocity*scene.dt,hand[k-1]+scene.hand_velocity*scene.dt)
    controls=controls_from_poses(pose,hand,scene.dt)
    if np.max(np.abs((controls[0,7:19]-hand_start)/scene.dt-hand_start_velocity))>1e-5:
        raise ValueError('Measured hand velocity has no feasible continuous first command')
    return controls


def hand_command_boundary(start):
    # Contact can deflect a finger from its drive target. That preload is
    # already present; holding it is not a new high-speed position command.
    if 'command_hand_q' in start:
        return np.asarray(start['command_hand_q'])
    return np.asarray(start['hand_q'])


def project_hand_commands(scene,commands,start):
    """Keep the recorded drive boundary and joint slew inside source time.

    This changes a proposal, not physics evidence: fresh padded/core Replay
    must still succeed before its commands become a validated artifact.
    """
    controls=np.asarray(commands).copy();previous=hand_command_boundary(start)
    for c in controls:
        c[7:19]=np.clip(c[7:19],scene.hand_limits[:,0],scene.hand_limits[:,1])
        c[7:19]=np.clip(c[7:19],previous-scene.hand_velocity*scene.dt,previous+scene.hand_velocity*scene.dt)
        previous=c[7:19].copy()
    return controls


def validate_commands(scene,commands,start):
    c=np.asarray(commands)
    if c.ndim!=2 or c.shape[1]!=25 or len(c)==0 or not np.isfinite(c).all():return {'feasible':False,'reason':'invalid floating controls'}
    q=np.vstack((hand_command_boundary(start),c[:,7:19]));ratio=float(np.max(np.abs(np.diff(q,axis=0)/scene.dt)/scene.hand_velocity))
    in_bounds=bool(np.all(c[:,7:19]>=scene.hand_limits[:,0]-1e-6) and np.all(c[:,7:19]<=scene.hand_limits[:,1]+1e-6))
    return {'feasible':bool(in_bounds and ratio<=1.0001),'hand_velocity_ratio':ratio,'hand_joint_bounds':in_bounds}


def rollout(scene,anchor,data,commands,start,out,mode='LOCAL_PADDED',core_only=False,mesh=True,collect_tracking_failures=False,continue_from_current=False):
    tpf=physics_ticks_per_frame(scene.manifest['fps'],scene.dt)
    start_frame=anchor.core_start if core_only else anchor.synth_start
    end_frame=anchor.core_end if core_only else anchor.synth_end
    if len(commands)!=(end_frame-start_frame)*tpf or not np.isclose(start['time_s'],start_frame/scene.manifest['fps'],atol=1e-8,rtol=0):
        raise ValueError('Local rollout commands/entry do not match the source window')
    check=validate_commands(scene,commands,start)
    if not check['feasible']:raise ValueError(f'Local command precheck failed: {check}')
    if continue_from_current:
        from multipath.anchors.compatibility import boundary_errors
        errors=boundary_errors(scene.state(),start)
        if any(v>1e-8 for v in errors.values() if isinstance(v,float)):
            raise ValueError('Continuous local rollout entry differs from current measured state')
        scene.begin_recording(out,mesh=mesh)
    else:scene.begin_trial(out,start,mesh=mesh)
    objective=WindowEvaluation(scene,anchor,data,start);entry=None;exit_state=None;failure=None
    resets=scene.restore_count;failures=[];executed=0
    if core_only:
        entry=scene.snapshot()
        if not objective.observe(start_frame,commands[0]):
            failure=objective.failure;failures.append(dict(failure))
    try:
        for k,c in enumerate(commands):
            if failure is not None and not collect_tracking_failures:break
            frame=start_frame+(k+1)/tpf;scene.step_control(c,mode,frame);executed=k+1
            if (k+1)%tpf:continue
            if frame==anchor.core_start:entry=scene.snapshot()
            if frame==anchor.core_end:exit_state=scene.snapshot()
            if not objective.observe(frame,c):
                current=dict(objective.failure);failures.append(current)
                if failure is None:failure=current
                if collect_tracking_failures and current['reason']=='core_object_fidelity':
                    objective.failure=None
                else:break
    except (RuntimeError,ValueError) as error:
        current={'reason':str(error),'source_frame':scene.ticks*scene.dt*scene.manifest['fps']};failures.append(current)
        if failure is None:failure=current
    finally:scene.save()
    result={'success':failure is None and executed==len(commands) and entry is not None and exit_state is not None,
        'failure':failure,'failures':failures,'padded_rollout_complete':executed==len(commands),
        'tracking_failures_collected':collect_tracking_failures,'core_entry_acquired':entry is not None,'core_exit_acquired':exit_state is not None,
        'state_resets_within_rollout':scene.restore_count-resets,'source_fps':scene.manifest['fps'],'control_dt':scene.dt,
        'tracking_parameters':object_tracking_parameters(),
        'embodiment':scene.embodiment,'command_format':'wrist_xyz_wxyz_hand12_twist6','actual_end_frame':scene.ticks*scene.dt*scene.manifest['fps']}
    if entry is not None:
        from multipath.validation.physics import core_pose_errors,object_tracking_score
        errors=core_pose_errors(entry,data['tool_pose'][anchor.core_start-anchor.synth_start])
        result['core_entry_source_boundary']=dict(source_frame=anchor.core_start,**errors,
                                                  accepted=object_tracking_score(errors)['feasible'])
    (Path(out)/'local_result.json').write_text(json.dumps(result,indent=2))
    return result,entry,exit_state


def object_relative_command(scene,anchor,data,frame,baseline,held_transform,handoff=None):
    """Local Anchor proposal preserving a *measured* grasp transform.

    Reference object poses condition only interaction Anchors. No object state
    is assigned. RELAXED planning never calls this helper.
    """
    poses=np.asarray(data['tool_pose']);last=len(poses)-1
    def target_at(f):
        index=float(np.clip(f-anchor.synth_start,0,last));lo=int(index);hi=min(lo+1,last);alpha=index-lo
        xyz=(1-alpha)*poses[lo,:3,3]+alpha*poses[hi,:3,3]
        if hi==lo:r=Rotation.from_matrix(poses[lo,:3,:3])
        else:r=Slerp([0.,1.],Rotation.from_matrix(poses[[lo,hi],:3,:3]))([alpha])[0]
        matrix=np.eye(4);matrix[:3,:3]=r.as_matrix();matrix[:3,3]=xyz
        matrix=matrix@np.linalg.inv(held_transform)
        if handoff is not None:
            elapsed=(f-handoff['frame'])/scene.manifest['fps']
            phase=np.clip(elapsed/handoff['duration_s'],0.,1.)
            weight=1-phase*phase*(3-2*phase)
            tangent=handoff['duration_s']*(phase-2*phase**2+phase**3)
            matrix[:3,3]+=weight*np.asarray(handoff['position_offset'])+tangent*np.asarray(handoff['position_tangent'])
            matrix[:3,:3]=(Rotation.from_rotvec(tangent*np.asarray(handoff['rotation_tangent']))*
                            Rotation.from_rotvec(weight*np.asarray(handoff['rotation_offset']))*
                            Rotation.from_matrix(matrix[:3,:3])).as_matrix()
        return matrix
    current=target_at(frame);previous=target_at(frame-scene.dt*scene.manifest['fps'])
    command=np.array(baseline).copy();command[:3]=current[:3,3]
    command[3:7]=Rotation.from_matrix(current[:3,:3]).as_quat()[[3,0,1,2]]
    command[19:22]=(current[:3,3]-previous[:3,3])/scene.dt
    command[22:25]=Rotation.from_matrix(current[:3,:3]@previous[:3,:3].T).as_rotvec()/scene.dt
    return command


def object_relative_handoff(scene,anchor,data,frame,held_transform):
    """Match the current drive target before changing the Anchor proposal.

    Fade the target offset during existing source time, sized by the shared
    wrist velocity budget. This does not wait, reset, or relax acceptance.
    """
    previous=np.r_[scene.command_pose,scene.hand_command,scene.command_twist]
    raw=object_relative_command(scene,anchor,data,frame,previous,held_transform)
    offset=previous[:3]-raw[:3]
    rotation=(Rotation.from_quat(previous[[4,5,6,3]])*Rotation.from_quat(raw[[4,5,6,3]]).inv()).as_rotvec()
    velocity=np.asarray(scene.cfg['planning']['velocity_limits'])
    duration=max(1/scene.manifest['fps'],float(np.max(1.5*np.abs(offset)/velocity[:3])),
                 float(1.5*np.linalg.norm(rotation)/np.min(velocity[3:])))
    deadline=anchor.core_end if frame<anchor.core_end else anchor.synth_end
    if duration>(deadline-frame)/scene.manifest['fps']:
        raise RuntimeError('Measured-grasp target handoff exceeds remaining source budget')
    next_raw=object_relative_command(scene,anchor,data,frame+scene.dt*scene.manifest['fps'],previous,held_transform)
    phase=scene.dt/duration;weight=1-phase**2*(3-2*phase);tangent=duration*(phase-2*phase**2+phase**3)
    position_tangent=(previous[:3]+scene.dt*previous[19:22]-next_raw[:3]-weight*offset)/tangent
    base=Rotation.from_rotvec(weight*rotation)*Rotation.from_quat(next_raw[[4,5,6,3]])
    desired=Rotation.from_rotvec(scene.dt*previous[22:25])*Rotation.from_quat(previous[[4,5,6,3]])
    return {'frame':float(frame),'duration_s':duration,'position_offset':offset.tolist(),'rotation_offset':rotation.tolist(),
            'position_tangent':position_tangent.tolist(),'rotation_tangent':((desired*base.inv()).as_rotvec()/tangent).tolist()}
