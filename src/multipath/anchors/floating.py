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


def rebase_grasp_commands(commands,previous_entry,reached_entry,dt):
    """Retain a local interaction's tool targets under the reached grasp frame.

    The old local wrist command is only a synthesis seed. Its implied tool
    target must be converted through the newly measured hand/tool transform.
    No state is assigned; the resulting core needs fresh physical validation.
    """
    from multipath.execution.mounted_scene import pose_matrix
    if not reached_entry.get('physically_acquired_state'):
        raise ValueError('Core adaptation requires an actual physical entry')
    c=np.asarray(commands).copy()
    transform=np.asarray(previous_entry['hand_object_transform'])@np.linalg.inv(reached_entry['hand_object_transform'])
    old=c[:,:7].copy()
    matrices=np.asarray([pose_matrix(p)@transform for p in old])
    poses=np.c_[matrices[:,:3,3],Rotation.from_matrix(matrices[:,:3,:3]).as_quat()[:,[3,0,1,2]]]
    offset=poses[:,:3]-old[:,:3]
    previous_offset=Rotation.from_rotvec(-c[:,22:25]*dt).apply(offset)
    # Stored drive targets can differ from the measured entry. Reconstructing
    # its first velocity from that target would invent a startup impulse.
    c[:,:7]=poses;c[:,19:22]+=(offset-previous_offset)/dt
    return c


def carry_boundary_commands(scene,anchor,commands,start):
    """Bring context's holding drive posture into CORE, then freeze it.

    A grasp-exporting core must not rely on later padding to finish closing
    its fingers. Only the local synthesis context is used; wrist commands
    and the original source clock are retained. Fresh physics is required.
    """
    if anchor.exit_contact!='grasped':return np.asarray(commands).copy()
    tpf=physics_ticks_per_frame(scene.manifest['fps'],scene.dt)
    end=(anchor.core_end-anchor.synth_start)*tpf
    context=(anchor.core_start-anchor.synth_start+anchor.synth_end-anchor.core_end)*tpf
    original=np.asarray(commands);final=original[-1,7:19].copy()
    core_begin=(anchor.core_start-anchor.synth_start)*tpf
    for begin in range(max(core_begin,end-max(tpf,context)),core_begin-1,-tpf):
        candidate=original.copy()
        old_index=np.arange(begin,len(original))
        warped=np.linspace(begin,len(original)-1,end-begin)
        for joint in range(12):candidate[begin:end,7+joint]=np.interp(warped,old_index,original[begin:,7+joint])
        candidate[end:,7:19]=final
        # A forward slew clamp alone can postpone closure into padding again.
        # Reserve the terminal posture by propagating its speed envelope back.
        step=scene.hand_velocity*scene.dt
        for k in range(end-2,begin-1,-1):
            candidate[k,7:19]=np.clip(candidate[k,7:19],candidate[k+1,7:19]-step,candidate[k+1,7:19]+step)
        previous=hand_command_boundary(start) if begin==0 else candidate[begin-1,7:19]
        if np.any(np.abs(candidate[begin,7:19]-previous)>step+1e-8):continue
        candidate=project_hand_commands(scene,candidate,start,anchor)
        if np.max(np.abs(candidate[end-1:,7:19]-final))<1e-6:return candidate
    raise RuntimeError('Holding drive posture cannot be established inside the Anchor core')


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
    low,high=grasp_command_bounds(scene,start,anchor)
    hand=np.clip(hand,low,high)
    # Finger slew is constrained; no arm IK or joint-space projection exists.
    for k in range(1,len(hand)):
        hand[k]=np.clip(hand[k],hand[k-1]-scene.hand_velocity*scene.dt,hand[k-1]+scene.hand_velocity*scene.dt)
    controls=controls_from_poses(pose,hand,scene.dt)
    if np.max(np.abs((controls[0,7:19]-hand_start)/scene.dt-hand_start_velocity))>1e-5:
        raise ValueError('Measured hand velocity has no feasible continuous first command')
    return project_hand_commands(scene,controls,start,anchor)


def hand_command_boundary(start):
    # Contact can deflect a finger from its drive target. That preload is
    # already present; holding it is not a new high-speed position command.
    if 'command_hand_q' in start:
        return np.asarray(start['command_hand_q'])
    return np.asarray(start['hand_q'])


def grasp_command_bounds(scene,start,anchor=None):
    """Preserve acquired preload only in physically contacting fingers.

    This projects command proposals, not simulator state. Acquisition/release
    Anchors keep their full action space; persistent grasp Anchors can strengthen
    preload and move non-contacting fingers, but cannot command its release.
    """
    low,high=np.asarray(scene.hand_limits,dtype=float).T.copy()
    if (anchor is None or anchor.entry_contact!=anchor.exit_contact or anchor.entry_contact!='grasped'
            or 'command_hand_q' not in start):return low,high
    from multipath.xhand import RIGHT_XHAND_FINGER_JOINT_NAMES,RIGHT_XHAND_CLOSING_DIRECTION
    contact=start['contact'];groups=('thumb','index','mid','ring','pinky')
    active={g for path,force in zip(contact.get('link_paths',[]),contact.get('force_norms_n',[]))
            if force>.01 for g in groups if g in path.rsplit('/',1)[-1]}
    loaded=np.array([any(g in name for g in active) for name in RIGHT_XHAND_FINGER_JOINT_NAMES])
    target=hand_command_boundary(start);direction=np.asarray(RIGHT_XHAND_CLOSING_DIRECTION)
    # Drive error is not the closure direction: a contacting flexion joint can
    # have negative error. Inferring direction from it blocked useful closing
    # and let other candidates actively extend the loaded finger.
    positive=loaded&(direction>0);negative=loaded&(direction<0)
    low[positive]=np.maximum(low[positive],target[positive]);high[negative]=np.minimum(high[negative],target[negative])
    return low,high


def project_hand_commands(scene,commands,start,anchor=None):
    """Keep the recorded drive boundary and joint slew inside source time.

    This changes a proposal, not physics evidence: fresh padded/core Replay
    must still succeed before its commands become a validated artifact.
    """
    controls=np.asarray(commands).copy();previous=hand_command_boundary(start)
    low,high=grasp_command_bounds(scene,start,anchor)
    # A transport-compatible padding may stabilize an existing grasp, but must
    # not synthesize a new finger posture that the frozen-grasp bridge cannot
    # supply. CORE remains free to adapt its fingers under the contact bounds.
    padding=0
    if getattr(scene,'bridge_entry_start',None) is not None and anchor is not None:
        padding=max(0,round((anchor.core_start/scene.manifest['fps']-start['time_s'])/scene.dt))
    for k,c in enumerate(controls):
        if k<padding:c[7:19]=hand_command_boundary(start)
        frame=(start['time_s']+(k+1)*scene.dt)*scene.manifest['fps']
        if (getattr(scene,'freeze_carry_core',False) and anchor is not None
                and anchor.exit_contact=='grasped' and frame>anchor.core_end+1e-8):
            c[7:19]=previous
        c[7:19]=np.clip(c[7:19],low,high)
        c[7:19]=np.clip(c[7:19],previous-scene.hand_velocity*scene.dt,previous+scene.hand_velocity*scene.dt)
        previous=c[7:19].copy()
    return controls


def validate_commands(scene,commands,start):
    c=np.asarray(commands)
    if c.ndim!=2 or c.shape[1]!=25 or len(c)==0 or not np.isfinite(c).all():return {'feasible':False,'reason':'invalid floating controls'}
    q=np.vstack((hand_command_boundary(start),c[:,7:19]));ratio=float(np.max(np.abs(np.diff(q,axis=0)/scene.dt)/scene.hand_velocity))
    in_bounds=bool(np.all(c[:,7:19]>=scene.hand_limits[:,0]-1e-6) and np.all(c[:,7:19]<=scene.hand_limits[:,1]+1e-6))
    return {'feasible':bool(in_bounds and ratio<=1.0001),'hand_velocity_ratio':ratio,'hand_joint_bounds':in_bounds}


def rollout(scene,anchor,data,commands,start,out,mode='LOCAL_PADDED',core_only=False,mesh=True,collect_tracking_failures=False,continue_from_current=False,controller_handoff=False):
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
    if controller_handoff:
        from multipath.execution.wrench_handoff import takeover_bias,bandwidth_handoff_profile
        from multipath.execution.transport_dynamics import TransportDynamics
        drive_bias=takeover_bias(start,commands[0],scene.cfg['controller'])
        model=TransportDynamics(scene,start['contact']['mode']=='grasped')
        rotation=Rotation.from_quat(np.asarray(start['wrist_xyz_wxyz'])[[4,5,6,3]]).as_matrix()
        drive_profile=bandwidth_handoff_profile(len(commands),scene.dt,model.mass,rotation@model.inertia@rotation.T,scene.cfg['controller'])
    if core_only:
        entry=scene.snapshot()
        if not objective.observe(start_frame,commands[0]):
            failure=objective.failure;failures.append(dict(failure))
    try:
        for k,c in enumerate(commands):
            if failure is not None and not collect_tracking_failures:break
            if controller_handoff:
                scene.wrench_bias=drive_bias*drive_profile[k]
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
    if controller_handoff:result['controller_handoff']=dict(initial_wrench_bias=drive_bias.tolist(),extra_time_s=0.,profile='measured_mass_inertia_gain_response')
    if entry is not None:
        from multipath.validation.physics import core_pose_errors,object_tracking_score
        errors=core_pose_errors(entry,data['tool_pose'][anchor.core_start-anchor.synth_start])
        result['core_entry_source_boundary']=dict(source_frame=anchor.core_start,**errors,
                                                  accepted=object_tracking_score(errors)['feasible'])
        predecessor=getattr(scene,'bridge_entry_start',None)
        if predecessor is not None:
            from multipath.planning.bridge import bridge_precheck
            result['bridge_entry_precheck']=bridge_precheck(predecessor,entry,
                anchor.core_start/scene.manifest['fps']-predecessor['time_s'],scene.cfg['planning'])
    if exit_state is not None and getattr(scene,'bridge_exit_constraints',False):
        from multipath.planning.bridge import coordinate_velocity,carried_object_clearance
        speed=coordinate_velocity(exit_state['wrist_xyz_wxyz'],exit_state['wrist_twist'])
        result['bridge_exit_velocity_ratio']=float(np.max(np.abs(speed)/scene.cfg['planning']['velocity_limits']))
        if anchor.exit_contact=='grasped':result['bridge_exit_carried_object_clearance_m']=carried_object_clearance(scene,exit_state)
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
