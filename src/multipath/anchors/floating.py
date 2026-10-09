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


def stopping_context(last_command,ticks,dt):
    """Local post-padding proposal: retain finger drive and stop smoothly.

    Padding has no reference-fidelity requirement. This is a command proposal,
    never a constructed entry state or proof that contact will be retained.
    """
    if ticks==0:return np.empty((0,25))
    last=np.asarray(last_command,dtype=float)
    phase=np.arange(1,ticks+1)/ticks
    scale=1-3*phase**2+2*phase**3
    elapsed=np.cumsum(scale)*dt
    post=np.repeat(last[None],ticks,axis=0)
    post[:,:3]=last[:3]+elapsed[:,None]*last[19:22]
    post[:,3:7]=(Rotation.from_rotvec(elapsed[:,None]*last[22:25])*
        Rotation.from_quat(last[[4,5,6,3]])).as_quat()[:,[3,0,1,2]]
    post[:,19:25]=scale[:,None]*last[19:25]
    return post


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
    transform=grasp_drive_transform(previous_entry)@np.linalg.inv(grasp_drive_transform(reached_entry))
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
        held=grasp_drive_transform(start)
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


def grasp_drive_transform(state,wrist_command=None):
    """Tool-relative wrist drive, retaining the physically applied preload.

    This is a command-space target. Actual hand/tool transforms remain the
    measured states used for retention, artifacts and bridge planning.
    """
    from multipath.execution.mounted_scene import pose_matrix
    command=state.get('command_wrist',state['wrist_xyz_wxyz']) if wrist_command is None else wrist_command
    return np.linalg.inv(pose_matrix(command))@pose_matrix(state['objects']['tool']['pose'])


def grasp_command_bounds(scene,start,anchor=None):
    """Preserve acquired preload only in physically contacting fingers.

    This projects command proposals, not simulator state. Acquisition/release
    Anchors keep their full action space; persistent grasp Anchors can strengthen
    preload and move non-contacting fingers, but cannot command its release.
    """
    low,high=np.asarray(scene.hand_limits,dtype=float).T.copy()
    if (anchor is None or anchor.entry_contact!='grasped' or anchor.exit_contact!='grasped'
            or start.get('contact',{}).get('mode')!='grasped'
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


def acquisition_preload(scene,anchor,command,bounds=None):
    """Preserve preload only during an already acquired persistent grasp.

    This projects and records commands during synthesis. The resulting
    command trajectory must still pass fresh padded/core physical Replay.
    Acquisition and release Anchors retain their original finger action space.
    Transient opposition during acquisition is not a load-bearing grasp and
    must not lock all subsequent candidates into monotonically closing drives.
    """
    if anchor.entry_contact!='grasped' or anchor.exit_contact!='grasped':return command,bounds
    measured=scene.snapshot()
    if measured['contact']['mode']=='grasped':
        low,high=grasp_command_bounds(scene,measured,anchor)
        bounds=(low,high) if bounds is None else (np.maximum(bounds[0],low),np.minimum(bounds[1],high))
    if bounds is None:return command,bounds
    result=np.asarray(command).copy();step=scene.hand_velocity*scene.dt
    result[7:19]=np.clip(result[7:19],np.maximum(bounds[0],scene.hand_command-step),
                         np.minimum(bounds[1],scene.hand_command+step))
    return result,bounds


def project_hand_commands(scene,commands,start,anchor=None):
    """Keep the recorded drive boundary and joint slew inside source time.

    This changes a proposal, not physics evidence: fresh padded/core Replay
    must still succeed before its commands become a validated artifact.
    """
    controls=np.asarray(commands).copy();previous=hand_command_boundary(start)
    if (getattr(scene,'freeze_carry_core',False) and anchor is not None
            and anchor.exit_contact=='grasped'):
        core_tick=round((anchor.core_end/scene.manifest['fps']-start['time_s'])/scene.dt)
        if 0<core_tick<len(controls):
            # Residual optimization can move the last core target. Padding
            # must stop that optimized target, not jump to the old baseline.
            post_ticks=physics_ticks_per_frame(scene.manifest['fps'],scene.dt)*(anchor.synth_end-anchor.core_end)
            bridge_context=getattr(scene,'post_context_commands',None)
            if bridge_context is None:
                post=stopping_context(controls[core_tick-1],post_ticks,scene.dt)
            else:
                # Replay the observed carry load as context. Rejoining its
                # wrist to the old CORE drive altered the failed bridge and
                # falsely made an unchanged grasp pass local validation.
                # These commands remain a proposal, never a RELAXED artifact.
                last=controls[core_tick-1]
                post=np.asarray(bridge_context).copy();post[:,7:19]=last[7:19]
            controls[core_tick:]=post[:len(controls)-core_tick]
    low,high=grasp_command_bounds(scene,start,anchor)
    # A transport-compatible padding may stabilize an existing grasp, but must
    # not synthesize a new finger posture that the frozen-grasp bridge cannot
    # supply. CORE remains free to adapt its fingers under the contact bounds.
    padding=0
    if getattr(scene,'bridge_entry_start',None) is not None and anchor is not None:
        padding=max(0,round((anchor.core_start/scene.manifest['fps']-start['time_s'])/scene.dt))
    for k,c in enumerate(controls):
        if k<padding:c[7:19]=hand_command_boundary(start)
        if (k==0 and anchor is not None and anchor.entry_contact=='grasped'
                and np.isclose(start['time_s'],anchor.core_start/scene.manifest['fps'],atol=1e-8,rtol=0)):
            c[7:19]=hand_command_boundary(start)
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


def rollout(scene,anchor,data,commands,start,out,mode='LOCAL_PADDED',core_only=False,mesh=True,collect_tracking_failures=False,continue_from_current=False,controller_handoff=False,synthesize_acquisition=False):
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
    commands=np.asarray(commands).copy()
    acquisition=(AcquiredGraspProposal(scene,anchor,data) if synthesize_acquisition and
                 anchor.entry_contact=='free' and anchor.exit_contact=='grasped' else None)
    objective=WindowEvaluation(scene,anchor,data,start);entry=None;exit_state=None;failure=None
    resets=scene.restore_count;failures=[];executed=0;post_contact_steps=None
    if controller_handoff:
        from multipath.execution.wrench_handoff import segment_wrenches
        stable_carry=anchor.entry_contact==anchor.exit_contact=='grasped'
        handoff_wrenches=segment_wrenches(scene,start,commands,transport=stable_carry)
        drive_bias=handoff_wrenches[0].copy()
    if start_frame==anchor.core_start:
        entry=scene.snapshot()
        # A restored development snapshot carries contact evidence from its
        # real acquisition; native contact buffers refresh at the first step.
        if not objective.observe(start_frame,commands[0],measured=entry):
            failure=objective.failure;failures.append(dict(failure))
    try:
        for k,c in enumerate(commands):
            if failure is not None and not collect_tracking_failures:break
            frame=start_frame+(k+1)/tpf
            if acquisition is not None and frame<=anchor.core_end:
                c=acquisition.command(frame,c);commands[k]=c
                if frame==anchor.core_end:
                    commands[k+1:]=project_hand_commands(scene,commands,start,anchor)[k+1:]
            if controller_handoff:
                if getattr(scene,'post_context_commands',None) is not None and start_frame+(k/tpf)>=anchor.core_end:
                    if start_frame+(k/tpf)==anchor.core_end:
                        from multipath.execution.wrench_handoff import segment_wrenches
                        post_wrenches=segment_wrenches(scene,scene.snapshot(),commands[k:],transport=True)
                        post_begin=k
                    scene.wrench_bias=post_wrenches[k-post_begin]
                else:scene.wrench_bias=handoff_wrenches[k]
            bridge_context=(getattr(scene,'post_context_commands',None) is not None
                            and frame>anchor.core_end and objective.held is not None)
            if bridge_context and post_contact_steps is None:post_contact_steps=scene.tool_target_contact_steps
            scene.step_control(c,mode,frame);executed=k+1
            if (getattr(scene,'post_context_commands',None) is not None
                    and frame>anchor.core_end and objective.held is not None):
                from multipath.validation.physics import check_retention,check_relaxed_tool_target_contact
                check_retention(scene.state(),objective.held)
                check_relaxed_tool_target_contact(scene,post_contact_steps)
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
    finally:
        scene.save()
        if acquisition is not None:
            # Only actually executed drives are evidence. An incomplete
            # trial's untouched future proposal must not be used as Replay.
            np.savez_compressed(Path(out)/'executed_commands.npz',commands=commands[:executed])
    result={'success':failure is None and executed==len(commands) and entry is not None and exit_state is not None,
        'failure':failure,'failures':failures,'rollout_complete':executed==len(commands),
        'padded_rollout_complete':executed==len(commands) and not core_only,
        'core_rollout_complete':executed==len(commands) and core_only,
        'rollout_interval':[start_frame,end_frame],
        'tracking_failures_collected':collect_tracking_failures,'core_entry_acquired':entry is not None,'core_exit_acquired':exit_state is not None,
        'state_resets_within_rollout':scene.restore_count-resets,'source_fps':scene.manifest['fps'],'control_dt':scene.dt,
        'tracking_parameters':object_tracking_parameters(),
        'embodiment':scene.embodiment,'command_format':'wrist_xyz_wxyz_hand12_twist6','actual_end_frame':scene.ticks*scene.dt*scene.manifest['fps']}
    if controller_handoff:result['controller_handoff']=dict(initial_wrench_bias=drive_bias.tolist(),extra_time_s=0.,
        profile='measured_mass_inertia_gain_response',carried_dynamics_feedforward=stable_carry)
    if acquisition is not None:
        result['synthesis_controller']='measured_tool_acquisition_then_source_object_motion'
        result['executed_commands_path']=str(Path(out)/'executed_commands.npz')
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
    """Local object-conditioned drive; actual grasp is validated separately.

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


def object_relative_handoff(scene,anchor,data,frame,held_transform,previous_command=None):
    """Match the current drive target before changing the Anchor proposal.

    Fade the target offset during existing source time, sized by the shared
    wrist velocity and acceleration budgets. This is nominal time fitting;
    native contact/retention still decides whether the handoff works.
    """
    previous=(np.r_[scene.command_pose,scene.hand_command,scene.command_twist]
              if previous_command is None else np.asarray(previous_command))
    raw=object_relative_command(scene,anchor,data,frame,previous,held_transform)
    offset=previous[:3]-raw[:3]
    rotation=(Rotation.from_quat(previous[[4,5,6,3]])*Rotation.from_quat(raw[[4,5,6,3]]).inv()).as_rotvec()
    velocity=np.asarray(scene.cfg['planning']['velocity_limits'])
    acceleration=np.asarray(scene.cfg['planning']['acceleration_limits'])
    # Velocity-only fitting caught up several centimetres in one source
    # frame after opposition appeared, then physically pulled the hand off
    # the resting tool. Size the same Hermite offset by its acceleration too.
    # This changes commands, never measured state or the source deadline.
    duration=max(1/scene.manifest['fps'],float(np.max(1.5*np.abs(offset)/velocity[:3])),
                 float(1.5*np.linalg.norm(rotation)/np.min(velocity[3:])),
                 float(np.max(np.sqrt(6*np.abs(offset)/acceleration[:3]))),
                 float(np.sqrt(6*np.linalg.norm(rotation)/np.min(acceleration[3:]))))
    deadline=anchor.core_end if frame<anchor.core_end else anchor.synth_end
    if duration>(deadline-frame)/scene.manifest['fps']:
        raise RuntimeError('Measured-grasp target handoff exceeds remaining source budget')
    next_raw=object_relative_command(scene,anchor,data,frame+scene.dt*scene.manifest['fps'],previous,held_transform)
    phase=scene.dt/duration;weight=1-phase**2*(3-2*phase);tangent=duration*(phase-2*phase**2+phase**3)
    position_tangent=(previous[:3]+scene.dt*previous[19:22]-next_raw[:3]-weight*offset)/tangent
    base=Rotation.from_rotvec(weight*rotation)*Rotation.from_quat(next_raw[[4,5,6,3]])
    desired=Rotation.from_rotvec(scene.dt*previous[22:25])*Rotation.from_quat(previous[[4,5,6,3]])
    return {'frame':float(frame),'duration_s':duration,'duration_sizing':'nominal_velocity_and_acceleration_offset',
            'position_offset':offset.tolist(),'rotation_offset':rotation.tolist(),
            'position_tangent':position_tangent.tolist(),'rotation_tangent':((desired*base.inv()).as_rotvec()/tangent).tolist()}


def source_grasp_core_controls(scene,anchor,data,start):
    """Fit local object-conditioned Replay from the actually reached q/dq.

    A single Hermite offset duration shared by translation and rotation kept
    translation momentum alive until the slower rotation handoff completed.
    Fit each coordinate to the local source under the existing nominal budgets
    instead. Controller takeover retains the previous wrench/preload; neither
    measured state nor the source clock is changed. Physics decides acceptance.
    """
    if not (start.get('physically_acquired_state') and start['contact']['mode']=='grasped'
            and anchor.entry_contact==anchor.exit_contact=='grasped'):
        raise ValueError('Carry core synthesis requires a physically reached persistent grasp')
    from multipath.planning.floating import pose_coordinates,coordinates_pose
    from multipath.planning.bridge import coordinate_velocity
    from multipath.planning.timing import boundary_fitted_path
    tpf=physics_ticks_per_frame(scene.manifest['fps'],scene.dt)
    ticks=(anchor.core_end-anchor.core_start)*tpf
    held=np.asarray(start['hand_object_transform'])
    baseline=np.r_[start['wrist_xyz_wxyz'],hand_command_boundary(start),start['wrist_twist']]
    poses=[np.asarray(start['wrist_xyz_wxyz'])]
    poses.extend(object_relative_command(scene,anchor,data,anchor.core_start+k/tpf,
        baseline,held)[:7] for k in range(1,ticks+1))
    near=pose_coordinates(poses[0]);raw=[]
    for pose in poses:
        near=pose_coordinates(pose,near);raw.append(near.copy())
    raw=np.asarray(raw);limits=scene.cfg['planning']
    velocity=np.asarray(limits['velocity_limits']);acceleration=np.asarray(limits['acceleration_limits'])
    first=coordinate_velocity(poses[0],start['wrist_twist'],raw[0])
    # Source motion initializes the canonical exit velocity, but is not a
    # physical boundary invariant. Limit the proposal to the shared budgets.
    last=np.clip((raw[-1]-raw[-2])/scene.dt,-velocity,velocity)
    path,fit=boundary_fitted_path(raw,ticks,scene.dt,velocity,acceleration,first,last)
    fit.pop('curobo_duration_s',None)
    fitted_poses=np.asarray([coordinates_pose(q) for q in path])
    hands=np.repeat(hand_command_boundary(start)[None],ticks+1,axis=0)
    commands=controls_from_poses(fitted_poses,hands,scene.dt)
    fit.update(frame=anchor.core_start,initial_boundary='actual_measured_wrist_pose_and_twist',
        source_conditioning='local_tool_pose_with_measured_hand_object_transform',
        command_target_continuity='existing_wrench_takeover',curobo_used=False,
        local_fit_duration_s=ticks*scene.dt,physical_validation_required=True)
    return project_hand_commands(scene,commands,start,anchor),fit


class AcquiredGraspProposal:
    """Existing object-relative MPC proposal, initialized by real opposition."""
    def __init__(self,scene,anchor,data):
        self.scene,self.anchor,self.data=scene,anchor,data
        self.contact_run=0;self.contact_loss_run=0;self.held=None;self.handoff=None;self.acquired_frame=None
        self.drive_held=None
        self.preload_bounds=None
        from multipath.execution.mounted_scene import pose_matrix
        self.entry_tool=pose_matrix(scene.state()['objects']['tool']['pose'])
        self.source_entry=data['tool_pose'][anchor.core_start-anchor.synth_start]
        self.stationary_prior=scene.manifest.get('retarget_details',{}).get(anchor.id,{}).get('pre_acquisition_tool_motion')=='stationary_entry'

    def acquisition_command(self,frame,baseline,measured,previous_command=None):
        """Execute an acquisition proposal relative to the real resting tool.

        Light one-sided contact can slide the tool before opposition forms.
        Keeping a world-fixed human target then closes around empty space.
        This is measured-state wrist control, not object placement or proof.
        """
        from multipath.execution.mounted_scene import pose_matrix
        reference=self.entry_tool
        if not self.stationary_prior:
            index=float(np.clip(frame-self.anchor.synth_start,0,len(self.data['tool_pose'])-1))
            lo=int(index);hi=min(lo+1,len(self.data['tool_pose'])-1);alpha=index-lo
            source=self.data['tool_pose'][lo].copy()
            if hi!=lo:
                source[:3,3]=(1-alpha)*source[:3,3]+alpha*self.data['tool_pose'][hi,:3,3]
                source[:3,:3]=Slerp([0,1],Rotation.from_matrix(self.data['tool_pose'][[lo,hi],:3,:3]))([alpha])[0].as_matrix()
            reference=self.entry_tool@np.linalg.inv(self.source_entry)@source
        actual=pose_matrix(measured['objects']['tool']['pose']);delta=actual@np.linalg.inv(reference)
        matrix=delta@pose_matrix(baseline[:7]);command=np.asarray(baseline).copy()
        command[:3]=matrix[:3,3];command[3:7]=Rotation.from_matrix(matrix[:3,:3]).as_quat()[[3,0,1,2]]
        # PPO supplies the previous nominal drive so sampled pose residuals
        # do not become velocity impulses. Replay uses its actual last drive.
        previous=(self.scene.command_pose if previous_command is None else previous_command[:7])
        command[19:22]=(command[:3]-previous[:3])/self.scene.dt
        command[22:25]=(Rotation.from_quat(command[[4,5,6,3]])*
            Rotation.from_quat(np.asarray(previous)[[4,5,6,3]]).inv()).as_rotvec()/self.scene.dt
        return command

    def command(self,frame,baseline,*,previous_command=None):
        measured=self.scene.snapshot()
        tpf=physics_ticks_per_frame(self.scene.manifest['fps'],self.scene.dt)
        self.contact_loss_run=(self.contact_loss_run+1 if measured['contact']['mode']!='grasped' else 0)
        # A single solver step without opposition must not restart a target
        # handoff. This is controller memory only: physical contact/retention
        # checks still use the native measured state, with unchanged gates.
        if self.anchor.entry_contact=='free' and self.contact_loss_run>=tpf and self.held is not None:
            from multipath.validation.physics import retention_errors
            error=retention_errors(measured,self.held)
            # Opposition is an observation, not a controller reset trigger.
            # A brief loss while the fingers redistribute contact must not
            # return a lifting hand to the stationary acquisition target.
            # Discard the proposal only after actual hand/tool separation;
            # native contact and final retention acceptance stay unchanged.
            if error['position_m']>.05 or error['rotation_rad']>.35:
                self.held=None;self.drive_held=None;self.handoff=None;self.preload_bounds=None
        self.contact_run=self.contact_run+1 if measured['contact']['mode']=='grasped' else 0
        if self.held is None and self.contact_run>=tpf:
            self.held=np.asarray(measured['hand_object_transform'])
            previous=self.scene.command_pose if previous_command is None else previous_command[:7]
            self.drive_held=grasp_drive_transform(measured,previous)
            self.preload_bounds=grasp_command_bounds(self.scene,measured,self.anchor)
            self.acquired_frame=measured['time_s']*self.scene.manifest['fps']
            self.handoff=object_relative_handoff(self.scene,self.anchor,self.data,self.acquired_frame,self.drive_held,previous_command)
        if self.held is None:return self.acquisition_command(frame,baseline,measured,previous_command)
        if (self.anchor.entry_contact=='free' and measured['contact']['mode']=='grasped'
                and self.scene.ticks%tpf==0):
            # Keep the physically acquired transform fixed. Replacing it
            # every frame incorporated wrist tracking lag into the next pose
            # target without its velocity, repeatedly cancelling the lift.
            # Finger closure still runs, and physics verifies its retention.
            low,high=grasp_command_bounds(self.scene,measured,self.anchor)
            self.preload_bounds=(np.maximum(self.preload_bounds[0],low),np.minimum(self.preload_bounds[1],high))
        command=object_relative_command(self.scene,self.anchor,self.data,frame,baseline,self.drive_held,self.handoff)
        low,high=self.preload_bounds;step=self.scene.hand_velocity*self.scene.dt
        command[7:19]=np.clip(command[7:19],np.maximum(low,self.scene.hand_command-step),
                              np.minimum(high,self.scene.hand_command+step))
        return command

    def command_with_residual(self,frame,nominal,desired,previous_command):
        """Execute the same acquisition for Replay, MPPI and residual PPO.

        Nominal drive history supplies feedforward. Pose residuals act after
        measured grasp adaptation, so they are neither erased nor converted
        into random velocity impulses.
        """
        base=np.asarray(nominal).copy();base[7:19]=desired[7:19]
        command=self.command(frame,base,previous_command=previous_command)
        next_nominal=command.copy()
        command[:3]+=desired[:3]-base[:3]
        delta=Rotation.from_quat(desired[[4,5,6,3]])*Rotation.from_quat(base[[4,5,6,3]]).inv()
        command[3:7]=(delta*Rotation.from_quat(command[[4,5,6,3]])).as_quat()[[3,0,1,2]]
        return command,next_nominal


def join_core_controls(scene,anchor,commands,start):
    """Join a local retarget seed to the reached drive, inside CORE time."""
    c=np.asarray(commands);dt=scene.dt
    ticks=physics_ticks_per_frame(scene.manifest['fps'],dt)
    context=max(anchor.core_start-anchor.synth_start,anchor.synth_end-anchor.core_end)
    blend=max(2,min(len(c)-1,context*ticks))
    initial=c[0,:7].copy();initial[:3]-=dt*c[0,19:22]
    initial[3:]=(Rotation.from_rotvec(-dt*c[0,22:25])*Rotation.from_quat(initial[[4,5,6,3]])).as_quat()[[3,0,1,2]]
    pose=np.vstack((initial,c[:,:7]));nominal_twist=controls_from_poses(pose,np.vstack((c[0,7:19],c[:,7:19])),dt)[:,19:25]
    phase=np.clip(np.arange(len(pose))/blend,0,1)
    weight=1-phase**2*(3-2*phase);previous=np.asarray(start['command_wrist']);twist=np.asarray(start['command_twist'])
    pose[:,:3]=_entry_curve(pose[:,:3],previous[:3],twist[:3],dt,blend)
    delta=Rotation.from_quat(previous[[4,5,6,3]])*Rotation.from_quat(pose[0,[4,5,6,3]]).inv()
    rotation=Rotation.from_rotvec(weight[:,None]*delta.as_rotvec())*Rotation.from_quat(pose[:,[4,5,6,3]])
    desired=Rotation.from_rotvec(twist[3:]*dt)*Rotation.from_quat(previous[[4,5,6,3]])
    correction=desired*rotation[1].inv();tangent=phase-2*phase**2+phase**3
    rotation=Rotation.from_rotvec(tangent[:,None]/tangent[1]*correction.as_rotvec())*rotation
    pose[:,3:]=rotation.as_quat()[:,[3,0,1,2]]
    hand=np.vstack((c[0,7:19],c[:,7:19]))
    hand=_entry_curve(hand,hand_command_boundary(start),np.zeros(12),dt,blend)
    joined=controls_from_poses(pose,hand,dt)
    # Optimized pose residuals deliberately keep nominal velocity feedforward.
    # Add only the joining correction's derivative; recomputing all velocities
    # silently turned local MPPI pose residuals into large velocity commands.
    joined[:,19:25]+=c[:,19:25]-nominal_twist
    return project_hand_commands(scene,joined,start,anchor)


def free_grasp_registration(anchor,data,start):
    """Register a proposal to the reached tool; never move the object state."""
    from multipath.execution.mounted_scene import pose_matrix
    if (anchor.entry_contact,anchor.exit_contact)!=('free','grasped'):return np.eye(4)
    source=data['tool_pose'][anchor.core_start-anchor.synth_start]
    return pose_matrix(start['objects']['tool']['pose'])@np.linalg.inv(source)


def register_grasp_controls(anchor,data,start,commands):
    from multipath.execution.mounted_scene import pose_matrix
    transform=free_grasp_registration(anchor,data,start)
    c=np.asarray(commands).copy();matrices=np.asarray([transform@pose_matrix(p) for p in c[:,:7]])
    c[:,:3]=matrices[:,:3,3]
    c[:,3:7]=Rotation.from_matrix(matrices[:,:3,:3]).as_quat()[:,[3,0,1,2]]
    c[:,19:22]=c[:,19:22]@transform[:3,:3].T;c[:,22:25]=c[:,22:25]@transform[:3,:3].T
    return c


def source_retarget_core_controls(scene,anchor,data,start):
    """Local retarget for acquisition/release, calibrated at a reached grasp.

    A release must preserve source hand withdrawal as well as finger actions;
    following the object forever would prevent the hand from disengaging.
    """
    from multipath.execution.mounted_scene import pose_matrix
    tpf=physics_ticks_per_frame(scene.manifest['fps'],scene.dt)
    sample=np.arange((anchor.core_end-anchor.core_start)*tpf+1)/tpf+anchor.core_start-anchor.synth_start
    ref=np.asarray(data['wrist_hand_q']);ref=np.vstack((ref,ref[-1]));index=np.arange(len(ref))
    pose=np.c_[np.stack([np.interp(sample,index,ref[:,j]) for j in range(3)],axis=1),
        Slerp(index,Rotation.from_quat(ref[:,[4,5,6,3]]))(sample).as_quat()[:,[3,0,1,2]]]
    hand=np.stack([np.interp(sample,index,ref[:,j]) for j in range(7,19)],axis=1)
    if anchor.entry_contact=='grasped':
        target=data['tool_pose'][anchor.core_start-anchor.synth_start]@np.linalg.inv(grasp_drive_transform(start))
        offset=np.linalg.inv(pose_matrix(pose[0]))@target
        matrices=np.asarray([pose_matrix(p)@offset for p in pose])
        pose=np.c_[matrices[:,:3,3],Rotation.from_matrix(matrices[:,:3,:3]).as_quat()[:,[3,0,1,2]]]
    controls=controls_from_poses(pose,hand,scene.dt)
    if anchor.entry_contact=='free':controls=register_grasp_controls(anchor,data,start,controls)
    return join_core_controls(scene,anchor,controls,start)


def quiet_release_exit(scene,anchor,commands,terminal_offset=None,*,preserve_motion=False):
    """Reach the local withdrawal position with a quiet free-hand boundary."""
    if anchor.exit_contact!='free':return np.asarray(commands).copy()
    c=np.asarray(commands).copy();tpf=physics_ticks_per_frame(scene.manifest['fps'],scene.dt)
    context=max(anchor.core_start-anchor.synth_start,anchor.synth_end-anchor.core_end)
    count=min(len(c)-1,max(tpf,context*tpf));begin=len(c)-count
    previous=c[begin-1];duration=count*scene.dt;s=np.arange(count+1)/count
    if preserve_motion:
        if terminal_offset is None:raise ValueError('Geometry-only refinement requires a measured offset')
        # A geometric overlap does not invalidate the previously executed
        # orientation/withdrawal controller. Add only the measured translation
        # correction; preserve wrist rotation, fingers and nominal angular twist.
        shift=(10*s**3-15*s**4+6*s**5)[:,None]*np.asarray(terminal_offset)
        c[begin:,:3]+=shift[1:]
        c[begin:,19:22]+=np.diff(shift,axis=0)/scene.dt
        c[-1,19:22]=commands[-1,19:22]
        return c
    # Braking must retain the retargeted withdrawal endpoint. Integrating a
    # decaying velocity alone truncated the retreat and left released fingers
    # inside the next planner's obstacle. This pose is a local command target;
    # no simulator state or source time is changed.
    end=c[-1,:7].copy()
    if terminal_offset is not None:end[:3]+=np.asarray(terminal_offset)
    weight=10*s**3-15*s**4+6*s**5
    tangent=s-6*s**3+8*s**4-3*s**5
    xyz=previous[:3]+weight[:,None]*(end[:3]-previous[:3])+duration*tangent[:,None]*previous[19:22]
    # Released wrist orientation is not a task boundary condition. Brake its
    # existing angular motion rather than forcing the human terminal spin.
    coast=duration*(s-s**3+.5*s**4)
    r=Rotation.from_rotvec(coast[:,None]*previous[22:25])*Rotation.from_quat(previous[[4,5,6,3]])
    poses=np.c_[xyz,r.as_quat()[:,[3,0,1,2]]]
    replacement=controls_from_poses(poses,np.vstack((previous[7:19],c[begin:,7:19])),scene.dt)
    replacement[-1,19:25]=0.
    c[begin:,:7]=replacement[:,:7];c[begin:,19:25]=replacement[:,19:25]
    return c
