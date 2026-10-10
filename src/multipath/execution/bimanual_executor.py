"""Continue a joint oracle candidate on the existing uninterrupted PhysX clock.

Unvalidated local cores are proposals. A complete candidate still needs the
separate padded/core artifact gates before it becomes an accepted V0 rollout.
"""
import json
from dataclasses import replace
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def hand_anchor(schedule,anchor,side):
    if side=='right':return anchor
    contact=schedule.metadata['hand_contact_schedule'][anchor.id][side]
    return replace(anchor,entry_contact=contact['entry'],exit_contact=contact['exit'])


def hand_data(data,side,role):
    if side=='right':return data
    return dict(data,wrist_hand_q=data[side+'_wrist_hand_q'],human_joints=data[side+'_human_joints'],
                tool_pose=data[role+'_pose'])


def continue_oracle(scene,schedule,manifest,proposals,output,*,boundary_feedback_run=None):
    from multipath.planning.floating import FloatingPlanner
    from multipath.planning.bridge import feedback_boundary_target,bridge_precheck,terminal_stop_target
    from multipath.execution.wrench_handoff import segment_wrenches,measured_force_balance,support_hold_commands
    from multipath.anchors.floating import (source_grasp_core_controls,source_retarget_core_controls,
        project_hand_commands,quiet_release_exit,validate_commands,acquisition_preload)
    from multipath.anchors.mpc import WindowEvaluation
    from multipath.validation.physics import check_retention,check_relaxed_tool_target_contact,core_pose_errors,object_tracking_score
    from multipath.planning.object_geometry import supported_object_targets
    from multipath.validation.task import evaluate_task
    output=Path(output);hands={'right':scene,**scene.supporting_hands};tpf=scene.tpf
    start_frame=round(scene.ticks/tpf);resets=scene.restore_count
    local={a.id:dict(np.load(Path(proposals)/manifest['anchors'][a.id],allow_pickle=False))
           for a in schedule.anchors.values()}
    report=dict(source='oracle',source_provenance=manifest['provenance'],source_fps=schedule.fps,
        source_duration_s=schedule.num_frames/schedule.fps,continuation_start_frame=start_frame,
        full_pipeline_success=False,task_success=False,continuous_execution_success=False,
        padded_artifacts_complete=False,vlm_used=False,complete_robot_reference_required=False,
        relaxed_reference_tracking=False,source_time_stretch=False,segments=[],anchors=[],
        initial_preparation_uses_synthesis_context=True)
    report['retention_sampling']=dict(relative_pose_and_native_touch_hz=1/scene.dt,
        thumb_and_non_thumb_opposition_hz=schedule.fps,native_dropouts_recorded=True)
    feedback=None
    if boundary_feedback_run is not None:
        from multipath.anchors.boundary_feedback import same_source_seed
        folder=Path(boundary_feedback_run)
        prior=json.loads((folder/'continuous_result.json').read_text())
        if not same_source_seed(prior['source_provenance'],manifest['provenance']):
            raise ValueError('Joint bridge feedback belongs to a different source')
        trace=json.loads((folder/'exported_core'/'actual_trace.json').read_text())
        feedback=(folder,prior,{round(s['source_frame']):s for s in trace})
    try:
        for wi,window in enumerate(schedule.windows):
            if window['end']<=start_frame:continue
            if window['start']<start_frame:raise ValueError('Continuation is not an oracle boundary')
            if scene.ticks!=window['start']*tpf:raise RuntimeError('Joint source clock mismatch')
            info=dict(window,budget_s=(window['end']-window['start'])/schedule.fps,success=False,hands={})
            report['segments'].append(info)
            print('BIMANUAL_SEGMENT_BEGIN',window,flush=True)
            starts={side:hand.snapshot() for side,hand in hands.items()}
            commands={};drives={};goals={};planners={};checks={};held={};preload={}
            if window['mode']=='RELAXED':
                later=next((w for w in schedule.windows[wi+1:] if w['mode']=='ANCHOR'),None)
                for side,hand in hands.items():
                    start=starts[side];carry=start['contact']['mode']=='grasped'
                    if carry:held[side]=np.asarray(start['hand_object_transform']);check_retention(start,held[side])
                    if later:
                        anchor=hand_anchor(schedule,schedule.anchors[later['anchor']],side)
                        if start['contact']['mode']!=anchor.entry_contact:
                            raise RuntimeError(f'{side}: RELAXED cannot change the next interaction contact mode')
                        role='tool' if side=='right' else hand.object_role
                        data=hand_data(local[anchor.id],side,role);index=anchor.core_start-anchor.synth_start
                        target=data['tool_pose'][index]
                        if not carry:raise RuntimeError('An unvalidated free entry requires local acquisition first')
                        lo=max(0,index-1);hi=min(len(data['tool_pose'])-1,index+1)
                        boundaries=np.asarray(data['tool_pose'])[[lo,index,hi]].copy()
                        support=schedule.metadata.get('object_support',{}).get(role)
                        support_fit=None
                        if support:
                            if support!='table':raise RuntimeError('Unsupported declared support geometry')
                            boundaries,support_fit=supported_object_targets(hand,boundaries,start)
                        wrists=boundaries@np.linalg.inv(held[side])
                        pose=np.r_[wrists[1,:3,3],Rotation.from_matrix(wrists[1,:3,:3]).as_quat()[[3,0,1,2]]]
                        twist=np.r_[(wrists[2,:3,3]-wrists[0,:3,3])*schedule.fps/(hi-lo),
                            (Rotation.from_matrix(wrists[2,:3,:3])*Rotation.from_matrix(wrists[0,:3,:3]).inv()).as_rotvec()*schedule.fps/(hi-lo)]
                        goal=dict(wrist_xyz_wxyz=pose.tolist(),wrist_twist=twist.tolist(),
                            interaction_entry_twist_prior=twist.tolist(),hand_q=start['hand_q'],hand_dq=start['hand_dq'],
                            command_hand_q=start['command_hand_q'],contact={'mode':anchor.entry_contact},scene_id=start['scene_id'])
                        goal,fit=feedback_boundary_target(start,goal,start['wrist_xyz_wxyz'],start,boundaries[1],
                            info['budget_s'],scene.dt,scene.cfg['planning'])
                        goal.update(unvalidated_oracle_boundary=True,source_tool_boundary=target.tolist())
                        info['hands'][side]=dict(canonical_boundary_fit=fit,source_entry_physically_verified=False)
                        calibration=(start['wrist_xyz_wxyz'],start)
                        if support_fit:info['hands'][side]['support_boundary_projection']=support_fit
                        if feedback and not support:
                            folder,prior,measured=feedback
                            segment=next((s for s in prior['segments'] if s['start']==window['start']
                                and s['end']==window['end'] and s['mode']=='RELAXED'),None)
                            transition=(segment or {}).get('hands',{}).get(side,{}).get('transition')
                            if transition and not transition['success'] and not prior.get('native_contact_failure'):
                                from multipath.anchors.compatibility import boundary_errors,compatible
                                if not compatible(boundary_errors(scene.snapshot(),measured[window['start']])):
                                    raise ValueError('Joint bridge feedback requires the same physical predecessor')
                                actual=measured[window['end']]
                                if side!='right':
                                    objects={'tool':actual['objects'][role]}
                                    actual=dict(actual,**actual['hands'][side]);actual['objects']=objects
                                previous=np.load(folder/f'segment_{window["start"]}_{window["end"]}.npz',
                                    allow_pickle=False)[side][-1,:7]
                                goal,fit=feedback_boundary_target(start,goal,previous,actual,boundaries[1],
                                    info['budget_s'],scene.dt,scene.cfg['planning'])
                                calibration=(previous,actual)
                                info['hands'][side]['boundary_feedback']=dict(fit,
                                    physical_rollout=str(folder),acceptance_reused=False)
                    else:
                        target=None;goal=terminal_stop_target(start,info['budget_s'],scene.dt,scene.cfg['planning'])
                        info['hands'][side]={}
                    role='tool' if side=='right' else hand.object_role
                    permitted=schedule.metadata.get('allowed_object_environment_contacts',{}).get(role,[])
                    supported=schedule.metadata.get('object_support',{}).get(role) in permitted
                    # A supported stabilizer already inside the next oracle
                    # boundary need not be turned into a rigid airborne load.
                    # Keep its actual drive preload; validate native support
                    # and the original object boundary in real execution.
                    if supported and carry and (target is None or object_tracking_score(core_pose_errors(start,target))['feasible']):
                        controls=support_hold_commands(hand,start,(window['end']-window['start'])*tpf)
                        if not validate_commands(hand,controls,start)['feasible']:
                            raise RuntimeError('Supported constraint controller command infeasible')
                        commands[side]=controls
                        drive=hand.last_support_hold_wrenches
                        drives[side]=segment_wrenches(hand,start,controls) if drive is None else drive
                        goal=dict(wrist_xyz_wxyz=controls[-1,:7].tolist(),contact={'mode':'grasped'})
                        goals[side]=(goal,target)
                        info['hands'][side].update(persistent_constraint='native supported contact stabilization',
                            spatial_motion_required=False,controller_origin='previously_executed_drive',
                            source_time_stretch=False,source_boundary_checked=True,curobo_used=False)
                        info['hands'][side]['validated_support_context_retained']=drive is not None
                        info['hands'][side]['handoff_force_trace']=[measured_force_balance(hand)]
                        continue
                    planner=FloatingPlanner(hand,carry,output/f'planner_{window["start"]}_{side}',
                        allowed_object_contacts=permitted)
                    info['hands'][side]['allowed_object_environment_contacts']=permitted
                    # An interaction boundary can lie on the next surface.
                    # Use the existing full-geometry free-boundary proposal,
                    # then let the fixed-time/source-condition fit and actual
                    # execution decide whether it is a reachable entry.
                    if later:
                        if planner.goal_clearance(goal['wrist_xyz_wxyz'])[0]<1e-4-1e-8:
                            goal,fit=feedback_boundary_target(start,goal,*calibration,boundaries[1],
                                info['budget_s'],scene.dt,scene.cfg['planning'],
                                clearance=lambda pose:planner.goal_clearance(pose)[0],
                                clearance_gradient=planner.path_clearance_gradient)
                            info['hands'][side]['collision_time_source_boundary_fit']=fit
                        info['hands'][side]['boundary_clearance_m']=1e-4
                    check=bridge_precheck(start,goal,info['budget_s'],scene.cfg['planning'])
                    info['hands'][side]['boundary_precheck']=check
                    if not check['feasible']:raise RuntimeError(f'{side}: fixed-time boundary failed {check}')
                    controls,timing=planner.plan(np.asarray(goal['wrist_xyz_wxyz']),
                        (window['end']-window['start'])*tpf,np.asarray(goal['wrist_twist']),goal_state=goal if later else None)
                    if not validate_commands(hand,controls,start)['feasible']:
                        raise RuntimeError(f'{side}: bridge finger command infeasible')
                    supported=schedule.metadata.get('object_support',{}).get(role) in permitted
                    commands[side]=controls;drives[side]=segment_wrenches(hand,start,controls,transport=carry,
                        free_motion=not carry,supported_object=supported)
                    info['hands'][side]['surface_supported_payload']=supported
                    info['hands'][side]['drive_memory_profile']=hand.last_drive_memory_profile
                    if supported:
                        info['hands'][side]['contact_preload_model']='native contact-point normal wrench; residual link force'
                        info['hands'][side]['support_contact_wrench']=hand.last_support_contact_wrench
                        info['hands'][side]['handoff_force_trace']=[measured_force_balance(hand)]
                    goals[side]=(goal,target);planners[side]=planner
                    info['hands'][side]['planning']=timing
            else:
                for side,hand in hands.items():
                    anchor=hand_anchor(schedule,schedule.anchors[window['anchor']],side)
                    data=hand_data(local[anchor.id],side,'tool' if side=='right' else hand.object_role)
                    start=starts[side];stable=anchor.entry_contact==anchor.exit_contact=='grasped'
                    role='tool' if side=='right' else hand.object_role
                    support=schedule.metadata.get('object_support',{}).get(role)
                    command_data=data;support_fit=None
                    if support:
                        if support!='table':raise RuntimeError('Unsupported declared support geometry')
                        poses,support_fit=supported_object_targets(hand,data['tool_pose'],start)
                        command_data=dict(data,tool_pose=poses)
                    source_core=data['tool_pose'][anchor.core_start-anchor.synth_start:anchor.core_end-anchor.synth_start+1]
                    keep_support=bool(support and stable and all(object_tracking_score(core_pose_errors(start,p))['feasible'] for p in source_core))
                    if keep_support:
                        controls=support_hold_commands(hand,start,(anchor.core_end-anchor.core_start)*tpf)
                        fit=dict(curobo_used=False,persistent_constraint='source-conditioned native support',
                            controller_origin='previously_executed_drive',original_core_acceptance_unchanged=True)
                    elif stable:controls,fit=source_grasp_core_controls(hand,anchor,command_data,start)
                    else:controls=source_retarget_core_controls(hand,anchor,command_data,start);fit={'curobo_used':False}
                    controls=quiet_release_exit(hand,anchor,project_hand_commands(hand,controls,start,anchor))
                    validation=validate_commands(hand,controls,start)
                    info['hands'][side]=dict(local_fit=fit,command_precheck=validation)
                    if support_fit:info['hands'][side]['support_command_projection']=support_fit
                    if not validation['feasible']:raise RuntimeError(f'{side}: local core command infeasible')
                    role='tool' if side=='right' else hand.object_role
                    supported=bool(schedule.metadata.get('object_support',{}).get(role))
                    commands[side]=controls
                    retained=hand.last_support_hold_wrenches if keep_support else None
                    drives[side]=(retained if retained is not None else segment_wrenches(hand,start,controls,
                        transport=stable and not keep_support,supported_object=supported))
                    check=WindowEvaluation(hand,anchor,data,start);checks[side]=check
                    if not check.observe(anchor.core_start,controls[0],measured=start):
                        raise RuntimeError(f'{side}: {check.failure}')
                    folder=output/'cores'/anchor.id/side;folder.mkdir(parents=True,exist_ok=True)
                    (folder/'entry.json').write_text(json.dumps(start,indent=2))
            np.savez_compressed(output/f'segment_{window["start"]}_{window["end"]}.npz',
                **commands,**{side+'_wrenches':drive for side,drive in drives.items()})
            contact_start=scene.tool_target_contact_steps
            if hasattr(scene,'relaxed_contact_failure'):del scene.relaxed_contact_failure
            for k in range((window['end']-window['start'])*tpf):
                step={}
                for side,hand in hands.items():
                    hand.wrench_bias=drives[side][k];command=commands[side][k]
                    if window['mode']=='ANCHOR':
                        a=hand_anchor(schedule,schedule.anchors[window['anchor']],side)
                        command,preload[side]=acquisition_preload(hand,a,command,preload.get(side))
                        commands[side][k]=command
                    step[side]=command
                frame=window['start']+(k+1)/tpf
                scene.step_control(step['right'],window['mode'],frame,
                    supporting_commands={side:c for side,c in step.items() if side!='right'})
                if window['mode']=='RELAXED' and k<2*tpf:
                    for side,hand in hands.items():
                        samples=info['hands'][side].get('handoff_force_trace')
                        if samples is not None:samples.append(measured_force_balance(hand))
                if window['mode']=='RELAXED':
                    # A transported tool can move the supported object and
                    # break the other hand's contact in the same step. Record
                    # the native collision first, so boundary repair receives
                    # its actual points/normals instead of the secondary loss.
                    check_relaxed_tool_target_contact(scene,contact_start)
                    for side,hand in hands.items():
                        if side in held:
                            try:check_retention(hand.state(),held[side],require_opposition=(k+1)%tpf==0)
                            except RuntimeError:
                                info['hands'][side]['retention_failure_frame']=frame
                                raise
                        if side not in planners:continue
                        clearance=planners[side].measured_clearance()
                        if min(clearance.values())<0:
                            collisions=planners[side].measured_mesh_collisions()
                            if collisions:
                                info['hands'][side]['collision_failure']=dict(source_frame=frame,meshes=collisions)
                                raise RuntimeError(f'{side}: actual RELAXED mesh collision')
                elif (k+1)%tpf==0:
                    for side,check in checks.items():
                        if not check.observe(frame,step[side]):
                            info['hands'][side]['failure']=check.failure
                            raise RuntimeError(f'{side}: {check.failure}')
            if window['mode']=='RELAXED':
                for side,hand in hands.items():
                    goal,target=goals[side]
                    transition=hand.check_transition(goal['wrist_xyz_wxyz'],goal['contact']['mode'],expected_object=target)
                    info['hands'][side]['transition']=transition
                    if not transition['success']:raise RuntimeError(f'{side}: actual oracle entry not reached')
            else:
                for side,hand in hands.items():
                    folder=output/'cores'/window['anchor']/side
                    (folder/'exit.json').write_text(json.dumps(hand.snapshot(),indent=2))
                    np.savez_compressed(folder/'commands.npz',commands=commands[side],wrenches=drives[side])
                report['anchors'].append(dict(id=window['anchor'],local_rollout_success=True,handoff_verified=True,
                    entry_physically_reachable=True,entry_origin='predecessor_rollout',padded_rollout_success=False))
            if scene.restore_count!=resets:raise RuntimeError('State reset inside joint continuous execution')
            info.update(success=True,actual_end_s=scene.ticks*scene.dt);scene.save()
            (output/'continuous_result.json').write_text(json.dumps(report,indent=2))
        report['continuous_execution_success']=True
        report['task']=evaluate_task(scene.state(),manifest,schedule.metadata['task_contract'],report['anchors'])
        report['task_success']=report['task']['success']
        report['status']='continuous_candidate_complete' if report['task_success'] else 'task_failed'
    except Exception as error:
        import traceback;traceback.print_exc()
        report.update(status='failed',error=f'{type(error).__name__}: {error}',failure_source_frame=scene.ticks/tpf)
        if hasattr(error,'boundary_fit_report'):report['boundary_fit_failure']=error.boundary_fit_report
        if hasattr(scene,'relaxed_contact_failure'):report['native_contact_failure']=scene.relaxed_contact_failure
        if window['mode']=='RELAXED':
            from multipath.validation.physics import retention_errors
            report['retention_at_failure']={side:retention_errors(hands[side].state(),transform)
                for side,transform in held.items()}
        (output/'failed_state.json').write_text(json.dumps(scene.snapshot(),indent=2))
    finally:
        scene.save();report.update(actual_duration_s=scene.ticks*scene.dt,
            state_resets_between_segments=scene.restore_count-resets,actual_trace=str(scene.out/'actual_trace.json'))
        (output/'continuous_result.json').write_text(json.dumps(report,indent=2))
        print('BIMANUAL_CONTINUOUS_RESULT',report['status'],report.get('error'),flush=True)
    return report
