"""Continuous oracle experiment; missing local cores are proposals, not proofs.

No restored states in an initial-state run. An optional real core exit is only
for tail development and can never be reported as a complete task rollout.
"""
import argparse,hashlib,json,os,shutil
from pathlib import Path


def main():
    p=argparse.ArgumentParser()
    for name in ('schedule','proposals','seed-artifacts','floating-config','xhand-urdf','object-cache','output-dir'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--start-core-run',type=Path,help='Passed real predecessor core; development tail only')
    p.add_argument('--initial-bridge-run',type=Path,help='Freeze a passed initial cuRobo segment while refining local Anchors')
    p.add_argument('--boundary-feedback-run',type=Path,help='Actual failed bridge rollout for next-boundary proposal correction')
    p.add_argument('--exit-geometry-feedback-run',type=Path,help='Actual released Anchor exit for geometric boundary resynthesis')
    p.add_argument('--template-artifacts',type=Path,help='Optional physically validated local interaction library; initialization only.')
    p.add_argument('--template-proposals',type=Path,help='Local source observations corresponding to that library.')
    p.add_argument('--template-free-entry-rank',type=int,default=0)
    args=p.parse_args();out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    from multipath.oracle_schedule import OracleSchedule
    schedule=OracleSchedule.load(args.schedule);manifest=json.loads((args.proposals/'manifest.json').read_text())
    if manifest['provenance']['schedule_sha256']!=hashlib.sha256(args.schedule.read_bytes()).hexdigest():
        raise ValueError('Oracle schedule provenance mismatch')
    report=dict(status='initializing',source='oracle',source_fps=schedule.fps,source_duration_s=schedule.num_frames/schedule.fps,
        source_provenance=manifest['provenance'],pbs_job_id=os.environ.get('PBS_JOBID'),full_pipeline_success=False,
        task_success=False,continuous_execution_success=False,vlm_used=False,complete_robot_reference_required=False,
        relaxed_reference_tracking=False,source_time_stretch=False,segments=[],anchors=[],development_only=True)
    import multipath
    shutil.copytree(Path(multipath.__file__).parent,out/'implementation'/'multipath',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copy2(__file__,out/'implementation'/Path(__file__).name)
    from multipath.execution.startup import watch_dependency_imports
    watch_dependency_imports(out)
    scene=None;app=None;resets=0
    try:
        import numpy as np
        from scipy.spatial.transform import Rotation
        from multipath.execution.startup import start_simulation
        from multipath.execution.floating_scene import FloatingScene
        from multipath.planning.floating import FloatingPlanner,sphere_world_clearance
        from multipath.planning.bridge import project_boundary_target,bridge_precheck,terminal_stop_target
        from multipath.execution.mounted_scene import pose_matrix
        from multipath.execution.wrench_handoff import segment_wrenches,local_anchor_wrenches
        from multipath.anchors.floating import (join_core_controls,source_grasp_core_controls,
            source_retarget_core_controls,quiet_release_exit,validate_commands,AcquiredGraspProposal,project_hand_commands,acquisition_preload,
            free_grasp_registration,register_grasp_controls)
        from multipath.anchors.mpc import WindowEvaluation
        from multipath.validation.physics import check_retention,check_relaxed_tool_target_contact,core_pose_errors,object_tracking_score
        from multipath.validation.task import evaluate_task
        first=min(schedule.anchors.values(),key=lambda a:a.core_start)
        local={a.id:dict(np.load(args.proposals/manifest['anchors'][a.id],allow_pickle=False)) for a in schedule.anchors.values()}
        templates={}
        if args.template_artifacts:
            if not args.template_proposals:raise ValueError('Template source observations required')
            from multipath.anchors.template import acquisition_proposals
            templates=acquisition_proposals(args.template_artifacts,args.template_proposals,manifest,schedule,local,FloatingScene.dt,
                planning=json.loads(args.floating_config.read_text())['planning'],free_entry_rank=args.template_free_entry_rank)
            report['interaction_initialization']={key:value['provenance'] for key,value in templates.items()}
        data=local[first.id];initial=data['wrist_hand_q'][0,:7].copy()
        initial[2]+=max(.1,float(np.linalg.norm(np.ptp(data['human_joints'][0],axis=0))))
        initial_hand=(templates[first.id]['entry']['command_hand_q'] if first.id in templates
                      else data['wrist_hand_q'][0,7:])
        if args.initial_bridge_run:
            prior=json.loads((args.initial_bridge_run/'pipeline_result.json').read_text())
            if prior['source_provenance']!=manifest['provenance']:raise ValueError('Initial preparation source mismatch')
            initial=np.asarray(prior['initial_state']['command_wrist'])
            # A refitted canonical pregrasp must also prepare the initial
            # fingers. Reusing the old fingers made their unused geometry
            # collide at the new goal and displaced the free entry again.
        app=start_simulation(out);scene=FloatingScene(args,manifest,out,initial,initial_hand=initial_hand)
        report['initial_hand_origin']=('local_interaction_canonical_free_pregrasp' if first.id in templates else 'local_human_context')
        initial_state=scene.snapshot();start_frame=0;resets=0
        report.update(embodiment=scene.embodiment,physics='PhysX CPU',table_z=float(scene.table['pose'][2]+.02),initial_state=initial_state,
            core_tracking_policy=scene.cfg.get('validation',{}).get('core_tracking_policy','strict'))
        if args.start_core_run:
            prior=json.loads((args.start_core_run/'bridge_result.json').read_text())
            if not prior.get('next_core_success') or prior['source_provenance']!=manifest['provenance']:
                raise ValueError('Tail start must be a passed real predecessor core of this source')
            trace=args.start_core_run/'next_core'/'actual_trace.json';state=json.loads(trace.read_text())[-1]
            state.update(entry_origin='physically_acquired_snapshot',source_provenance=manifest['provenance'],
                entry_provenance={'trace':str(trace),'source_frame':state['source_frame'],'result':str(args.start_core_run/'bridge_result.json')})
            scene.restore_development(state);start_frame=round(state['source_frame']);resets=scene.restore_count
            scene.begin_recording(out,mesh=True);scene.capture('INITIAL',start_frame)
            report.update(start_frame=start_frame,validation_scope='physical_tail_development')
        for wi,w in enumerate(schedule.windows):
            if w['end']<=start_frame:continue
            if w['start']<start_frame:raise ValueError('Tail start is not a segment boundary')
            start=scene.snapshot();budget=(w['end']-w['start'])/schedule.fps
            if not np.isclose(start['time_s'],w['start']/schedule.fps,atol=1e-8,rtol=0):raise RuntimeError('Source clock mismatch')
            info=dict(w,budget_s=budget,success=False);report['segments'].append(info)
            print('CONNECTED_SEGMENT_BEGIN',w,flush=True)
            if w['mode']=='RELAXED':
                carry=w['contact']=='grasped';held=np.asarray(start['hand_object_transform'])
                cached_initial=None
                if w['start']==0 and args.initial_bridge_run:
                    prior=json.loads((args.initial_bridge_run/'pipeline_result.json').read_text())
                    segment=next(s for s in prior['segments'] if s['start']==0)
                    if not segment['success'] or prior['source_provenance']!=manifest['provenance']:
                        raise ValueError('Initial bridge cache was not physically validated for this source')
                    from multipath.anchors.compatibility import boundary_errors,compatible
                    errors=boundary_errors(start,prior['initial_state'])
                    if compatible(errors):cached_initial=segment
                    else:
                        # An optional cache is not a requirement. Different
                        # scene/config provenance needs fresh planning and
                        # physics, never a relaxed compatibility check.
                        info['initial_bridge_cache_rejected']=errors
                if start['contact']['mode']!=w['contact']:raise RuntimeError('Planner cannot change contact mode')
                later=next((v for v in schedule.windows[wi+1:] if v['mode']=='ANCHOR'),None)
                source_target=None
                if later:
                    a=schedule.anchors[later['anchor']];d=local[a.id];index=a.core_start-a.synth_start
                    source_target=d['tool_pose'][index]
                    if carry:
                        wrists=d['tool_pose']@np.linalg.inv(held);lo=max(0,index-1);hi=min(len(wrists)-1,index+1)
                        pose=np.r_[wrists[index,:3,3],Rotation.from_matrix(wrists[index,:3,:3]).as_quat()[[3,0,1,2]]]
                        twist=np.r_[(wrists[hi,:3,3]-wrists[lo,:3,3])*schedule.fps/(hi-lo),
                            (Rotation.from_matrix(wrists[hi,:3,:3])*Rotation.from_matrix(wrists[lo,:3,:3]).inv()).as_rotvec()*schedule.fps/(hi-lo)]
                    else:
                        entry_path=args.seed_artifacts/a.id/'entry.json'
                        entry=json.loads(entry_path.read_text()) if entry_path.exists() else None
                        if entry is None and a.id in templates:entry=templates[a.id]['entry']
                        pose=np.asarray(entry['wrist_xyz_wxyz']) if entry else d['wrist_hand_q'][index,:7]
                        twist=np.zeros(6)
                        if entry is None or not entry_path.exists():
                            matrix=free_grasp_registration(a,d,start)@pose_matrix(pose)
                            pose=np.r_[matrix[:3,3],Rotation.from_matrix(matrix[:3,:3]).as_quat()[[3,0,1,2]]]
                        if entry_path.exists() and entry.get('physically_acquired_state'):
                            # The actual reachable wrist state and the drive
                            # which produced it are different boundary data.
                            # Replanning to the measured pose as a drive changes
                            # the next grasp's controller handoff unnecessarily.
                            pose=np.asarray(entry['command_wrist'])
                            twist=np.asarray(entry['command_twist'])
                            info['next_anchor_target_origin']='physically_validated_entry_drive'
                    goal=dict(wrist_xyz_wxyz=pose.tolist(),wrist_twist=twist.tolist(),hand_q=start['hand_q'],hand_dq=start['hand_dq'],
                        command_hand_q=start['command_hand_q'],contact={'mode':w['contact']},scene_id=start['scene_id'])
                    if not carry and entry:
                        # A free bridge may prepare the next Anchor's fingers;
                        # acquired grasps retain their existing preload above.
                        for key in ('hand_q','hand_dq','command_hand_q'):goal[key]=entry[key]
                    elif not carry:
                        # A raw local Anchor still supplies its core-entry
                        # pregrasp posture. Keeping synth_start's fingers here
                        # postpones their preparation until after CORE begins.
                        hand=np.clip(d['wrist_hand_q'][index,7:],scene.hand_limits[:,0],scene.hand_limits[:,1])
                        goal.update(hand_q=hand.tolist(),hand_dq=np.zeros_like(hand).tolist(),command_hand_q=hand.tolist())
                    if carry:
                        from multipath.planning.bridge import feedback_boundary_target
                        # Oracle pose/contact/time are fixed. Human wrist
                        # velocity is not a required robot entry velocity.
                        # Identity tracking calibration uses only the real
                        # predecessor, never a fabricated future state.
                        goal['interaction_entry_twist_prior']=twist.tolist()
                        goal,boundary_fit=feedback_boundary_target(start,goal,start['wrist_xyz_wxyz'],
                            start,source_target,budget,scene.dt,scene.cfg['planning'])
                        goal['state_origin']='unvalidated_source_conditioned_reachable_boundary_proposal'
                        info['canonical_boundary_fit']=dict(boundary_fit,
                            human_velocity_required=False,physical_acceptance=False)
                        if not (args.seed_artifacts/a.id/'entry.json').exists():
                            goal.update(unvalidated_oracle_boundary=True,source_tool_boundary=source_target.tolist())
                    else:
                        goal=project_boundary_target(start,goal,budget,scene.cfg['planning'],dt=scene.dt)
                else:
                    # No next interaction imposes a return-to-initial condition.
                    # Stop the released hand inside the remaining source time.
                    goal=terminal_stop_target(start,budget,scene.dt,scene.cfg['planning'])
                if carry and args.boundary_feedback_run:
                    prior=json.loads((args.boundary_feedback_run/'pipeline_result.json').read_text())
                    if prior['source_provenance']!=manifest['provenance']:raise ValueError('Boundary feedback source mismatch')
                    measured={round(x['source_frame']):x for x in json.loads((args.boundary_feedback_run/'actual_trace.json').read_text())}
                    prior_segment=next((s for s in prior['segments'] if s['start']==w['start'] and s.get('transition')),None)
                    if prior_segment and not prior_segment['transition']['success']:
                        from multipath.anchors.compatibility import boundary_errors,compatible
                        from multipath.planning.bridge import feedback_boundary_target
                        if not compatible(boundary_errors(start,measured[w['start']])):
                            raise ValueError('Boundary feedback requires the same physical predecessor')
                        previous=np.load(args.boundary_feedback_run/f'relaxed_{w["start"]}_{w["end"]}.npz')['commands'][-1]
                        goal,feedback=feedback_boundary_target(start,goal,previous[:7],measured[w['end']],source_target,budget,scene.dt,scene.cfg['planning'])
                        info['boundary_feedback']=dict(feedback,physical_rollout=str(args.boundary_feedback_run))
                if not carry and later and args.boundary_feedback_run:
                    prior=json.loads((args.boundary_feedback_run/'pipeline_result.json').read_text())
                    if prior['source_provenance']!=manifest['provenance']:raise ValueError('Free boundary feedback source mismatch')
                    segment=next((s for s in prior['segments'] if s['start']==w['start'] and not s['success']),None)
                    if segment:
                        from multipath.planning.floating import pose_coordinates,coordinates_pose
                        fraction=segment.get('free_entry_fraction',1.)*.9
                        q0=pose_coordinates(start['wrist_xyz_wxyz']);q1=pose_coordinates(goal['wrist_xyz_wxyz'],q0)
                        goal['wrist_xyz_wxyz']=coordinates_pose(q0+fraction*(q1-q0)).tolist()
                        info['free_entry_fraction']=fraction
                planner=FloatingPlanner(scene,carry,out/f'planner_{w["start"]}',goal_hand=goal.get('command_hand_q') if not carry else None)
                if cached_initial:
                    cached_commands=np.load(args.initial_bridge_run/f'relaxed_{w["start"]}_{w["end"]}.npz')['commands']
                    if (not np.allclose(pose_matrix(cached_commands[-1,:7]),pose_matrix(goal['wrist_xyz_wxyz']),atol=1e-5,rtol=0)
                            or not np.allclose(cached_commands[-1,7:19],goal['command_hand_q'],atol=1e-5,rtol=0)):
                        info['initial_bridge_cache_goal_rejected']='canonical free entry/drive changed; reacquire with cuRobo'
                        cached_initial=None
                # Conservative task-boundary projection is a command proposal;
                # the actual object condition must still pass after execution.
                if not cached_initial:
                    clearance=1e-4
                    for _ in range(20):
                        worst=planner.goal_clearance(goal['wrist_xyz_wxyz'])
                        if worst[0]>=clearance-1e-8:break
                        pose=np.asarray(goal['wrist_xyz_wxyz']);pose[:3]+=(clearance-worst[0])*worst[1];goal['wrist_xyz_wxyz']=pose.tolist()
                    else:raise RuntimeError('No collision-free oracle boundary proposal')
                    info['boundary_clearance_m']=clearance
                check=bridge_precheck(start,goal,budget,scene.cfg['planning']);info['boundary_precheck']=check
                if not check['feasible']:raise RuntimeError(f'Fixed source-time boundary rejected: {check["reasons"]}')
                if cached_initial:
                    commands=np.load(args.initial_bridge_run/f'relaxed_{w["start"]}_{w["end"]}.npz')['commands']
                    if commands.shape!=(round(budget/scene.dt),25) or not validate_commands(scene,commands,start)['feasible']:
                        raise ValueError('Initial bridge cache violates source time or hand limits')
                    timing=cached_initial['planning'];info['frozen_initial_curobo']=str(args.initial_bridge_run)
                else:
                    rejections=[]
                    for attempt in range(5):
                        try:
                            commands,timing=planner.plan(np.asarray(goal['wrist_xyz_wxyz']),round(budget/scene.dt),np.asarray(goal['wrist_twist']),goal_state=goal)
                            break
                        except RuntimeError as error:
                            if (carry or not later or not any(text in str(error) for text in
                                ['No collision-free trajectory in source time','Floating cuRobo failed:'])):raise
                            rejections.append(str(error))
                            if attempt==4:raise
                            # Free entry has no validated hand pose yet. Choose
                            # a closer free boundary; object/contact conditions
                            # and the original source-time budget stay fixed.
                            from multipath.planning.floating import pose_coordinates,coordinates_pose
                            q0=pose_coordinates(start['wrist_xyz_wxyz']);q1=pose_coordinates(goal['wrist_xyz_wxyz'],q0)
                            goal['wrist_xyz_wxyz']=coordinates_pose(q0+.9*(q1-q0)).tolist()
                            info['free_entry_fraction']=info.get('free_entry_fraction',1.)*.9
                            for _ in range(20):
                                worst=planner.goal_clearance(goal['wrist_xyz_wxyz'])
                                if worst[0]>=1e-4-1e-8:break
                                goal['wrist_xyz_wxyz'][:3]=(np.asarray(goal['wrist_xyz_wxyz'][:3])+(1e-4-worst[0])*worst[1]).tolist()
                            check=bridge_precheck(start,goal,budget,scene.cfg['planning'])
                            info['boundary_precheck']=check
                            if not check['feasible']:raise
                    if rejections:info['free_entry_resynthesis']=dict(rejected_paths=rejections,
                        actual_object_boundary_unchanged=True,source_time_stretch=False)
                if not cached_initial and not carry:timing['free_motion_dynamics_feedforward']=True
                free_model=timing.get('free_motion_dynamics_feedforward',False)
                info['planning']=timing
                if timing.get('canonical_boundary_jointly_fitted'):
                    info['boundary_precheck']=bridge_precheck(start,goal,budget,scene.cfg['planning'])
                    if not info['boundary_precheck']['feasible']:raise RuntimeError('Jointly fitted canonical boundary violates source-time dynamics')
                wrenches=segment_wrenches(scene,start,commands,transport=carry,free_motion=free_model) if carry or w['start']>0 or free_model else np.zeros((len(commands),6))
                np.savez_compressed(out/f'relaxed_{w["start"]}_{w["end"]}.npz',commands=commands)
                contact_steps_at_entry=scene.tool_target_contact_steps
                for k,c in enumerate(commands):
                    scene.wrench_bias=wrenches[k];scene.step_control(c,'RELAXED',w['start']+(k+1)*scene.dt*schedule.fps)
                    if carry:
                        check_retention(scene.state(),held)
                        check_relaxed_tool_target_contact(scene,contact_steps_at_entry)
                    clearance=planner.measured_clearance()
                    if min(clearance.values())<0:
                        collisions=planner.measured_mesh_collisions()
                        if collisions:
                            info['collision_failure']=dict(source_frame=w['start']+(k+1)*scene.dt*schedule.fps,
                                measured_meshes=collisions,clearance_m=clearance)
                            raise RuntimeError('Actual RELAXED mesh collision')
                transition=scene.check_transition(np.asarray(goal['wrist_xyz_wxyz']),w['contact'],expected_object=source_target)
                info['transition']=transition
                if not transition['success']:raise RuntimeError('Actual oracle boundary failed')
            else:
                a=schedule.anchors[w['anchor']];d=local[a.id];reused=False;native_wrenches=None
                stable_carry=a.entry_contact==a.exit_contact=='grasped'
                template_used=False
                folder=out/'cores'/a.id;folder.mkdir(parents=True,exist_ok=True)
                (folder/'entry.json').write_text(json.dumps(start,indent=2))
                seed_path=args.seed_artifacts/a.id/'commands.npz'
                if seed_path.exists():
                    from multipath.anchors.artifact import load_validated_anchor
                    from multipath.anchors.compatibility import boundary_errors,compatible
                    from multipath.anchors.floating import rebase_grasp_commands
                    seed=load_validated_anchor(args.seed_artifacts,a,provenance=manifest['provenance'],fps=schedule.fps,dt=scene.dt)
                    handoff=seed['report']['validation'].get('controller_handoff')
                    stable_carry=bool(isinstance(handoff,dict) and handoff.get('carried_dynamics_feedforward'))
                    reused=compatible(boundary_errors(start,seed['entry'])) and all(np.allclose(start[k],seed['entry'][k],rtol=0,atol=1e-8) for k in ['command_wrist','command_twist','command_hand_q'])
                    drive_matches=all(np.allclose(start[k],seed['entry'][k],rtol=0,atol=1e-6)
                        for k in ['command_wrist','command_twist','command_hand_q'])
                    commands=seed['commands'].copy()
                    preserve_free_drive=a.entry_contact=='free' and drive_matches
                    if not reused and not preserve_free_drive:
                        if a.entry_contact=='grasped':
                            commands=rebase_grasp_commands(commands,seed['entry'],start,scene.dt)
                        commands=join_core_controls(scene,a,commands,start)
                    info['canonical_core_reused']=reused
                    info['seed_drive_boundary_matches']=drive_matches
                    info['seed_commands_preserved']=bool(reused or preserve_free_drive)
                    if reused or preserve_free_drive:native_wrenches=seed.get('controller_wrenches')
                    info['validated_local_seed']=str(seed_path.parent)
                elif a.entry_contact==a.exit_contact=='grasped':
                    commands,handoff=source_grasp_core_controls(scene,a,d,start)
                else:
                    if a.id in templates:
                        template=templates[a.id]
                        drive_matches=(np.allclose(pose_matrix(start['command_wrist']),pose_matrix(template['previous_drive_wrist']),atol=1e-5,rtol=0)
                            and np.allclose(start['command_hand_q'],template['previous_drive_hand'],atol=1e-5,rtol=0))
                        # The standard wrench handoff already preserves the
                        # measured response across feedforward changes. A tiny
                        # terminal twist difference need not warp learned poses.
                        local_template=register_grasp_controls(a,d,start,template['commands'])
                        commands=(local_template if drive_matches else join_core_controls(scene,a,local_template,start))
                        info['template_drive_boundary_matches']=drive_matches
                        template_used=True;info['interaction_initialization']=templates[a.id]['provenance']
                    else:commands=source_retarget_core_controls(scene,a,d,start)
                terminal_offset=None
                if a.exit_contact=='free' and args.exit_geometry_feedback_run:
                    prior=json.loads((args.exit_geometry_feedback_run/'pipeline_result.json').read_text())
                    if prior['source_provenance']!=manifest['provenance']:raise ValueError('Exit geometry feedback source mismatch')
                    state=json.loads((args.exit_geometry_feedback_run/'cores'/a.id/'exit.json').read_text())
                    geometry=json.loads((args.exit_geometry_feedback_run/f'planner_{a.core_end}'/'collision_geometry.json').read_text())
                    from multipath.planning.bridge import free_exit_clearance_offset
                    terminal_offset,feedback=free_exit_clearance_offset(scene,state,geometry)
                    previous_segment=next(s for s in prior['segments'] if s.get('anchor')==a.id)
                    previous_offset=np.asarray(previous_segment.get('exit_geometry_feedback',{}).get('offset_m',np.zeros(3)))
                    feedback['incremental_offset_m']=terminal_offset.tolist()
                    terminal_offset+=previous_offset
                    feedback['offset_m']=terminal_offset.tolist()
                    feedback['previous_offset_m']=previous_offset.tolist()
                    info['exit_geometry_feedback']=dict(feedback,physical_rollout=str(args.exit_geometry_feedback_run))
                if native_wrenches is None or terminal_offset is not None:
                    commands=project_hand_commands(scene,commands,start,a)
                    commands=quiet_release_exit(scene,a,commands,terminal_offset)
                    # A reshaped proposal requires new physical synthesis;
                    # its controls no longer belong to the saved program.
                    native_wrenches=None
                np.savez_compressed(folder/'commands.npz',commands=commands)
                check=validate_commands(scene,commands,start);info['command_precheck']=check
                if not check['feasible']:raise RuntimeError('Local core command rejected')
                objective=WindowEvaluation(scene,a,d,start)
                if not objective.observe(a.core_start,commands[0]):raise RuntimeError(str(objective.failure))
                info['carried_dynamics_feedforward']=stable_carry
                wrenches=(native_wrenches if native_wrenches is not None else
                          local_anchor_wrenches(scene,start,commands,a))
                info['native_controller_program_replayed']=native_wrenches is not None
                acquisition=AcquiredGraspProposal(scene,a,d) if native_wrenches is None and not reused and a.entry_contact=='free' and a.exit_contact=='grasped' else None
                preload=None
                for k,c in enumerate(commands):
                    scene.wrench_bias=wrenches[k];frame=a.core_start+(k+1)*scene.dt*schedule.fps
                    if acquisition is not None:
                        c=acquisition.command(frame,c)
                    if native_wrenches is None:
                        c,preload=acquisition_preload(scene,a,c,preload)
                    commands[k]=c
                    scene.step_control(c,'ANCHOR',frame)
                    if (k+1)%scene.tpf==0 and not objective.observe(frame,c):
                        np.savez_compressed(folder/'commands.npz',commands=commands)
                        info['failure']=objective.failure;raise RuntimeError(str(objective.failure))
                if acquisition is not None:
                    info['local_refinement']=dict(method='measured_grasp_object_relative_MPC_proposal',
                        grasp_acquired_frame=acquisition.acquired_frame,target_handoff=acquisition.handoff)
                exit_state=scene.snapshot();(folder/'exit.json').write_text(json.dumps(exit_state,indent=2))
                info['core_tracking_policy']=objective.core_tracking_policy
                info['core_tracking_diagnostics']=objective.tracking_failures
                info['native_preload_retained_during_synthesis']=preload is not None
                np.savez_compressed(folder/'commands.npz',commands=commands)
                report['anchors'].append(dict(id=a.id,core_interval=[a.core_start,a.core_end],synthesis_interval=[a.synth_start,a.synth_end],
                    local_rollout_success=True,handoff_verified=True,entry_physically_reachable=True,entry_origin=start['entry_origin']))
            if scene.restore_count!=resets:raise RuntimeError('State reset between segments')
            info.update(success=True,actual_end_s=scene.ticks*scene.dt)
            (out/'progress.json').write_text(json.dumps(report,indent=2));scene.save()
        report['continuous_execution_success']=True
        report['task']=evaluate_task(scene.state(),manifest,schedule.metadata['task_contract'],report['anchors'])
        report['task_success']=report['task']['success'];report['status']='continuous_candidate_valid' if report['task_success'] else 'task_failed'
        # A passed candidate earns core evidence. Padded local contexts and a
        # fresh artifact-consuming rollout are still explicit remaining gates.
        report['padded_artifacts_complete']=False
    except Exception as error:
        import traceback;traceback.print_exc();report.update(status='failed',error=f'{type(error).__name__}: {error}')
        if scene:(out/'failed_state.json').write_text(json.dumps(scene.snapshot(),indent=2))
    finally:
        if scene:
            scene.save();report.update(actual_duration_s=scene.ticks*scene.dt,state_resets_between_segments=scene.restore_count-resets)
        (out/'pipeline_result.json').write_text(json.dumps(report,indent=2));print('CONNECTED_RESULT',report['status'],report.get('error'),flush=True)
        if app:app.close()


if __name__=='__main__':main()
