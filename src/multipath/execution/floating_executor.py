"""Execute validated floating Anchors and cuRobo bridges in one PhysX world."""
import json
from pathlib import Path
import numpy as np
from multipath.anchors.artifact import load_validated_anchor
from multipath.anchors.floating import validate_commands
from multipath.anchors.mpc import WindowEvaluation
from multipath.planning.floating import FloatingPlanner,pose_coordinates
from multipath.planning.timing import physics_ticks_per_frame
from multipath.validation.physics import check_retention
from multipath.validation.task import evaluate_task


def measured_drive_matches(start,entry):
    if start['scene_id']!=entry['scene_id'] or start['contact']['mode']!=entry['contact']['mode']:return False
    for key in ['wrist_xyz_wxyz','wrist_twist','hand_q','hand_dq',
                'command_wrist','command_twist','command_hand_q','controller_wrench_bias']:
        if not np.allclose(start[key],entry[key],rtol=0,atol=1e-6):return False
    return all(np.allclose(state[k],entry['objects'][role][k],rtol=0,atol=1e-6)
               for role,state in start['objects'].items() for k in ['pose','velocity'])


def execute(scene,schedule,manifest,proposal_dir,report):
    tpf=physics_ticks_per_frame(schedule.fps,scene.dt)
    before=getattr(scene.args,'stop_before_anchor',None)
    through=getattr(scene.args,'through_anchor',None)
    required=[a for a in schedule.anchors.values()
              if (not before or a.core_start<schedule.anchors[before].core_start)
              and (not through or a.core_start<=schedule.anchors[through].core_start)]
    canonical={a.id:load_validated_anchor(scene.args.validated_anchors,a,provenance=manifest['provenance'],
                                         fps=schedule.fps,dt=scene.dt) for a in required}
    if any(v['entry'].get('embodiment')!=scene.embodiment for v in canonical.values()):
        raise ValueError('Canonical Anchor embodiment mismatch')
    initial=scene.state();held=None
    cache=getattr(scene.args,'planned_bridges',None)
    if cache:
        prior=json.loads((cache/'pipeline_result.json').read_text())
        if (not (prior.get('continuous_execution_success') or prior.get('full_pipeline_success')) or not prior.get('task_success')
                or prior.get('state_resets_between_segments')!=0
                or prior['source_provenance']!=manifest['provenance'] or prior.get('start_frame',0)!=0):
            raise ValueError('Bridge cache is not a passed initial-state cuRobo candidate')
    bank_root=getattr(scene.args,'bridge_bank',None);bank=None
    solution=getattr(scene.args,'bridge_solution',0)
    diverse=bool(bank_root and solution>0)
    if bank_root:
        bank=json.loads((bank_root/'bridge_bank.json').read_text())
        if bank['source_provenance']!=manifest['provenance']:raise ValueError('Bridge bank source mismatch')
    previous_runs=[]
    for directory in getattr(scene.args,'exclude_actual_run',[]):
        prior=json.loads((directory/'pipeline_result.json').read_text())
        if (not prior.get('full_pipeline_success') or not prior.get('diverse_solution_success') or
            prior.get('state_resets_between_segments')!=0 or prior['source_provenance']!=manifest['provenance'] or
            not np.isclose(prior['actual_duration_s'],schedule.num_frames/schedule.fps,atol=1e-8,rtol=0)):
            raise ValueError('Serial diversity requires a successful continuous predecessor rollout')
        previous_runs.append(directory)
    prefix_root=getattr(scene.args,'executed_prefix_run',None);passed_prefix={};prefix_trace=[]
    if prefix_root:
        prefix_report=json.loads((prefix_root/'pipeline_result.json').read_text())
        if (prefix_report['source_provenance']!=manifest['provenance'] or prefix_report.get('state_resets_between_segments')!=0 or
            prefix_report.get('bridge_solution')!=solution or prefix_report.get('source')!='oracle'):
            raise ValueError('Executed prefix must be an uninterrupted actual attempt of the selected solution')
        prefix_trace=json.loads((prefix_root/'actual_trace.json').read_text())
        for segment in prefix_report['segments']:
            if not segment.get('success'):break
            passed_prefix[segment['start']]=segment
        report['physically_verified_prefix']=str(prefix_root)
    report.update(source='oracle',embodiment=scene.embodiment,source_fps=schedule.fps,
                  source_duration_s=schedule.num_frames/schedule.fps,vlm_used=False,
                  complete_robot_reference_required=False,relaxed_reference_tracking=False,
                  state_resets_between_segments=0,segments=[],anchors=[],task_success=False,full_pipeline_success=False)
    if bank:
        report.update(bridge_bank=str(bank_root),bridge_solution=solution,
                      path_exclusions={'segments':[],'exclusion_scope':'wrist'},diverse_solution_success=False,
                      previous_actual_rollouts=[str(p) for p in previous_runs])
    for wi,window in enumerate(schedule.windows):
        if before and window.get('anchor')==before:
            report.update(status='predecessor_prefix_complete',prefix_success=True)
            return
        info=dict(window,budget_s=(window['end']-window['start'])/schedule.fps,success=False)
        report['segments'].append(info);start=scene.state()
        if not np.isclose(start['time_s'],window['start']/schedule.fps,rtol=0,atol=1e-8):
            raise RuntimeError('Continuous source clock mismatch at segment entry')
        print('ORACLE_FLOATING_SEGMENT',info,flush=True)
        if window['mode']=='RELAXED':
            bridge=json.loads((cache/f'relaxed_{window["start"]}_{window["end"]}.json').read_text()) if cache else None
            command_path=cache/f'relaxed_{window["start"]}_{window["end"]}.npz' if cache else None
            baseline_command_path=command_path;base_bridge=bridge
            exclusion=None
            if bank and solution>0:
                interval=next(i for i in bank['intervals'] if i['window']==[window['start'],window['end']])
                selected=next((s for s in interval['solutions'] if s['index']==solution),None)
                if not selected or not selected['local_physical_success']:raise RuntimeError('Missing physically validated interval solution')
                stem=bank_root/selected['artifact'];bridge=json.loads(stem.with_suffix('.json').read_text())
                command_path=stem.with_suffix('.npz');exclusion=bridge['exclusions']
                if window['start'] in passed_prefix:
                    previous=passed_prefix[window['start']]
                    entry=next(r for r in prefix_trace if np.isclose(r['source_frame'],window['start']))
                    if measured_drive_matches(start,entry):
                        previous_exclusion=next(e for e in prefix_report['path_exclusions']['segments']
                                                if e['window']==[window['start'],window['end']])
                        bridge=dict(bridge,entry=entry,planning=previous,exclusions=previous_exclusion)
                        exclusion=previous_exclusion
                        command_path=prefix_root/f'relaxed_{window["start"]}_{window["end"]}.npz'
                        info['physically_verified_prefix_drive_reused']=True
                if previous_runs:
                    from multipath.planning.floating_diversity import interval_exclusions
                    baseline_trace=json.loads((cache/'actual_trace.json').read_text())
                    paths=[dict(id='baseline',points=np.asarray([r['wrist_xyz_wxyz'][:3] for r in baseline_trace
                        if window['start']<=r['source_frame']<=window['end']]),
                        command_points=np.vstack((base_bridge['entry']['wrist_xyz_wxyz'][:3],
                            np.load(baseline_command_path)['commands'][:,:3])))]
                    for directory in previous_runs:
                        data=np.load(directory/f'relaxed_{window["start"]}_{window["end"]}.npz')
                        trace=json.loads((directory/'actual_trace.json').read_text())
                        entry=next(r for r in trace if np.isclose(r['source_frame'],window['start']))
                        paths.append(dict(id=str(directory),points=data['actual_wrist_path'],
                            command_points=np.vstack((entry['wrist_xyz_wxyz'][:3],data['commands'][:,:3]))))
                    exclusion=interval_exclusions(paths,[window['start'],window['end']],info['budget_s'],
                        scene.cfg['planning'],exclusion['size_multiplier'],exclusion.get('free_diversity_budget_s'))
                    info['exclusion_from_previous_full_rollouts']=True
                report['path_exclusions']['segments'].append(exclusion)
            if bridge and (not bridge['physical_validation_success'] or bridge['source_provenance']!=manifest['provenance']):
                raise ValueError('Unvalidated or unrelated cuRobo segment artifact')
            carry=window['contact']=='grasped'
            if start['contact']['mode']!=window['contact']:raise RuntimeError('RELAXED cannot change contact mode')
            if carry:
                if held is None:raise RuntimeError('No physically acquired grasp transform')
                check_retention(start,held)
            later=next((w for w in schedule.windows[wi+1:] if w['mode']=='ANCHOR'),None)
            # A prefix before an unvalidated Anchor stops at the preceding exit;
            # it cannot invent the next canonical entry from a human reference.
            prefix=bool(later and later['anchor'] not in canonical)
            terminal_frame=window['end']
            if prefix:
                if later['anchor']!=before:raise RuntimeError('Missing validated canonical Anchor')
                anchor=schedule.anchors[later['anchor']];terminal_frame=anchor.synth_start
                if terminal_frame==window['start']:
                    report['segments'].pop();report.update(status='predecessor_prefix_complete',prefix_success=True);return
                data=dict(np.load(Path(proposal_dir)/manifest['anchors'][anchor.id],allow_pickle=False))
                goal_pose=data['wrist_hand_q'][0,:7].copy()
                if carry:
                    from scipy.spatial.transform import Rotation
                    matrix=data['tool_pose'][0]@np.linalg.inv(held)
                    goal_pose=np.r_[matrix[:3,3],Rotation.from_matrix(matrix[:3,:3]).as_quat()[[3,0,1,2]]]
                goal=dict(wrist_xyz_wxyz=goal_pose,wrist_twist=np.zeros(6),hand_q=start['hand_q'])
                info.update(end=terminal_frame,budget_s=(terminal_frame-window['start'])/schedule.fps,
                            development_prefix=True,canonical_goal=False)
            elif later:goal=canonical[later['anchor']]['entry']
            elif cache:goal=bridge['exit']
            else:
                from multipath.planning.bridge import terminal_stop_target
                goal=terminal_stop_target(start,info['budget_s'],scene.dt,scene.cfg['planning'])
            if later and not prefix and not cache:
                from multipath.planning.bridge import bridge_precheck
                check=bridge_precheck(start,goal,(terminal_frame-window['start'])/schedule.fps,scene.cfg['planning'])
                info['boundary_precheck']=check
                if not check['feasible']:
                    raise RuntimeError(f'Canonical bridge boundaries require local resynthesis: {check["reasons"]}')
            planner=FloatingPlanner(scene,carry,scene.out/f'planner_{window["start"]}',goal_hand=goal.get('command_hand_q') if not carry else None)
            if prefix and not carry:
                reference=np.asarray(goal['wrist_xyz_wxyz']).copy()
                for height in np.linspace(0,float(np.linalg.norm(scene.meshes['tool'].extents)),9):
                    target=reference.copy();target[2]+=height
                    if planner.valid(pose_coordinates(target)):
                        goal['wrist_xyz_wxyz']=target;break
                else:raise RuntimeError('No collision-free development entry')
            if diverse:
                from multipath.planning.floating_diversity import plan_detour,DETOUR_DIRECTION_COUNT,boundary_history_ticks,set_arc_reference
                from multipath.anchors.floating import rebase_grasp_commands
                from multipath.execution.interaction_boundary import grasp_conditioned_boundary
                baseline_commands=np.load(baseline_command_path)['commands']
                if carry:
                    baseline_commands=rebase_grasp_commands(baseline_commands,base_bridge['entry'],scene.snapshot(),scene.dt)
                    goal=grasp_conditioned_boundary(goal,start)
                elif not later:
                    from multipath.planning.floating_diversity import free_terminal_nominal
                    baseline_commands,goal=free_terminal_nominal(scene,start,(terminal_frame-window['start'])*tpf)
                    info['terminal_target_origin']='actual_released_state_free_stop'
                    info['terminal_wrist_endpoint_is_task_constraint']=False
                set_arc_reference(exclusion,start,baseline_commands[-1],
                    (terminal_frame-window['start'])*tpf,scene.dt,scene.cfg['planning'])
                # The bank selects a geometric detour. Each bridge is fitted
                # anew from its actual predecessor q/dq and acquired grasp.
                # Cached canonical contact velocities are not assigned.
                failures=[];selected_seed=bridge['planning']['seed_index']
                # Reuse a physically validated drive when its full measured
                # entry and controller memory match. This avoids changing a
                # proved upstream path solely because the fitter improved.
                entry=bridge['entry'];same_entry=measured_drive_matches(start,entry)
                if not later and not carry:same_entry=False
                if (same_entry and not carry and later and not info.get('physically_verified_prefix_drive_reused') and
                    bridge['planning'].get('common_terminal_history_ticks',0)<boundary_history_ticks(scene,False,
                        (terminal_frame-window['start'])*tpf,True)):
                    same_entry=False
                if same_entry and previous_runs:
                    from multipath.path_exclusion import compare_path
                    cached=np.load(command_path)['commands']
                    planning_exclusion={**exclusion,'cuboid':exclusion['planning_cuboid']}
                    same_entry=compare_path(np.vstack((start['wrist_xyz_wxyz'][:3],cached[:,:3])),
                                            planning_exclusion)['actual_wrist_exclusion_pass']
                if same_entry:
                    commands=np.load(command_path)['commands'];timing=dict(bridge['planning'],
                        actual_predecessor_replanning=False,validated_drive_reused=True)
                else:
                    for seed_index in [selected_seed]+[i for i in range(DETOUR_DIRECTION_COUNT) if i!=selected_seed]:
                        try:
                            commands,timing=plan_detour(planner,scene.snapshot(),baseline_commands[-1],
                                (terminal_frame-window['start'])*tpf,exclusion,goal,seed_index,
                                baseline_commands=baseline_commands,has_next_anchor=bool(later),
                                regularize=bridge['planning'].get('trajectory_cost','').startswith('normalized'),
                                terminal_history_ticks=bridge['planning'].get('common_terminal_history_ticks'))
                            timing.update(actual_predecessor_replanning=True,selected_local_seed=selected_seed,
                                          replanning_rejections=failures)
                            break
                        except RuntimeError as error:failures.append(str(error))
                    else:raise RuntimeError(f'No fixed-time detour from actual predecessor: {failures}')
                if not validate_commands(scene,commands,start)['feasible']:
                    raise RuntimeError('Replanned detour violates finger drive limits')
            elif cache:
                from multipath.anchors.compatibility import boundary_errors,compatible
                errors=boundary_errors(start,bridge['entry'])
                if not compatible(errors):raise RuntimeError(f'Cached bridge entry differs: {errors}')
                commands=np.load(command_path)['commands']
                if commands.shape!=((terminal_frame-window['start'])*tpf,25) or not validate_commands(scene,commands,start)['feasible']:
                    raise RuntimeError('Cached cuRobo commands violate source-time/hand limits')
                timing=bridge['planning']
                info['cached_curobo_entry_errors']=errors
            else:
                commands,timing=planner.plan(np.asarray(goal['wrist_xyz_wxyz']),
                                             (terminal_frame-window['start'])*tpf,np.asarray(goal['wrist_twist']),
                                             goal_state=goal if later and not prefix else None)
            info.update(timing);info['success']=False
            free_model=bool(bridge and bridge['planning'].get('free_motion_dynamics_feedforward'))
            if free_model:info['free_motion_dynamics_feedforward']=True
            info['collision_model']={'current_hand':True,'carried_object':carry,'assembly_self_collision':False}
            np.savez_compressed(scene.out/f'relaxed_{window["start"]}_{window["end"]}.npz',commands=commands)
            from multipath.execution.wrench_handoff import segment_wrenches
            delta_baseline=np.load(baseline_command_path)['commands'] if timing.get('delta_dynamics_feedforward') else None
            wrenches=segment_wrenches(scene,scene.snapshot(),commands,transport=carry,
                baseline_commands=delta_baseline,free_motion=free_model) if cache and (carry or window['start']>0 or delta_baseline is not None or free_model) else np.zeros((len(commands),6))
            actual_wrist=[start['wrist_xyz_wxyz'][:3]]
            contact_steps_at_entry=scene.tool_target_contact_steps
            for j,command in enumerate(commands):
                scene.wrench_bias=wrenches[j]
                scene.step_control(command,'RELAXED',window['start']+(j+1)/tpf)
                if exclusion:actual_wrist.append(scene.state()['wrist_xyz_wxyz'][:3])
                if carry:
                    from multipath.validation.physics import check_relaxed_tool_target_contact
                    check_relaxed_tool_target_contact(scene,contact_steps_at_entry)
                if cache:
                    if carry:check_retention(scene.state(),held)
                    clearance=planner.measured_clearance()
                    if min(clearance.values())<0:
                        collisions=planner.measured_mesh_collisions()
                        if collisions:
                            info['collision_failure']=dict(source_frame=window['start']+(j+1)/tpf,
                                measured_meshes=collisions,clearance_m=clearance)
                            raise RuntimeError('Measured cached RELAXED mesh collision')
                if (timing['hand_boundary_motion'] and not planner.measured_valid()
                        and planner.measured_mesh_collisions()):
                    raise RuntimeError('Measured RELAXED hand boundary collision')
                if (j+1)%tpf==0:
                    if carry:check_retention(scene.state(),held)
                    if not planner.measured_valid() and (not cache or planner.measured_mesh_collisions()):raise RuntimeError('Measured RELAXED collision')
            if exclusion:
                from multipath.path_exclusion import compare_path
                from multipath.planning.floating_diversity import actual_feasible_exclusions
                accepted_exclusion,info['diversity']=actual_feasible_exclusions(exclusion,np.asarray(actual_wrist),info['budget_s'],scene.cfg['planning'])
                if accepted_exclusion is not exclusion:
                    exclusion.clear();exclusion.update(accepted_exclusion)
                np.savez_compressed(scene.out/f'relaxed_{window["start"]}_{window["end"]}.npz',
                                    commands=commands,actual_wrist_path=np.asarray(actual_wrist))
                if not info['diversity']['actual_wrist_exclusion_pass'] or not info['diversity']['smooth_arc_pass']:
                    raise RuntimeError('Continuous measured wrist path failed its interval exclusion')
            if diverse:
                from multipath.execution.interaction_boundary import check_interaction_boundary
                transition=check_interaction_boundary(scene,goal,window['contact'],terminal=not later and not carry)
                info['measured_exit']=scene.snapshot()
            else:
                transition=scene.check_transition(np.asarray(goal['wrist_xyz_wxyz']),window['contact'],canonical=goal if later and not prefix else None)
        else:
            anchor=schedule.anchors[window['anchor']];artifact=canonical[anchor.id]
            if diverse:
                from multipath.execution.interaction_boundary import check_interaction_boundary
                transition=check_interaction_boundary(scene,artifact['entry'],anchor.entry_contact)
            else:
                transition=scene.check_transition(np.asarray(artifact['entry']['wrist_xyz_wxyz']),anchor.entry_contact,canonical=artifact['entry'])
            info['entry_transition']=transition
            if not transition['success']:raise RuntimeError('Physically reached canonical Anchor entry mismatch')
            data=dict(np.load(Path(proposal_dir)/manifest['anchors'][anchor.id],allow_pickle=False))
            commands=artifact['commands']
            prefix_core=False
            if diverse and window['start'] in passed_prefix:
                entry=next(r for r in prefix_trace if np.isclose(r['source_frame'],window['start']))
                previous_artifact=next(a for a in prefix_report['anchors'] if a['id']==anchor.id)
                artifact_matches=all(previous_artifact.get(k)==artifact['report'].get(k)
                    for k in ('entry_provenance','validation'))
                info['prefix_anchor_artifact_matches']=artifact_matches
                if artifact_matches and measured_drive_matches(start,entry):
                    commands=np.load(prefix_root/'anchors'/anchor.id/'executed_core.npz')['commands']
                    info['physically_verified_prefix_core_reused']=True;prefix_core=True
            if diverse and anchor.entry_contact=='grasped' and not prefix_core:
                from multipath.anchors.floating import rebase_grasp_commands
                commands=rebase_grasp_commands(commands,artifact['entry'],scene.snapshot(),scene.dt)
                info['core_grasp_frame_conditioned']=True
            # Preserve attempted controls too, so a real interaction failure
            # can be repaired locally from this physically executed prefix.
            np.savez_compressed(scene.out/'anchors'/anchor.id/'executed_core.npz',commands=commands)
            check=validate_commands(scene,commands,start)
            if not check['feasible']:raise RuntimeError(f'Canonical commands infeasible from actual entry: {check}')
            objective=WindowEvaluation(scene,anchor,data,start)
            if not objective.observe(anchor.core_start,commands[0]):raise RuntimeError(str(objective.failure))
            from multipath.execution.wrench_handoff import segment_wrenches
            handoff=artifact['report']['validation'].get('controller_handoff')
            stable_carry=bool(isinstance(handoff,dict) and handoff.get('carried_dynamics_feedforward'))
            info['carried_dynamics_feedforward']=stable_carry
            wrenches=segment_wrenches(scene,scene.snapshot(),commands,transport=stable_carry) if handoff else np.zeros((len(commands),6))
            for j,command in enumerate(commands):
                scene.wrench_bias=wrenches[j]
                frame=anchor.core_start+(j+1)/tpf;scene.step_control(command,'ANCHOR',frame)
                if (j+1)%tpf==0 and not objective.observe(frame,command):
                    info['failure']=objective.failure;raise RuntimeError(f'Continuous Anchor failed: {objective.failure}')
            if diverse:
                transition=check_interaction_boundary(scene,artifact['exit'],anchor.exit_contact)
                np.savez_compressed(scene.out/'anchors'/anchor.id/'executed_core.npz',commands=commands)
                info['measured_exit']=scene.snapshot()
            else:
                transition=scene.check_transition(np.asarray(artifact['exit']['wrist_xyz_wxyz']),anchor.exit_contact,canonical=artifact['exit'])
            held=np.asarray(scene.state()['hand_object_transform']) if anchor.exit_contact=='grasped' else None
            report['anchors'].append(dict(artifact['report'],handoff_verified=transition['success'],
                core_grasp_frame_conditioned=bool(diverse and anchor.entry_contact=='grasped'),
                measured_entry=start,measured_exit=scene.snapshot(),
                continuous_execution_success=transition['success']))
        info['transition']=transition
        if not transition['success']:raise RuntimeError('Actual segment handoff failed')
        if scene.restore_count:raise RuntimeError('State reset during continuous execution')
        if not np.isclose(scene.ticks*scene.dt,info['end']/schedule.fps,rtol=0,atol=1e-8):
            raise RuntimeError('Source time budget changed')
        info.update(success=True,actual_end_s=scene.ticks*scene.dt)
        (scene.out/'progress.json').write_text(json.dumps(report,indent=2))
        if window['mode']=='RELAXED' and prefix:
            report.update(status='predecessor_prefix_complete',prefix_success=True);return
        if through and window.get('anchor')==through:
            report.update(status='prefix_complete',prefix_success=True);return
    task=evaluate_task(scene.state(),manifest,schedule.metadata['task_contract'],report['anchors'])
    success=bool(task['success'] and len(report['anchors'])==len(schedule.anchors) and scene.restore_count==0)
    report.update(task=task,task_success=task['success'],full_pipeline_success=success,
                  actual_duration_s=scene.ticks*scene.dt,status='complete' if success else 'task_failed')
    if bank:report['diverse_solution_success']=bool(success and solution>0)
    (scene.out/'progress.json').write_text(json.dumps(report,indent=2))
