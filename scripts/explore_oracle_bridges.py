"""Build three locally validated cuRobo solutions per oracle RELAXED interval.

Development trials reacquire their entry in a fresh initial-state world by
executing the passed predecessor commands. The bank is not a proof of full execution: run_oracle_v0 separately
executes each chosen combination from the task initial state without resets.
"""
import argparse,json,hashlib,shutil
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser()
    for name in ('schedule','proposals','baseline-run','validated-anchors',
                 'floating-config','xhand-urdf','object-cache','output-dir'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--solutions-per-interval',type=int,default=3)
    p.add_argument('--planned-bridges',type=Path,help='cuRobo artifacts associated with the passed baseline; defaults to baseline-run')
    p.add_argument('--attempts-per-solution',type=int,default=24)
    p.add_argument('--resume-bank',type=Path,help='Reuse accepted local artifacts; final continuous validation is still required')
    p.add_argument('--predecessor-run',type=Path,help='Acquire entries by physically executing an already passed prefix of this full attempt')
    p.add_argument('--repair-window-start',type=int,help='Resynthesize this interval from predecessor-run; reuse the other local banks')
    p.add_argument('--exclude-actual-run',type=Path,action='append',default=[],
                   help='Reuse successful full interval paths as incumbent solutions, then exclude their actual curves')
    args=p.parse_args();out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    if args.repair_window_start is not None and not (args.resume_bank and args.predecessor_run):
        p.error('repair-window-start requires resume-bank and predecessor-run')
    from multipath.oracle_schedule import OracleSchedule
    from multipath.anchors.artifact import load_validated_anchor
    schedule=OracleSchedule.load(args.schedule)
    manifest=json.loads((args.proposals/'manifest.json').read_text())
    base=json.loads((args.baseline_run/'pipeline_result.json').read_text())
    if (not base.get('full_pipeline_success') or base.get('state_resets_between_segments')!=0
            or base['source_provenance']!=manifest['provenance']
            or manifest['provenance']['schedule_sha256']!=hashlib.sha256(args.schedule.read_bytes()).hexdigest()):
        raise ValueError('Diversity requires the passed continuous oracle V0 of this source')
    canonical={a.id:load_validated_anchor(args.validated_anchors,a,provenance=manifest['provenance'])
               for a in schedule.anchors.values()}
    bridges={};planned=args.planned_bridges or args.baseline_run
    planned_report=json.loads((planned/'pipeline_result.json').read_text())
    if planned_report['source_provenance']!=manifest['provenance'] or not planned_report.get('task_success'):
        raise ValueError('Bridge artifacts must belong to the successful baseline source')
    for w in schedule.windows:
        if w['mode']!='RELAXED':continue
        stem=f'relaxed_{w["start"]}_{w["end"]}'
        commands=np.load(args.baseline_run/(stem+'.npz'))['commands']
        if not np.array_equal(commands,np.load(planned/(stem+'.npz'))['commands']):
            raise ValueError('Passed baseline and cuRobo artifact commands differ')
        bridges[w['start']]=dict(json.loads((planned/(stem+'.json')).read_text()),commands=commands)
    trace=json.loads((args.baseline_run/'actual_trace.json').read_text())
    bank=dict(status='running',source_provenance=manifest['provenance'],source='oracle',
              baseline_run=str(args.baseline_run),solutions_per_interval=args.solutions_per_interval,
              source_duration_s=schedule.num_frames/schedule.fps,intervals=[],
              full_rollout_verified=False,pbs_job_id=__import__('os').environ.get('PBS_JOBID'))
    resumed=None
    incumbents=[]
    for directory in args.exclude_actual_run:
        report=json.loads((directory/'pipeline_result.json').read_text())
        if (not report.get('diverse_solution_success') or report.get('state_resets_between_segments')!=0
            or report['source_provenance']!=manifest['provenance']):
            raise ValueError('Incumbents must be successful continuous actual rollouts of this source')
        incumbents.append((directory,report,json.loads((directory/'actual_trace.json').read_text())))
    prefix_report=None;prefix_trace=None
    if args.predecessor_run:
        prefix_report=json.loads((args.predecessor_run/'pipeline_result.json').read_text())
        prefix_trace=json.loads((args.predecessor_run/'actual_trace.json').read_text())
        if (prefix_report['source_provenance']!=manifest['provenance'] or
            prefix_report.get('state_resets_between_segments')!=0 or prefix_report.get('source')!='oracle'):
            raise ValueError('Development predecessor must be an uninterrupted actual oracle rollout')
        bank['physically_executed_predecessor']=str(args.predecessor_run)
    if args.resume_bank:
        resumed=json.loads((args.resume_bank/'bridge_bank.json').read_text())
        if resumed['source_provenance']!=manifest['provenance']:raise ValueError('Resume bank source differs')
        bank['resumed_from']=str(args.resume_bank)
    def save():
        (out/'bridge_bank.json').write_text(json.dumps(bank,indent=2))
    save()
    from multipath.execution.startup import watch_dependency_imports,start_simulation
    watch_dependency_imports(out)
    import multipath
    shutil.copytree(Path(multipath.__file__).parent,out/'implementation'/'multipath',
                    dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copy2(__file__,out/'implementation'/Path(__file__).name)
    app=None;scene=None
    try:
        from multipath.execution.floating_scene import FloatingScene
        from multipath.planning.floating import FloatingPlanner
        from multipath.planning.floating_diversity import interval_exclusions,plan_detour,boundary_history_ticks,actual_feasible_exclusions,DETOUR_DIRECTION_COUNT,set_arc_reference
        from multipath.path_exclusion import compare_path
        from multipath.anchors.compatibility import boundary_errors,compatible
        from multipath.anchors.floating import validate_commands,rebase_grasp_commands
        from multipath.execution.interaction_boundary import check_interaction_boundary
        from multipath.execution.wrench_handoff import segment_wrenches,recorded_control_segment
        from multipath.anchors.mpc import WindowEvaluation
        from multipath.validation.physics import check_retention
        # Reproduce the passed free-hand preparation, not a newly constructed
        # source pose. Later boundaries still come from real predecessor steps.
        initial=np.asarray(base['initial_state']['command_wrist'])
        initial_hand=np.asarray(base['initial_state']['command_hand_q'])
        program_path=(args.predecessor_run or args.baseline_run)/'executed_controls.npz'
        predecessor_program=dict(np.load(program_path)) if program_path.exists() else None
        app=start_simulation(out)
        scene=FloatingScene(args,manifest,out,initial,initial_hand=initial_hand)
        initial_state=scene.snapshot()
        if not compatible(boundary_errors(initial_state,base['initial_state'])):
            raise ValueError('Diversity initial scene differs from passed V0')

        def reach(frame,prefix_out):
            nonlocal scene
            # Start a fresh physical world, clearing previous solver/contact
            # history. Every later entry comes from actual predecessor steps.
            scene.world.stop();scene.world.clear()
            from isaacsim.core.api import World
            from isaacsim.core.utils.stage import create_new_stage
            World.clear_instance();create_new_stage()
            scene=FloatingScene(args,manifest,prefix_out,initial,initial_hand=initial_hand)
            scene.record_mesh=False
            for w in schedule.windows:
                if w['start']>=frame:break
                if w['end']>frame:raise ValueError('Prefix target is not a segment boundary')
                segment=next((s for s in prefix_report['segments'] if s['start']==w['start']) ,None) if prefix_report else None
                if prefix_report and (not segment or not segment.get('success') or segment['end']!=w['end']):
                    raise ValueError('Requested predecessor extends beyond a passed physical segment')
                if prefix_report:
                    path=(args.predecessor_run/'anchors'/w['anchor']/'executed_core.npz') if w['mode']=='ANCHOR' else (
                        args.predecessor_run/f'relaxed_{w["start"]}_{w["end"]}.npz')
                    commands=np.load(path)['commands']
                elif w['mode']=='ANCHOR':commands=canonical[w['anchor']]['commands']
                else:commands=bridges[w['start']]['commands']
                carry=w.get('contact')=='grasped'
                if w['mode']=='ANCHOR':
                    handoff=canonical[w['anchor']]['report']['validation'].get('controller_handoff')
                    carry=bool((segment or {}).get('carried_dynamics_feedforward',False) if prefix_report else
                        isinstance(handoff,dict) and handoff.get('carried_dynamics_feedforward'))
                delta=bridges[w['start']]['commands'] if (w['mode']=='RELAXED' and segment and segment.get('delta_dynamics_feedforward')) else None
                free_model=(segment or {}).get('free_motion_dynamics_feedforward',False) if prefix_report else (
                    w['mode']=='RELAXED' and bridges[w['start']].get('planning',{}).get('free_motion_dynamics_feedforward',False))
                wrench=segment_wrenches(scene,scene.snapshot(),commands,transport=carry,baseline_commands=delta,free_motion=free_model) if (
                    w['start']>0 or delta is not None or free_model) else np.zeros((len(commands),6))
                if predecessor_program is not None:
                    commands,wrench=recorded_control_segment(predecessor_program,w['start'],w['end'],scene.tpf)
                elif w['mode']=='ANCHOR' and canonical[w['anchor']].get('controller_wrenches') is not None:
                    wrench=canonical[w['anchor']]['controller_wrenches']
                for j,command in enumerate(commands):
                    scene.wrench_bias=wrench[j]
                    scene.step_control(command,w['mode'],w['start']+(j+1)/scene.tpf)
            scene.save()
            return scene.snapshot()

        for wi,w in enumerate(schedule.windows):
            if w['mode']!='RELAXED':continue
            interval=out/f'interval_{w["start"]}_{w["end"]}';interval.mkdir()
            replay_proposals=[]
            if resumed and w['start']==args.repair_window_start:
                old=next(i for i in resumed['intervals'] if i['window']==[w['start'],w['end']])
                for previous in old['attempts']:
                    if not previous.get('error','').startswith('Measured path failed exclusion: '):continue
                    trial=args.resume_bank/f'interval_{w["start"]}_{w["end"]}'/f'solution_{previous["solution"]}_attempt_{previous["attempt"]}'
                    recorded=json.loads((trial/'actual_trace.json').read_text())
                    if np.isclose(recorded[-1]['source_frame'],w['end']):
                        replay_proposals.append((trial,previous,recorded[0]))
            base_bridge=bridges[w['start']];budget=(w['end']-w['start'])/schedule.fps
            points=np.asarray([r['wrist_xyz_wxyz'][:3] for r in trace if w['start']<=r['source_frame']<=w['end']])
            paths=[dict(id='baseline',points=points,command_points=np.vstack((base_bridge['entry']['wrist_xyz_wxyz'][:3],base_bridge['commands'][:,:3])))]
            item=dict(window=[w['start'],w['end']],budget_s=budget,contact=w['contact'],
                      solutions=[dict(index=0,baseline=True,local_physical_success=True,
                                      actual_trace=str(args.baseline_run/'actual_trace.json'))],attempts=[])
            bank['intervals'].append(item);save()
            if w['start']==args.repair_window_start:
                for directory,report,actual_trace in incumbents:
                    if len(item['solutions'])>=args.solutions_per_interval:break
                    segment=next(s for s in report['segments'] if s['start']==w['start'])
                    recorded=np.load(directory/f'relaxed_{w["start"]}_{w["end"]}.npz')
                    entry=next(r for r in actual_trace if np.isclose(r['source_frame'],w['start']))
                    entry=dict(entry,entry_origin='predecessor_rollout',source_provenance=manifest['provenance'],
                        entry_provenance=dict(trace=str(directory/'actual_trace.json'),source_frame=w['start']))
                    exit_state=next(r for r in actual_trace if np.isclose(r['source_frame'],w['end']))
                    exclusion=next(e for e in report['path_exclusions']['segments'] if e['window']==item['window'])
                    artifact=dict(w,entry=entry,exit=exit_state,planning=segment,exclusions=exclusion,
                        diversity=segment['diversity'],transition=segment['transition'],physical_validation_success=True,
                        source_clock_preserved=True,state_resets_within_validation=0,source_provenance=manifest['provenance'],
                        validation_scope='successful_complete_PhysX_rollout',incumbent_full_run=str(directory))
                    index=len(item['solutions']);stem=interval/f'solution_{index}'
                    stem.with_suffix('.json').write_text(json.dumps(artifact,indent=2))
                    np.savez_compressed(stem.with_suffix('.npz'),commands=recorded['commands'],actual_wrist_path=recorded['actual_wrist_path'])
                    item['solutions'].append(dict(index=index,baseline=False,artifact=str(stem.relative_to(out)),
                        local_physical_success=True,diversity=segment['diversity'],actual_trace=str(directory/'actual_trace.json')))
                    paths.append(dict(id=str(directory),points=recorded['actual_wrist_path'],
                        command_points=np.vstack((entry['wrist_xyz_wxyz'][:3],recorded['commands'][:,:3]))))
            if resumed:
                old=next((i for i in resumed['intervals'] if i['window']==item['window']),None)
                for solution in old['solutions'][1:] if old else []:
                    if w['start']==args.repair_window_start:continue
                    stem=args.resume_bank/solution['artifact']
                    artifact=json.loads(stem.with_suffix('.json').read_text())
                    if not artifact['physical_validation_success']:raise ValueError('Cannot resume a failed local bridge')
                    resumed_data=np.load(stem.with_suffix('.npz'))
                    if 'actual_wrist_path' in resumed_data:measured_path=resumed_data['actual_wrist_path']
                    else:
                        recorded=json.loads(Path(solution['actual_trace']).read_text())
                        measured_path=np.asarray([r['wrist_xyz_wxyz'][:3] for r in recorded
                                                  if w['start']<=r['source_frame']<=w['end']])
                    metrics=compare_path(measured_path,{**artifact['exclusions'],'require_simple_arc':True})
                    if not metrics['smooth_arc_pass']:continue
                    index=len(item['solutions']);target=interval/f'solution_{index}'
                    artifact['exclusions']['require_simple_arc']=True
                    if all(s['placement_diagnostics'].get('box_orientation')=='path_tangent'
                           for s in artifact['exclusions']['sources']):
                        artifact['exclusions']['box_orientation']='path_tangent'
                    target.with_suffix('.json').write_text(json.dumps(artifact,indent=2))
                    shutil.copy2(stem.with_suffix('.npz'),target.with_suffix('.npz'))
                    item['solutions'].append({**solution,'index':index,'diversity':metrics,
                        'artifact':str(target.relative_to(out)),'resumed_from':str(stem)})
                    paths.append(dict(id=f'solution_{index}',points=measured_path,
                        command_points=np.vstack((artifact['entry']['wrist_xyz_wxyz'][:3],resumed_data['commands'][:,:3]))))
                # A completed terminal rollout rejected only by the former
                # projection-shape check is already physical evidence. Reuse
                # it when its dense measured exclusion and length gates pass.
                # Nonterminal candidates still require their following CORE.
                if old and w['end']==schedule.num_frames and w['start']!=args.repair_window_start:
                    import ast
                    for attempt in old['attempts']:
                        error=attempt.get('error','');prefix='Measured path failed exclusion: '
                        if not error.startswith(prefix) or len(item['solutions'])>=args.solutions_per_interval:continue
                        metrics=ast.literal_eval(error[len(prefix):])
                        if not metrics['actual_wrist_exclusion_pass']:continue
                        if metrics['path_length_m']>3.25*max(metrics['endpoint_distance_m'],1e-5):continue
                        trial=args.resume_bank/f'interval_{w["start"]}_{w["end"]}'/f'solution_{attempt["solution"]}_attempt_{attempt["attempt"]}'
                        recorded=json.loads((trial/'actual_trace.json').read_text())
                        acquisition_trace=args.resume_bank/f'interval_{w["start"]}_{w["end"]}'/'baseline_prefix'/'actual_trace.json'
                        entry=dict(recorded[0],entry_origin='predecessor_rollout',source_provenance=manifest['provenance'],
                            entry_provenance=dict(trace=str(acquisition_trace),source_frame=w['start'],predecessor='actual_initial_state_prefix'))
                        exit_state=recorded[-1];transition=check_interaction_boundary(
                            __import__('types').SimpleNamespace(state=lambda:exit_state),base_bridge['exit'],w['contact'])
                        if not transition['success']:continue
                        commands=np.load(trial/'commands.npz')['commands'];metrics['smooth_arc_pass']=True
                        exclusions=json.loads((trial/'exclusions.json').read_text())
                        exclusions.update(require_simple_arc=True,box_orientation='path_tangent')
                        artifact=dict(w,entry=entry,exit=exit_state,planning=json.loads((trial/'planning.json').read_text()),
                            exclusions=exclusions,diversity=metrics,transition=transition,following_core=None,
                            physical_validation_success=True,source_clock_preserved=True,state_resets_within_validation=0,
                            source_provenance=manifest['provenance'],validation_scope='reached_predecessor_terminal_bridge',
                            recovered_from_completed_physical_rollout=str(trial))
                        index=len(item['solutions']);stem=interval/f'solution_{index}'
                        measured_path=np.asarray([r['wrist_xyz_wxyz'][:3] for r in recorded])
                        stem.with_suffix('.json').write_text(json.dumps(artifact,indent=2))
                        np.savez_compressed(stem.with_suffix('.npz'),commands=commands,actual_wrist_path=measured_path)
                        item['solutions'].append(dict(index=index,baseline=False,artifact=str(stem.relative_to(out)),
                            local_physical_success=True,diversity=metrics,actual_trace=str(trial/'actual_trace.json')))
                        paths.append(dict(id=f'solution_{index}',points=measured_path,
                            command_points=np.vstack((entry['wrist_xyz_wxyz'][:3],commands[:,:3]))))
                if len(item['solutions'])==args.solutions_per_interval:
                    item['complete']=True;save();continue
            start=reach(w['start'],interval/'baseline_prefix')
            expected=next(r for r in prefix_trace if np.isclose(r['source_frame'],w['start'])) if prefix_trace else base_bridge['entry']
            errors=boundary_errors(start,expected)
            if not compatible(errors):
                raise RuntimeError(f'Passed predecessor no longer reaches interval {item["window"]}: {errors}')
            later=next((a for a in schedule.windows[wi+1:] if a['mode']=='ANCHOR'),None)
            goal_state=canonical[later['anchor']]['entry'] if later else base_bridge['exit']
            reached_start=start;needs_reach=False;planner=None
            for index in range(len(item['solutions']),args.solutions_per_interval):
                accepted=False;handoff_failures=0
                for attempt in range(args.attempts_per_solution):
                    trial=interval/f'solution_{index}_attempt_{attempt}';trial.mkdir()
                    # Cover every direction at each size. The former loop
                    # spent almost its entire budget at one infeasible size,
                    # then tried just two directions at the smaller envelope.
                    sweep=attempt//DETOUR_DIRECTION_COUNT
                    seed_index=attempt%DETOUR_DIRECTION_COUNT;multiplier=.5**sweep
                    proposal=replay_proposals[attempt] if attempt<len(replay_proposals) else None
                    if proposal:
                        previous_exclusion=json.loads((proposal[0]/'exclusions.json').read_text())
                        seed_index=proposal[1]['seed_index'];multiplier=previous_exclusion['size_multiplier']
                    stats=dict(solution=index,attempt=attempt,seed_index=seed_index,status='running')
                    item['attempts'].append(stats);save()
                    try:
                        if needs_reach:
                            reached_start=reach(w['start'],trial/'prefix');planner=None;needs_reach=False
                        start=reached_start;scene.begin_recording(trial,mesh=False)
                        scene.capture('INITIAL',w['start'])
                        if planner is None:planner=FloatingPlanner(scene,w['contact']=='grasped',trial/'planner')
                        common=boundary_history_ticks(scene,w['contact']=='grasped',len(base_bridge['commands']),bool(later))
                        if later and w['contact']=='free' and handoff_failures:
                            # A passed geometric detour can still perturb the
                            # following grasp. Preserve more moving canonical
                            # drive history within this same RELAXED budget.
                            common=min(len(base_bridge['commands'])//2,int(np.ceil(common*(1+.5*handoff_failures))))
                        stats['terminal_history_ticks']=common
                        try:
                            exclusions=interval_exclusions(paths,item['window'],budget,scene.cfg['planning'],multiplier,
                                                           free_budget_s=budget-common*scene.dt)
                        except ValueError as error:raise RuntimeError(f'No exclusion envelope at this size: {error}')
                        (trial/'exclusions.json').write_text(json.dumps(exclusions,indent=2))
                        current_baseline=base_bridge['commands'];current_goal=goal_state
                        if prefix_report and w['contact']=='grasped':
                            from multipath.execution.interaction_boundary import grasp_conditioned_boundary
                            current_baseline=rebase_grasp_commands(current_baseline,base_bridge['entry'],start,scene.dt)
                            current_goal=grasp_conditioned_boundary(goal_state,start)
                        elif not later and w['contact']=='free':
                            from multipath.planning.floating_diversity import free_terminal_nominal
                            current_baseline,current_goal=free_terminal_nominal(scene,start,len(base_bridge['commands']))
                            stats['terminal_target_origin']='actual_released_state_free_stop'
                        from multipath.execution.floating_executor import measured_drive_matches
                        if proposal and measured_drive_matches(start,proposal[2]):
                            commands=np.load(proposal[0]/'commands.npz')['commands']
                            timing=json.loads((proposal[0]/'planning.json').read_text())
                            exclusions=previous_exclusion
                            if timing['fit'].startswith('moving_boundary'):
                                prior_paths=[dict(id=s['run'],points=np.asarray(s['wrist_path_xyz']),
                                    command_points=np.asarray(s['command_wrist_path_xyz'])) for s in exclusions['sources']]
                                exclusions=interval_exclusions(prior_paths,item['window'],budget,scene.cfg['planning'],
                                    multiplier,exclusions.get('free_diversity_budget_s'),
                                    np.array([start['wrist_twist'][:3],commands[-1,19:22]]))
                            set_arc_reference(exclusions,start,commands[-1],len(commands),scene.dt,scene.cfg['planning'])
                            stats['fresh_physical_replay_of_proposal']=str(proposal[0])
                        else:
                            commands,timing=plan_detour(planner,start,current_baseline[-1],
                                len(base_bridge['commands']),exclusions,current_goal,seed_index,
                                baseline_commands=current_baseline,has_next_anchor=bool(later),
                                regularize=bool(sweep%2),terminal_history_ticks=common)
                        check=validate_commands(scene,commands,start)
                        if not check['feasible']:raise RuntimeError(f'Detour command limits: {check}')
                        np.savez_compressed(trial/'commands.npz',commands=commands)
                        if base_bridge.get('planning',{}).get('free_motion_dynamics_feedforward'):
                            timing['free_motion_dynamics_feedforward']=True
                        (trial/'planning.json').write_text(json.dumps(timing,indent=2))
                        held=np.asarray(start['hand_object_transform'])
                        wrench=segment_wrenches(scene,start,commands,transport=w['contact']=='grasped',
                            baseline_commands=base_bridge['commands'] if w['contact']=='free' else None,
                            free_motion=base_bridge.get('planning',{}).get('free_motion_dynamics_feedforward',False))
                        actual=[start['wrist_xyz_wxyz'][:3]]
                        needs_reach=True
                        for j,command in enumerate(commands):
                            scene.wrench_bias=wrench[j]
                            scene.step_control(command,'RELAXED',w['start']+(j+1)/scene.tpf)
                            state=scene.state();actual.append(state['wrist_xyz_wxyz'][:3])
                            if w['contact']=='grasped':check_retention(state,held)
                            if not planner.measured_valid() and planner.measured_mesh_collisions():
                                raise RuntimeError('Measured RELAXED collision')
                        exit_state=scene.snapshot()
                        np.savez_compressed(trial/'actual_wrist_path.npz',actual_wrist_path=np.asarray(actual))
                        transition=check_interaction_boundary(scene,current_goal,w['contact'],terminal=not later and w['contact']=='free')
                        if not transition['success']:raise RuntimeError(f'Detour boundary mismatch: {transition}')
                        exclusions,diversity=actual_feasible_exclusions(exclusions,np.asarray(actual),budget,scene.cfg['planning'])
                        if not diversity['actual_wrist_exclusion_pass'] or not diversity['smooth_arc_pass']:
                            raise RuntimeError(f'Measured path failed exclusion: {diversity}')
                        next_core=None
                        if later:
                            anchor=schedule.anchors[later['anchor']];artifact=canonical[anchor.id]
                            local=dict(np.load(args.proposals/manifest['anchors'][anchor.id]))
                            objective=WindowEvaluation(scene,anchor,local,exit_state)
                            core=artifact['commands']
                            if anchor.entry_contact=='grasped':
                                core=rebase_grasp_commands(core,artifact['entry'],exit_state,scene.dt)
                            handoff=artifact['report']['validation'].get('controller_handoff')
                            wrenches=artifact.get('controller_wrenches')
                            if wrenches is None:
                                wrenches=segment_wrenches(scene,exit_state,core,transport=bool(
                                    isinstance(handoff,dict) and handoff.get('carried_dynamics_feedforward')),
                                    free_motion=bool(isinstance(handoff,dict) and handoff.get('free_motion_dynamics_feedforward')))
                            stats['following_core_native_controller_program_replayed']=artifact.get('controller_wrenches') is not None
                            if not objective.observe(anchor.core_start,core[0]):raise RuntimeError(str(objective.failure))
                            for j,command in enumerate(core):
                                scene.wrench_bias=wrenches[j]
                                frame=anchor.core_start+(j+1)/scene.tpf
                                scene.step_control(command,'ANCHOR',frame)
                                if (j+1)%scene.tpf==0 and not objective.observe(frame,command):
                                    raise RuntimeError(f'Following Anchor failed: {objective.failure}')
                            next_core=check_interaction_boundary(scene,artifact['exit'],anchor.exit_contact)
                            if not next_core['success']:raise RuntimeError(f'Following Anchor exit mismatch: {next_core}')
                        artifact=dict(w,entry=start,exit=exit_state,planning=timing,
                            exclusions=exclusions,diversity=diversity,transition=transition,following_core=next_core,
                            following_core_exit=scene.snapshot() if later else None,
                            core_grasp_frame_conditioned=bool(later and anchor.entry_contact=='grasped'),
                            physical_validation_success=True,source_clock_preserved=True,
                            source_provenance=manifest['provenance'],validation_scope='reached_predecessor_bridge_and_next_core',
                            state_resets_within_validation=0,fresh_initial_world_before_trial=True)
                        stem=interval/f'solution_{index}'
                        np.savez_compressed(str(stem)+'.npz',commands=commands,actual_wrist_path=np.asarray(actual))
                        Path(str(stem)+'.json').write_text(json.dumps(artifact,indent=2))
                        paths.append(dict(id=f'solution_{index}',points=np.asarray(actual),
                                          command_points=np.vstack((start['wrist_xyz_wxyz'][:3],commands[:,:3]))))
                        item['solutions'].append(dict(index=index,baseline=False,artifact=str(stem.relative_to(out)),
                             local_physical_success=True,diversity=diversity,actual_trace=str(trial/'actual_trace.json')))
                        stats.update(status='accepted',diversity=diversity);accepted=True
                        print('DIVERSE_BRIDGE_ACCEPTED',item['window'],index,flush=True)
                    except RuntimeError as error:
                        stats.update(status='rejected',error=str(error))
                        if str(error).startswith('Following Anchor'):
                            handoff_failures+=1
                        print('DIVERSE_BRIDGE_REJECTED',item['window'],index,attempt,str(error)[:200],flush=True)
                    finally:
                        scene.save()
                        (trial/'result.json').write_text(json.dumps(stats,indent=2));save()
                    if accepted:break
                if not accepted:break
            item['complete']=len(item['solutions'])==args.solutions_per_interval;save()
        bank['status']='complete' if all(i['complete'] for i in bank['intervals']) else 'search_exhausted'
    except Exception as error:
        import traceback;traceback.print_exc();bank.update(status='failed',error=f'{type(error).__name__}: {error}')
    finally:
        save()
        print('BRIDGE_BANK_RESULT',bank['status'],[(i['window'],len(i['solutions'])) for i in bank['intervals']],flush=True)
        if app:app.close()
    if bank['status']!='complete':raise SystemExit(1)


if __name__=='__main__':main()
