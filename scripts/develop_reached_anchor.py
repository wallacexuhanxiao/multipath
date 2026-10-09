"""Refine a local Anchor at the core entry reached by a real cuRobo prefix."""
import argparse,json
from pathlib import Path


def main():
    p=argparse.ArgumentParser()
    for name in ('schedule','proposals','entry-run','xhand-urdf','object-cache','floating-config','output-dir'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--context-controls',type=Path,help='Optional padded warm start; otherwise derive stabilizing post-context from this local core proposal.')
    p.add_argument('--planned-bridges',type=Path,help='Original cuRobo drives used by a diverse runtime prefix for its dynamics feedforward.')
    p.add_argument('--bridge-context-run',type=Path,help='Actual failed following carry bridge; its planned commands condition local post-context.')
    p.add_argument('--exit-geometry-feedback-run',type=Path,help='Actual free-exit collision; reshape this Anchor before local physical validation.')
    p.add_argument('--anchor-id',required=True)
    p.add_argument('--mpc-candidates',type=int,default=16);p.add_argument('--mpc-iters',type=int,default=2)
    p.add_argument('--ppo-updates',type=int,default=20)
    args=p.parse_args();out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    from multipath.execution.startup import watch_dependency_imports,start_simulation
    watch_dependency_imports(out)
    import numpy as np
    from dataclasses import replace
    from multipath.oracle_schedule import OracleSchedule
    from multipath.execution.floating_scene import FloatingScene
    from multipath.anchors.floating import rollout,project_hand_commands
    from multipath.anchors.floating_search import refine
    from multipath.anchors.mpc import RefinementRequired
    from multipath.anchors.ppo import PPOConfig
    from multipath.anchors.artifact import save_anchor
    from multipath.planning.timing import physics_ticks_per_frame
    from multipath.execution.wrench_handoff import segment_wrenches
    schedule=OracleSchedule.load(args.schedule);anchor=schedule.anchors[args.anchor_id]
    manifest=json.loads((args.proposals/'manifest.json').read_text())
    predecessor=json.loads((args.entry_run/'pipeline_result.json').read_text())
    if predecessor['source_provenance']!=manifest['provenance']:raise ValueError('Predecessor source mismatch')
    if any(not s['success'] for s in predecessor['segments'] if s['end']<=anchor.core_start):
        raise ValueError('Core entry was not reached by successful predecessors')
    runtime_prefix=not (args.entry_run/'cores'/anchor.id/'entry.json').exists()
    if runtime_prefix:
        trace=json.loads((args.entry_run/'actual_trace.json').read_text())
        entry=dict(next(s for s in trace if np.isclose(s['source_frame'],anchor.core_start)),
            entry_origin='predecessor_rollout',source_provenance=manifest['provenance'],
            entry_provenance=dict(trace=str(args.entry_run/'actual_trace.json'),source_frame=anchor.core_start))
        if not entry.get('physically_acquired_state') or predecessor.get('state_resets_between_segments')!=0:
            raise ValueError('Runtime local repair requires a continuous physically acquired entry')
    else:entry=json.loads((args.entry_run/'cores'/anchor.id/'entry.json').read_text())
    data=dict(np.load(args.proposals/manifest['anchors'][anchor.id],allow_pickle=False))
    tpf=physics_ticks_per_frame(schedule.fps,FloatingScene.dt)
    skip=anchor.core_start-anchor.synth_start
    local=replace(anchor,synth_start=anchor.core_start)
    cropped={k:(v[skip:] if v.ndim and len(v)==anchor.synth_end-anchor.synth_start else v) for k,v in data.items()}
    core_path=(args.entry_run/'anchors'/anchor.id/'executed_core.npz' if runtime_prefix
               else args.entry_run/'cores'/anchor.id/'commands.npz')
    core=np.load(core_path)['commands']
    bridge_context=None
    if args.bridge_context_run:
        context_report=json.loads((args.bridge_context_run/'pipeline_result.json').read_text())
        if context_report['source_provenance']!=manifest['provenance']:
            raise ValueError('Carry context source mismatch')
        window=next(w for w in schedule.windows if w['start']==anchor.core_end)
        if window['mode']!='RELAXED' or window.get('contact')!='grasped':
            raise ValueError('Carry context requires the outgoing grasped RELAXED interval')
        bridge_context=np.load(args.bridge_context_run/f'relaxed_{window["start"]}_{window["end"]}.npz')['commands']
        needed=(anchor.synth_end-anchor.core_end)*tpf
        if len(bridge_context)<needed:raise ValueError('Planned bridge does not cover this local padding')
        bridge_context=bridge_context[:needed];post=bridge_context.copy()
    elif args.context_controls:
        context=np.load(args.context_controls)['commands']
        post=context[(anchor.core_end-anchor.synth_start)*tpf:]
    else:
        from multipath.anchors.floating import stopping_context
        post=stopping_context(core[-1],(anchor.synth_end-anchor.core_end)*tpf,FloatingScene.dt)
    controls=np.concatenate((core,post))
    report=dict(anchor_id=anchor.id,core_success=False,full_pipeline_success=False,source_provenance=manifest['provenance'],
        synthesis_interval=[anchor.synth_start,anchor.synth_end],optimization_interval=[local.synth_start,local.synth_end],
        pre_context='actual cuRobo predecessor; no reference state initialization',entry_run=str(args.entry_run))
    first=min(schedule.anchors.values(),key=lambda a:a.core_start)
    initial_data=dict(np.load(args.proposals/manifest['anchors'][first.id],allow_pickle=False))
    # Recreate the predecessor's actual initial preparation. A template can
    # choose a free pregrasp posture; raw synth_start fingers are then a
    # different initial condition, even when the bridge commands are identical.
    initial=np.asarray(predecessor['initial_state']['command_wrist'])
    initial_hand=np.asarray(predecessor['initial_state']['command_hand_q'])
    app=None;scene=None
    try:
        app=start_simulation(out);scene=FloatingScene(args,manifest,out/'initial',initial,initial_hand=initial_hand)
        if anchor.exit_contact=='free' and anchor.synth_end==schedule.num_frames:
            scene.terminal_task_contract=schedule.metadata['task_contract']
            report['terminal_task_contract']=scene.terminal_task_contract
        scene.freeze_carry_core=anchor.exit_contact=='grasped'
        # Actual outgoing speed is a planning boundary requirement, not an
        # actuator acceleration gate. This protects the already observed case
        # where a locally valid interaction cannot enter its fixed-time bridge.
        scene.bridge_exit_constraints=True
        exit_path=args.entry_run/'cores'/anchor.id/'exit.json'
        if anchor.exit_contact=='grasped' and exit_path.exists():
            # A real grasp can still leave the object table-supported. Use
            # that observed exit to propose clearance inside this same CORE;
            # the proposal must pass fresh physical Replay and source fidelity.
            from multipath.planning.bridge import carried_object_clearance
            state=json.loads(exit_path.read_text());clearance=carried_object_clearance(scene,state)
            if clearance['table']<0:
                trace=json.loads((args.entry_run/'actual_trace.json').read_text())
                states=[s for s in trace if anchor.core_start<=s['source_frame']<=anchor.core_end]
                acquired=next((s['source_frame'] for i,s in enumerate(states)
                    if all(v['contact']['mode']=='grasped' for v in states[i:])),None)
                if acquired is not None and acquired<anchor.core_end:
                    height=-clearance['table']+.002
                    duration=(anchor.core_end-acquired)/schedule.fps
                    planning=scene.cfg['planning']
                    minimum=max(1.875*height/planning['velocity_limits'][2],
                        np.sqrt(5.773503*height/planning['acceleration_limits'][2]))
                    if duration>=minimum:
                        frames=anchor.core_start+np.arange(1,len(controls)+1)/tpf
                        phase=np.clip((frames-acquired)/(anchor.core_end-acquired),0,1)
                        shift=height*(10*phase**3-15*phase**4+6*phase**5)
                        controls[:,2]+=shift
                        controls[:,21]+=np.diff(np.r_[0.,shift])/scene.dt
                    report['carry_exit_geometry_feedback']=dict(measured_clearance_m=clearance,
                        lift_proposal_m=height,physically_acquired_grasp_frame=acquired,
                        available_duration_s=duration,minimum_nominal_duration_s=float(minimum),
                        proposal_applied=duration>=minimum,
                        proposal_only=True,source_time_stretch=False)
        if bridge_context is not None:
            scene.post_context_commands=bridge_context
            report['post_context_origin']='planned carry commands from observed failed bridge; fresh physical acceptance required'
        initial_state=scene.snapshot()
        if args.exit_geometry_feedback_run:
            from multipath.planning.bridge import free_exit_clearance_offset
            from multipath.anchors.floating import quiet_release_exit,stopping_context
            feedback_run=args.exit_geometry_feedback_run
            feedback_report=json.loads((feedback_run/'pipeline_result.json').read_text())
            if feedback_report['source_provenance']!=manifest['provenance'] or feedback_report.get('state_resets_between_segments')!=0:
                raise ValueError('Free exit feedback must be a continuous actual rollout of this source')
            state=next(s for s in json.loads((feedback_run/'actual_trace.json').read_text()) if np.isclose(s['source_frame'],anchor.core_end))
            geometry=json.loads((feedback_run/f'planner_{anchor.core_end}'/'collision_geometry.json').read_text())
            import ast
            reported=ast.literal_eval(feedback_report['error'].split('Actual mesh intersects a planning obstacle at entry: ',1)[1])
            roles=[item['obstacle'] if isinstance(item,dict) else item for item in reported]
            offset,feedback=free_exit_clearance_offset(scene,state,geometry,obstacle_roles=roles)
            core=quiet_release_exit(scene,anchor,core,offset,preserve_motion=True)
            post=stopping_context(core[-1],(anchor.synth_end-anchor.core_end)*tpf,scene.dt)
            controls=np.concatenate((core,post))
            report['exit_geometry_feedback']=dict(feedback,physical_run=str(feedback_run),
                original_rotation_and_finger_commands_preserved=True)
        controls=project_hand_commands(scene,controls,entry,local)
        def predecessor_commands(window):
            path=(args.entry_run/f'relaxed_{window["start"]}_{window["end"]}.npz' if window['mode']=='RELAXED'
                  else args.entry_run/'anchors'/window['anchor']/'executed_core.npz' if runtime_prefix
                  else args.entry_run/'cores'/window['anchor']/'commands.npz')
            return np.load(path)['commands']
        def reach(frame,folder,record_from=None):
            scene.begin_initial_trial(folder,mesh=False)
            if record_from==0:
                scene.capture('PHYSICAL_CONTEXT_ENTRY',0)
                scene.context_entry=scene.snapshot()
            for window in schedule.windows:
                if window['start']>=frame:break
                command=predecessor_commands(window)[:(min(window['end'],frame)-window['start'])*tpf]
                carry=window.get('contact')=='grasped'
                segment=next(s for s in predecessor['segments'] if s['start']==window['start'])
                delta=None
                if segment.get('delta_dynamics_feedforward'):
                    if not args.planned_bridges:raise ValueError('Diverse prefix requires its baseline cuRobo drives')
                    delta=np.load(args.planned_bridges/f'relaxed_{window["start"]}_{window["end"]}.npz')['commands'][:len(command)]
                free_model=segment.get('planning',segment).get('free_motion_dynamics_feedforward',False)
                wrench=segment_wrenches(scene,scene.snapshot(),command,transport=carry,baseline_commands=delta,free_motion=free_model) if (
                    window['start']>0 or delta is not None or free_model) else np.zeros((len(command),6))
                for k,c in enumerate(command):
                    source_frame=window['start']+(k+1)/tpf
                    scene.wrench_bias=wrench[k];scene.step_control(c,window['mode'],source_frame)
                    if record_from is not None and source_frame==record_from:
                        scene.begin_recording(folder,mesh=False)
                        scene.capture('PHYSICAL_CONTEXT_ENTRY',source_frame)
                        scene.context_entry=scene.snapshot()
            scene.save();return scene.snapshot()
        entry=reach(anchor.core_start,out/'replay_entry_acquisition')
        acquisition_synthesis=anchor.entry_contact=='free' and anchor.exit_contact=='grasped'
        if acquisition_synthesis:
            from multipath.anchors.floating import source_retarget_core_controls,stopping_context
            # Failed runtime commands contain an executed feedback prefix and
            # an untouched future. Rebuild a local nominal proposal instead
            # of optimizing that hybrid as if it were one executable Replay.
            core=source_retarget_core_controls(scene,anchor,data,entry)
            post=(np.asarray(bridge_context).copy() if bridge_context is not None else
                  stopping_context(core[-1],(anchor.synth_end-anchor.core_end)*tpf,scene.dt))
            controls=project_hand_commands(scene,np.concatenate((core,post)),entry,local)
            report['acquisition_nominal_origin']='local_source_retarget_registered_to_actual_core_entry'
            report['failed_runtime_commands_used_as_reference']=False
        result,_,_=rollout(scene,local,cropped,controls,entry,out/'replay',continue_from_current=True,
                           controller_handoff=True,synthesize_acquisition=acquisition_synthesis)
        if result['success'] and acquisition_synthesis:
            controls=np.load(out/'replay'/'executed_commands.npz')['commands']
        report['replay']=result
        if not result['success']:
            try:
                controls=refine(scene,local,cropped,controls,entry,out/'refinement',args.mpc_candidates,args.mpc_iters,
                    ppo_config=PPOConfig(iterations=args.ppo_updates,max_seconds=240.))
            except RefinementRequired as error:
                controls=error.controls;report['refinement_failure']=str(error)
            controls=project_hand_commands(scene,controls,entry,local)
            reached=reach(anchor.core_start,out/'fresh_local_entry_acquisition')
            result,_,_=rollout(scene,local,cropped,controls,reached,out/'fresh_local',continue_from_current=True,controller_handoff=True)
        if not result['success']:
            from multipath.anchors.floating_search import FloatingResidualEnv
            from multipath.anchors.ppo import LocalPPOSolver
            accepted={};attempts=[]
            def accept_full(candidate):
                index=len(attempts)
                reached=reach(anchor.core_start,out/'ppo_replay_entry_acquisition')
                value,_,_=rollout(scene,local,cropped,candidate,reached,out/'whole_local_replay'/f'candidate_{index:03d}',
                    continue_from_current=True,controller_handoff=True)
                attempts.append(value);report['whole_local_replay_attempts']=attempts
                if value['success']:accepted['validation']=value
                print('WHOLE_LOCAL_REPLAY',index,value['success'],value['failure'],flush=True)
                return value['success']
            def reached_env(*values):
                return FloatingResidualEnv(*values,entry_acquirer=lambda:reach(anchor.core_start,out/'ppo_entry_acquisition'),
                    controller_handoff=True)
            solver=LocalPPOSolver(scene,local,cropped,out/'whole_local_ppo',PPOConfig(iterations=args.ppo_updates,max_seconds=240.),
                env_factory=reached_env,accept_controls=accept_full)
            candidate=solver(controls,entry,0)
            if candidate is not None:controls=candidate;result=accepted['validation']
        report['local_replay']=result
        np.savez_compressed(out/'local_controls.npz',commands=controls)
        if not result['success']:raise RuntimeError('Reached-entry local interaction failed')
        # Execute the exact predecessor controller through the pre-context,
        # then switch to the same local CORE controller as final execution.
        # No state restore occurs at synth_start or core_start.
        reached=reach(anchor.core_start,out/'padded_entry_acquisition',record_from=anchor.synth_start)
        pre_trace=list(scene.records);padded_entry=scene.context_entry
        from multipath.anchors.mpc import WindowEvaluation
        context=WindowEvaluation(scene,anchor,data,padded_entry)
        for state in pre_trace:
            command=np.r_[state['command_wrist'],state['command_hand_q'],state['command_twist']]
            if not context.observe(state['source_frame'],command,measured=state):
                raise RuntimeError(f'Physical pre-context failed: {context.failure}')
        padded_result,_,_=rollout(scene,local,cropped,controls,reached,out/'fresh_padded',
                                 continue_from_current=True,controller_handoff=True,mesh=False)
        core_trace=list(scene.records)
        padded_result.update(rollout_interval=[anchor.synth_start,anchor.synth_end],
            pre_context='actual predecessor controller execution; no boundary reset',
            padded_rollout_complete=padded_result['success'])
        (out/'fresh_padded'/'actual_trace.json').write_text(json.dumps(pre_trace+core_trace))
        (out/'fresh_padded'/'local_result.json').write_text(json.dumps(padded_result,indent=2))
        padded_meta=json.loads((out/'fresh_padded'/'pipeline_result.json').read_text())
        padded_meta.update(actual_duration_s=(anchor.synth_end-anchor.synth_start)/schedule.fps,
                           validation_scope='continuous actual predecessor padding plus local core/post-context')
        (out/'fresh_padded'/'pipeline_result.json').write_text(json.dumps(padded_meta,indent=2))
        report['padded_replay']=padded_result
        if not padded_result['success']:raise RuntimeError('Continuous padded local replay failed')
        # Reexecute the real initial-state cuRobo prefix. The exported CORE
        # starts at its freshly measured endpoint, with no restore at handoff.
        reached=reach(anchor.core_start,out/'core_entry_acquisition');core=controls[:(anchor.core_end-anchor.core_start)*tpf]
        validation,core_entry,core_exit=rollout(scene,anchor,data,core,reached,out/'exported_core',
            core_only=True,continue_from_current=True,controller_handoff=True)
        report['core_replay']=validation
        if hasattr(scene,'last_bridge_exit_hand_clearance'):
            report['core_exit_hand_clearance']=scene.last_bridge_exit_hand_clearance
        if not validation['success']:raise RuntimeError('Fresh physically reached CORE failed')
        valid=dict(padded_result,success=True,padded_rollout_complete=True,core_replay_success=True,
            core_rollout_complete=validation['core_rollout_complete'],
            state_resets_within_padded_rollout=0,state_resets_within_core_replay=0,handoff_verified=True,
            controller_handoff='measured_mass_inertia_gain_response',
            validation_scope='continuous padded replay from physical predecessor plus fresh physical-prefix core replay')
        report['anchor']=save_anchor(out/'anchors',anchor,core_entry,core_exit,core,valid,manifest['provenance'])
        report.update(core_success=report['anchor']['local_rollout_success'],status='reached_core_valid')
    except Exception as error:
        import traceback;traceback.print_exc();report.update(status='failed',error=str(error))
    finally:
        (out/'development_result.json').write_text(json.dumps(report,indent=2));print('REACHED_ANCHOR_RESULT',report.get('status'),report.get('error'),flush=True)
        if app:app.close()


if __name__=='__main__':main()
