"""Resynthesize one Anchor at a real alternative entry, then export its core.

Use only when a completed interaction has an infeasible outgoing bridge.
Small physical shooting trials refine its exit tangent. Both the padded
interaction and a fresh exported-core execution must pass. No reference
state, source-time extension or runtime reset is allowed.
"""
import argparse,json,shutil
from pathlib import Path


def main():
    p=argparse.ArgumentParser()
    for name in ('schedule','proposals','entry-run','baseline-run','bridge-bank','validated-anchors',
                 'xhand-urdf','object-cache','floating-config','output-dir'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--anchor-id',required=True)
    p.add_argument('--exclude-actual-run',type=Path,action='append',default=[])
    p.add_argument('--candidates',type=int,default=8)
    args=p.parse_args();out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    from multipath.execution.startup import watch_dependency_imports,start_simulation
    watch_dependency_imports(out)
    import numpy as np
    from multipath.oracle_schedule import OracleSchedule
    from multipath.execution.floating_scene import FloatingScene
    from multipath.execution.wrench_handoff import segment_wrenches,recorded_control_segment
    from multipath.anchors.artifact import load_validated_anchor,save_anchor
    from multipath.anchors.floating import rebase_grasp_commands,validate_commands
    from multipath.anchors.floating_search import carry_preload_candidates
    from multipath.anchors.mpc import WindowEvaluation
    from multipath.anchors.bridge_boundary import nearest_entry_velocity,terminal_velocity_residual
    from multipath.planning.floating import FloatingPlanner
    from multipath.planning.floating_diversity import interval_exclusions,plan_detour,actual_feasible_exclusions
    from multipath.planning.timing import boundary_fitted_path
    from multipath.planning.floating import pose_coordinates
    from multipath.planning.floating_diversity import pose_velocity
    from multipath.execution.interaction_boundary import grasp_conditioned_boundary,check_interaction_boundary
    from multipath.validation.physics import check_retention
    schedule=OracleSchedule.load(args.schedule);anchor=schedule.anchors[args.anchor_id]
    manifest=json.loads((args.proposals/'manifest.json').read_text())
    prior=json.loads((args.entry_run/'pipeline_result.json').read_text())
    if prior['source_provenance']!=manifest['provenance'] or prior.get('state_resets_between_segments')!=0:
        raise ValueError('Local development requires an uninterrupted physical predecessor')
    for w in schedule.windows:
        if w['end']>anchor.core_start:break
        s=next((s for s in prior['segments'] if s['start']==w['start']),None)
        if not s or not s.get('success'):raise ValueError('Anchor entry was not physically reached')
    following=next(w for w in schedule.windows if w['start']==anchor.core_end)
    if following['mode']!='RELAXED' or following['end']<anchor.synth_end:
        raise ValueError('Post-context must lie inside the outgoing relaxed bridge')
    later=next(w for w in schedule.windows if w['start']==following['end'])
    goal=load_validated_anchor(args.validated_anchors,schedule.anchors[later['anchor']],provenance=manifest['provenance'])['entry']
    bank=json.loads((args.bridge_bank/'bridge_bank.json').read_text())
    item=next(i for i in bank['intervals'] if i['window']==[following['start'],following['end']])
    selected=next(s for s in item['solutions'] if s['index']==prior['bridge_solution'])
    bridge=json.loads((args.bridge_bank/selected['artifact']).with_suffix('.json').read_text())
    baseline_trace=json.loads((args.baseline_run/'actual_trace.json').read_text())
    baseline_meta=json.loads((args.baseline_run/f'relaxed_{following["start"]}_{following["end"]}.json').read_text())
    baseline=np.load(args.baseline_run/f'relaxed_{following["start"]}_{following["end"]}.npz')['commands']
    paths=[dict(id='baseline',points=np.asarray([r['wrist_xyz_wxyz'][:3] for r in baseline_trace
        if following['start']<=r['source_frame']<=following['end']]),
        command_points=np.vstack((baseline_meta['entry']['wrist_xyz_wxyz'][:3],baseline[:,:3])))]
    for directory in args.exclude_actual_run:
        report=json.loads((directory/'pipeline_result.json').read_text())
        if not report.get('diverse_solution_success') or report['source_provenance']!=manifest['provenance']:
            raise ValueError('Exclusions require a successful actual predecessor trajectory')
        data=np.load(directory/f'relaxed_{following["start"]}_{following["end"]}.npz')
        trace=json.loads((directory/'actual_trace.json').read_text())
        entry=next(r for r in trace if np.isclose(r['source_frame'],following['start']))
        paths.append(dict(id=str(directory),points=data['actual_wrist_path'],
            command_points=np.vstack((entry['wrist_xyz_wxyz'][:3],data['commands'][:,:3]))))
    local=dict(np.load(args.proposals/manifest['anchors'][anchor.id]))
    core=np.load(args.entry_run/'anchors'/anchor.id/'executed_core.npz')['commands']
    program_path=args.entry_run/'executed_controls.npz'
    predecessor_program=dict(np.load(program_path)) if program_path.exists() else None
    initial=np.asarray(prior['initial_state']['command_wrist'])
    initial_hand=np.asarray(prior['initial_state']['command_hand_q'])
    report=dict(status='initializing',anchor_id=anchor.id,success=False,source_provenance=manifest['provenance'],
        entry_run=str(args.entry_run),core_interval=[anchor.core_start,anchor.core_end],
        synthesis_interval=[anchor.synth_start,anchor.synth_end],trials=[],source_time_stretch=False)
    app=None;scene=None;delta=np.zeros(6);accepted=None
    joint_proposals=[];joint_velocity_delta=None
    try:
        app=start_simulation(out)
        def acquire(folder):
            nonlocal scene
            if scene is not None:
                scene.world.stop();scene.world.clear()
                from isaacsim.core.api import World
                from isaacsim.core.utils.stage import create_new_stage
                World.clear_instance();create_new_stage()
            scene=FloatingScene(args,manifest,folder,initial,initial_hand=initial_hand)
            padded_entry=None
            for w in schedule.windows:
                if w['start']>=anchor.core_start:break
                path=(args.entry_run/f'relaxed_{w["start"]}_{w["end"]}.npz' if w['mode']=='RELAXED'
                    else args.entry_run/'anchors'/w['anchor']/'executed_core.npz')
                commands=np.load(path)['commands'];s=next(s for s in prior['segments'] if s['start']==w['start'])
                old=np.load(args.baseline_run/f'relaxed_{w["start"]}_{w["end"]}.npz')['commands'] if s.get('delta_dynamics_feedforward') else None
                wrench=segment_wrenches(scene,scene.snapshot(),commands,transport=w.get('contact')=='grasped' or s.get('carried_dynamics_feedforward',False),
                    baseline_commands=old) if w['start']>0 or old is not None else np.zeros((len(commands),6))
                if predecessor_program is not None:
                    commands,wrench=recorded_control_segment(predecessor_program,w['start'],w['end'],scene.tpf)
                for j,c in enumerate(commands):
                    scene.wrench_bias=wrench[j];frame=w['start']+(j+1)/scene.tpf;scene.step_control(c,w['mode'],frame)
                    if np.isclose(frame,anchor.synth_start):padded_entry=scene.snapshot()
            if padded_entry is None:raise ValueError('No real padded entry was acquired')
            return padded_entry,scene.snapshot()
        def execute_core(candidate,wrench=None):
            entry=scene.snapshot();objective=WindowEvaluation(scene,anchor,local,entry)
            if not validate_commands(scene,candidate,entry)['feasible']:raise RuntimeError('Candidate finger limits')
            if not objective.observe(anchor.core_start,candidate[0]):raise RuntimeError(str(objective.failure))
            changed_at=None;baseline_drive=None;baseline_wrench=None
            if wrench is None and predecessor_program is not None:
                baseline_drive,baseline_wrench=recorded_control_segment(
                    predecessor_program,anchor.core_start,anchor.core_end,scene.tpf)
                changed=np.flatnonzero(np.any(candidate!=baseline_drive,axis=1))
                changed_at=int(changed[0]) if len(changed) else None
                wrench=baseline_wrench.copy()
            elif wrench is None:
                wrench=segment_wrenches(scene,entry,candidate,transport=anchor.entry_contact==anchor.exit_contact=='grasped')
            for j,c in enumerate(candidate):
                if changed_at is not None and j==changed_at:
                    # Preserve the physically acquired preload and unchanged
                    # prefix. Add only the model response to this residual,
                    # evaluated at its actually reached onset state.
                    actual=scene.snapshot();carry=actual['contact']['mode']=='grasped'
                    new=segment_wrenches(scene,actual,candidate[j:],transport=carry)
                    old=segment_wrenches(scene,actual,baseline_drive[j:],transport=carry)
                    wrench[j:]=baseline_wrench[j:]+new-old
                scene.wrench_bias=wrench[j];frame=anchor.core_start+(j+1)/scene.tpf;scene.step_control(c,'ANCHOR',frame)
                if (j+1)%scene.tpf==0 and not objective.observe(frame,c):raise RuntimeError(str(objective.failure))
            return entry,scene.snapshot(),objective,wrench
        def outgoing(entry,folder,physical):
            commands=rebase_grasp_commands(baseline,baseline_meta['entry'],entry,scene.dt)
            q0=pose_coordinates(entry['wrist_xyz_wxyz']);q1=pose_coordinates(commands[-1,:7],q0)
            dq0=pose_velocity(entry['wrist_xyz_wxyz'],entry['wrist_twist'],q0)
            dq1=pose_velocity(commands[-1,:7],commands[-1,19:25],q1)
            boundary_fitted_path(np.vstack((q0,q1)),len(commands),scene.dt,
                np.asarray(scene.cfg['planning']['velocity_limits']),np.asarray(scene.cfg['planning']['acceleration_limits']),dq0,dq1)
            if not physical:return commands,None,None
            exclusion=interval_exclusions(paths,item['window'],len(commands)*scene.dt,scene.cfg['planning'],
                bridge['exclusions']['size_multiplier'],bridge['exclusions'].get('free_diversity_budget_s'))
            planner=FloatingPlanner(scene,True,folder/'planner');errors=[]
            target=grasp_conditioned_boundary(goal,entry)
            for seed in [bridge['planning']['seed_index']]+[i for i in range(8) if i!=bridge['planning']['seed_index']]:
                try:
                    result,timing=plan_detour(planner,entry,commands[-1],len(commands),exclusion,target,seed,
                        baseline_commands=commands,regularize=bridge['planning'].get('trajectory_cost','').startswith('normalized'))
                    break
                except RuntimeError as error:errors.append(str(error))
            else:raise RuntimeError(str(errors))
            held=np.asarray(entry['hand_object_transform']);wrench=segment_wrenches(scene,entry,result,transport=True)
            actual=[entry['wrist_xyz_wxyz'][:3]]
            for j,c in enumerate(result):
                scene.wrench_bias=wrench[j];scene.step_control(c,'RELAXED',following['start']+(j+1)/scene.tpf)
                check_retention(scene.state(),held);actual.append(scene.state()['wrist_xyz_wxyz'][:3])
                if min(planner.measured_clearance().values())<0 and planner.measured_mesh_collisions():
                    raise RuntimeError('Outgoing physical mesh collision')
            exclusion,metrics=actual_feasible_exclusions(exclusion,actual,len(commands)*scene.dt,scene.cfg['planning'])
            transition=check_interaction_boundary(scene,target,following['contact'])
            if not metrics['actual_wrist_exclusion_pass'] or not metrics['smooth_arc_pass'] or not transition['success']:
                raise RuntimeError('Outgoing actual boundary/exclusion failed')
            np.savez_compressed(folder/'outgoing_bridge.npz',commands=result,actual_wrist_path=actual)
            return result,timing,dict(exclusions=exclusion,diversity=metrics,transition=transition)
        for index in range(args.candidates):
            folder=out/f'candidate_{index:02d}';padded_entry,entry=acquire(folder)
            duration=(anchor.core_start-anchor.synth_start+anchor.synth_end-anchor.core_end)/schedule.fps
            candidate=terminal_velocity_residual(core,delta,duration,scene.dt)
            trial=dict(index=index,terminal_coordinate_velocity_residual=delta.tolist(),success=False)
            joint_index=index-max(2,args.candidates//2)
            if 0<=joint_index<len(joint_proposals):
                preload,metadata=joint_proposals[joint_index]
                candidate=terminal_velocity_residual(preload,joint_velocity_delta,duration,scene.dt)
                trial.update(terminal_coordinate_velocity_residual=joint_velocity_delta.tolist(),
                    finger_preload=metadata,proposal_family='joint_wrist_and_finger_physical_MPC')
            report['trials'].append(trial);exit_state=None
            try:
                native=None
                if predecessor_program is not None and np.array_equal(candidate,core):
                    _,native=recorded_control_segment(predecessor_program,anchor.core_start,anchor.core_end,scene.tpf)
                entry,exit_state,objective,wrench=execute_core(candidate,native)
                _,timing,validation=outgoing(exit_state,folder,True)
                scene.save();trial.update(success=True,padded_local_rollout_success=True,
                    measured_exit=exit_state,outgoing_planning=timing,outgoing_validation=validation)
                accepted=candidate;accepted_wrenches=wrench.copy();break
            except RuntimeError as error:
                trial['error']=str(error);scene.save()
                if exit_state is not None:
                    command=rebase_grasp_commands(baseline,baseline_meta['entry'],exit_state,scene.dt)[-1]
                    correction=nearest_entry_velocity(exit_state,command,len(baseline),scene.dt,scene.cfg['planning'])
                    # Aim inside the feasible envelope. This changes a proposal
                    # velocity only; the original limits still decide acceptance.
                    correction=np.where(abs(correction)>1e-5,correction+
                        np.sign(correction)*.02*np.asarray(scene.cfg['planning']['velocity_limits']),0.)
                    if not joint_proposals and index==0:
                        # Wrist-only exit shaping can spoil an otherwise real
                        # grasp. Reuse the existing contact-directed finger
                        # proposals in the same trial budget, inside the Anchor.
                        # Fingers remain frozen during the outgoing bridge.
                        joint_proposals=list(carry_preload_candidates(scene,anchor,core,entry,exit_state))[-4:]
                        joint_velocity_delta=2*correction
                    delta+=2*correction
                else:delta*=.5
            (out/'development_result.json').write_text(json.dumps(report,indent=2))
            print('ANCHOR_BOUNDARY_TRIAL',index,trial.get('error'),delta.tolist(),flush=True)
        if accepted is None:raise RuntimeError('No interaction-valid boundary shooting candidate')
        padded_entry,reached=acquire(out/'fresh_exported_core')
        core_entry,core_exit,_,_=execute_core(accepted,accepted_wrenches);outgoing(core_exit,scene.out,False);scene.save()
        validation=dict(success=True,padded_rollout_complete=True,core_replay_success=True,
            source_fps=schedule.fps,control_dt=scene.dt,
            embodiment=scene.embodiment,command_format='wrist_xyz_wxyz_hand12_twist6',
            core_rollout_interval=[anchor.core_start,anchor.core_end],
            padded_rollout_interval=[anchor.synth_start,anchor.synth_end],
            state_resets_within_padded_rollout=0,state_resets_within_core_replay=0,handoff_verified=True,
            controller_handoff='measured_mass_inertia_gain_response',
            validation_scope='fresh initial-state prefix + local padded interaction and outgoing bridge; fresh exported-core replay',
            boundary_refinement='actual_PhysX_terminal_velocity_shooting',padded_validation_trial=report['trials'][-1]['index'],
            physical_trace=str(scene.out/'actual_trace.json'))
        artifact=save_anchor(out/'anchors',anchor,core_entry,core_exit,accepted,validation,manifest['provenance'],
            controller_wrenches=accepted_wrenches)
        for other in schedule.anchors.values():
            if other.id!=anchor.id:shutil.copytree(args.validated_anchors/other.id,out/'anchors'/other.id,dirs_exist_ok=True)
        report.update(status='boundary_refined',success=artifact['local_rollout_success'],anchor=artifact)
    except Exception as error:
        import traceback;traceback.print_exc();report.update(status='failed',error=str(error))
    finally:
        (out/'development_result.json').write_text(json.dumps(report,indent=2))
        print('ANCHOR_BOUNDARY_RESULT',report['status'],report.get('error'),flush=True)
        if app:app.close()
    if not report['success']:raise SystemExit(1)


if __name__=='__main__':main()
