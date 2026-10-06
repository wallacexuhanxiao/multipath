"""Refine a local Anchor at the core entry reached by a real cuRobo prefix."""
import argparse,json
from pathlib import Path


def main():
    p=argparse.ArgumentParser()
    for name in ('schedule','proposals','entry-run','context-controls','xhand-urdf','object-cache','floating-config','output-dir'):
        p.add_argument('--'+name,type=Path,required=True)
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
    entry=json.loads((args.entry_run/'cores'/anchor.id/'entry.json').read_text())
    data=dict(np.load(args.proposals/manifest['anchors'][anchor.id],allow_pickle=False))
    tpf=physics_ticks_per_frame(schedule.fps,FloatingScene.dt)
    skip=anchor.core_start-anchor.synth_start
    local=replace(anchor,synth_start=anchor.core_start)
    cropped={k:(v[skip:] if v.ndim and len(v)==anchor.synth_end-anchor.synth_start else v) for k,v in data.items()}
    core=np.load(args.entry_run/'cores'/anchor.id/'commands.npz')['commands']
    context=np.load(args.context_controls)['commands']
    controls=np.concatenate((core,context[(anchor.core_end-anchor.synth_start)*tpf:]))
    report=dict(anchor_id=anchor.id,core_success=False,full_pipeline_success=False,source_provenance=manifest['provenance'],
        synthesis_interval=[anchor.synth_start,anchor.synth_end],optimization_interval=[local.synth_start,local.synth_end],
        pre_context='actual cuRobo predecessor; no reference state initialization',entry_run=str(args.entry_run))
    first=min(schedule.anchors.values(),key=lambda a:a.core_start)
    initial_data=dict(np.load(args.proposals/manifest['anchors'][first.id],allow_pickle=False))
    initial=initial_data['wrist_hand_q'][0,:7].copy();initial[2]+=max(.1,float(np.linalg.norm(np.ptp(initial_data['human_joints'][0],axis=0))))
    app=None;scene=None
    try:
        app=start_simulation(out);scene=FloatingScene(args,manifest,out/'initial',initial,initial_hand=initial_data['wrist_hand_q'][0,7:])
        initial_state=scene.snapshot();controls=project_hand_commands(scene,controls,entry,local)
        def predecessor_commands(window):
            path=(args.entry_run/f'relaxed_{window["start"]}_{window["end"]}.npz' if window['mode']=='RELAXED'
                  else args.entry_run/'cores'/window['anchor']/'commands.npz')
            return np.load(path)['commands']
        def reach(frame,folder):
            scene.begin_trial(folder,initial_state,mesh=False)
            for window in schedule.windows:
                if window['start']>=frame:break
                command=predecessor_commands(window)[:(min(window['end'],frame)-window['start'])*tpf]
                carry=window.get('contact')=='grasped'
                wrench=segment_wrenches(scene,scene.snapshot(),command,transport=carry) if window['start']>0 else np.zeros((len(command),6))
                for k,c in enumerate(command):
                    scene.wrench_bias=wrench[k];scene.step_control(c,window['mode'],window['start']+(k+1)/tpf)
            scene.save();return scene.snapshot()
        entry=reach(anchor.core_start,out/'replay_entry_acquisition')
        result,_,_=rollout(scene,local,cropped,controls,entry,out/'replay',continue_from_current=True,controller_handoff=True)
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
        pre=[]
        for window in schedule.windows:
            lo=max(anchor.synth_start,window['start']);hi=min(anchor.core_start,window['end'])
            if lo<hi:pre.append(predecessor_commands(window)[(lo-window['start'])*tpf:(hi-window['start'])*tpf])
        padded=np.concatenate((*pre,controls))
        padded_entry=reach(anchor.synth_start,out/'padded_entry_acquisition')
        padded_result,_,_=rollout(scene,anchor,data,padded,padded_entry,out/'fresh_padded',continue_from_current=True)
        report['padded_replay']=padded_result
        if not padded_result['success']:raise RuntimeError('Continuous padded local replay failed')
        # Reexecute the real initial-state cuRobo prefix. The exported CORE
        # starts at its freshly measured endpoint, with no restore at handoff.
        reached=reach(anchor.core_start,out/'core_entry_acquisition');core=controls[:(anchor.core_end-anchor.core_start)*tpf]
        validation,core_entry,core_exit=rollout(scene,anchor,data,core,reached,out/'exported_core',
            core_only=True,continue_from_current=True,controller_handoff=True)
        report['core_replay']=validation
        if not validation['success']:raise RuntimeError('Fresh physically reached CORE failed')
        valid=dict(padded_result,success=True,padded_rollout_complete=True,core_replay_success=True,
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
