"""Initial floating PhysX -> cuRobo predecessor -> local padded/core synthesis."""
import argparse,hashlib,json,os,shutil,time
from pathlib import Path


def main():
    p=argparse.ArgumentParser()
    for name in ('proposals','schedule','xhand-urdf','object-cache','output-dir','floating-config'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--replay-only',action='store_true')
    p.add_argument('--mpc-candidates',type=int,default=64);p.add_argument('--mpc-iters',type=int,default=3)
    args=p.parse_args();out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    from multipath.oracle_schedule import OracleSchedule
    schedule=OracleSchedule.load(args.schedule);anchor=min(schedule.anchors.values(),key=lambda a:a.core_start)
    manifest=json.loads((args.proposals/'manifest.json').read_text())
    if manifest['provenance']['schedule_sha256']!=hashlib.sha256(args.schedule.read_bytes()).hexdigest():raise ValueError('Schedule provenance mismatch')
    report={'embodiment':'floating_xhand','m2_success':False,'full_pipeline_success':False,'source_provenance':manifest['provenance'],
        'pbs_job_id':os.environ.get('PBS_JOBID'),'source_duration_s':schedule.num_frames/schedule.fps,'vlm_used':False,
        'complete_robot_reference_required':False,'source_time_stretch':False,'development_only':True,
        'note':'Bridge to synth_start is a development predecessor; final bridge to core entry still requires verification.'}
    import multipath
    shutil.copytree(Path(multipath.__file__).parent,out/'implementation'/'multipath',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copy2(__file__,out/'implementation'/Path(__file__).name)
    app=None;scene=None;started=time.perf_counter()
    try:
        import numpy as np
        from multipath.execution.startup import start_simulation
        from multipath.execution.floating_scene import FloatingScene
        from multipath.planning.floating import FloatingPlanner,pose_coordinates
        from multipath.anchors.floating import local_controls,rollout,validate_commands
        from multipath.anchors.artifact import save_anchor
        data=dict(np.load(args.proposals/manifest['anchors'][anchor.id],allow_pickle=False))
        initial=data['wrist_hand_q'][0,:7].copy()
        # A collision-separated nominal ready pose, initialized before source t=0.
        extent=float(np.linalg.norm(np.ptp(data['human_joints'][0],axis=0)))
        initial[2]+=max(.1,extent)
        app=start_simulation(out);scene=FloatingScene(args,manifest,out/'predecessor',initial,initial_hand=data['wrist_hand_q'][0,7:])
        report['initial_hand_origin']='first_local_synthesis_context_pregrasp; free_initial_state'
        print('FLOATING_SCENE_READY',time.perf_counter()-started,flush=True)
        planner=FloatingPlanner(scene,False,out/'planner')
        target=data['wrist_hand_q'][0,:7].copy()
        entry_candidates=[];reference_target=target.copy()
        for height in np.linspace(0,max(extent,float(np.linalg.norm(scene.meshes['tool'].extents))),9):
            target=reference_target.copy();target[2]+=height
            feasible=planner.valid(pose_coordinates(target))
            entry_candidates.append({'offset_z_m':float(height),'collision_free':feasible})
            if feasible:break
        else:raise RuntimeError('No collision-free floating synthesis entry within local geometry extent')
        report['entry_geometry_search']=entry_candidates
        ticks=round(anchor.synth_start/schedule.fps/scene.dt)
        commands,timing=planner.plan(target,ticks);report['predecessor_plan']=timing
        print('FLOATING_PLAN_READY',timing,flush=True)
        np.savez_compressed(out/'predecessor_commands.npz',commands=commands)
        for i,command in enumerate(commands):
            frame=(i+1)*scene.dt*schedule.fps;scene.step_control(command,'RELAXED',frame)
            if (i+1)%round(1/(scene.dt*schedule.fps))==0:
                if not planner.measured_valid():raise RuntimeError('Measured floating bridge collision')
        transition=scene.check_transition(target,anchor.entry_contact);report['predecessor_transition']=transition
        scene.save()
        if not transition['success']:raise RuntimeError('Floating bridge handoff failed')
        scene.lineage={'trace':str(scene.out/'actual_trace.json'),'trace_sha256':hashlib.sha256((scene.out/'actual_trace.json').read_bytes()).hexdigest(),
            'source_frame':anchor.synth_start,'planner':'cuRobo','predecessor_success':True}
        entry=scene.snapshot();(out/'acquired_snapshot.json').write_text(json.dumps(entry,indent=2))
        print('FLOATING_PREDECESSOR_PASSED',flush=True)
        controls=local_controls(scene,anchor,data,entry);check=validate_commands(scene,controls,entry);report['command_precheck']=check
        if not check['feasible']:raise RuntimeError('Floating local command precheck failed')
        np.savez_compressed(out/'local_controls.npz',commands=controls)
        validation,core_entry,core_exit=rollout(scene,anchor,data,controls,entry,out/'replay')
        report['padded_replay']=validation
        if not validation['success'] and not args.replay_only:
            from multipath.anchors.floating_search import refine
            controls=refine(scene,anchor,data,controls,entry,out/'refinement',args.mpc_candidates,args.mpc_iters)
            validation,core_entry,core_exit=rollout(scene,anchor,data,controls,entry,out/'refined_replay')
            report['refined_padded_replay']=validation
        if not validation['success']:raise RuntimeError('Floating padded interaction failed')
        tpf=round(1/(scene.dt*schedule.fps));core=controls[(anchor.core_start-anchor.synth_start)*tpf:(anchor.core_end-anchor.synth_start)*tpf]
        core_result,_,_=rollout(scene,anchor,data,core,core_entry,out/'exported_core',mode='EXPORTED_CORE',core_only=True)
        report['core_replay']=core_result
        valid=dict(validation,padded_rollout_complete=validation['success'],core_replay_success=core_result['success'],
            state_resets_within_padded_rollout=validation['state_resets_within_rollout'],state_resets_within_core_replay=core_result['state_resets_within_rollout'],
            handoff_verified=core_result['success'])
        report['anchor']=save_anchor(out/'anchors',anchor,core_entry,core_exit,core,valid,manifest['provenance'])
        report['m2_success']=report['anchor']['local_rollout_success'];report['status']='anchor_valid' if report['m2_success'] else 'core_failed'
    except Exception as error:
        import traceback
        traceback.print_exc();report.update(status='failed',error=str(error))
    finally:
        report['elapsed_s']=time.perf_counter()-started
        (out/'development_result.json').write_text(json.dumps(report,indent=2))
        print('FLOATING_RESULT',report.get('status'),report.get('error'),flush=True)
        if scene:scene.save()
        if app:app.close()


if __name__=='__main__':main()
