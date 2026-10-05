"""Initial floating PhysX -> cuRobo predecessor -> local padded/core synthesis."""
import argparse,hashlib,json,os,shutil,time
from pathlib import Path


def main():
    p=argparse.ArgumentParser()
    for name in ('proposals','schedule','xhand-urdf','object-cache','output-dir','floating-config'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--replay-only',action='store_true')
    p.add_argument('--mpc-only',action='store_true',help='Short physical attribution run; default still escalates to residual PPO.')
    p.add_argument('--mpc-candidates',type=int,default=64);p.add_argument('--mpc-iters',type=int,default=3)
    p.add_argument('--ppo-updates',type=int,default=60)
    p.add_argument('--anchor-id',help='Develop this oracle Anchor; later entries require a real predecessor snapshot.')
    p.add_argument('--entry-snapshot',type=Path,help='Development only: physically acquired state at synth_start.')
    p.add_argument('--predecessor-run',type=Path,help='Reuse a passed cuRobo bridge only when its actual initial state and source match.')
    args=p.parse_args();out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    if args.entry_snapshot and args.predecessor_run:p.error('Choose a snapshot or a validated predecessor bridge')
    from multipath.oracle_schedule import OracleSchedule
    schedule=OracleSchedule.load(args.schedule);first=min(schedule.anchors.values(),key=lambda a:a.core_start)
    anchor=schedule.anchors[args.anchor_id] if args.anchor_id else first
    if anchor.id!=first.id and not args.entry_snapshot:
        raise ValueError('Non-initial Anchor development requires a physically acquired predecessor snapshot')
    manifest=json.loads((args.proposals/'manifest.json').read_text())
    if manifest['provenance']['schedule_sha256']!=hashlib.sha256(args.schedule.read_bytes()).hexdigest():raise ValueError('Schedule provenance mismatch')
    if manifest['num_frames']!=schedule.num_frames or manifest['fps']!=schedule.fps:
        raise ValueError('Proposal/source clocks differ')
    report={'embodiment':'floating_xhand','m2_success':False,'full_pipeline_success':False,'source_provenance':manifest['provenance'],
        'pbs_job_id':os.environ.get('PBS_JOBID'),'source_duration_s':schedule.num_frames/schedule.fps,'vlm_used':False,
        'complete_robot_reference_required':False,'source_time_stretch':False,'development_only':True,
        'anchor_id':anchor.id,'core_interval':[anchor.core_start,anchor.core_end],
        'synthesis_interval':[anchor.synth_start,anchor.synth_end],
        'note':'Bridge to synth_start is a development predecessor; final bridge to core entry still requires verification.'}
    from multipath.execution.startup import watch_dependency_imports
    watch_dependency_imports(out)
    import multipath
    shutil.copytree(Path(multipath.__file__).parent,out/'implementation'/'multipath',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copy2(__file__,out/'implementation'/Path(__file__).name)
    app=None;scene=None;started=time.perf_counter()
    try:
        import numpy as np
        from multipath.execution.startup import start_simulation
        from multipath.execution.floating_scene import FloatingScene
        from multipath.planning.floating import FloatingPlanner,pose_coordinates
        from multipath.anchors.floating import local_controls,rollout,validate_commands,validate_local_data
        from multipath.planning.timing import physics_ticks_per_frame
        from multipath.anchors.artifact import save_anchor
        data=dict(np.load(args.proposals/manifest['anchors'][anchor.id],allow_pickle=False))
        validate_local_data(anchor,data);tpf=physics_ticks_per_frame(schedule.fps,FloatingScene.dt)
        initial_data=data if anchor.id==first.id else dict(np.load(args.proposals/manifest['anchors'][first.id],allow_pickle=False))
        initial=initial_data['wrist_hand_q'][0,:7].copy()
        # A collision-separated nominal ready pose, initialized before source t=0.
        extent=float(np.linalg.norm(np.ptp(initial_data['human_joints'][0],axis=0)))
        initial[2]+=max(.1,extent)
        app=start_simulation(out);scene=FloatingScene(args,manifest,out/'predecessor',initial,initial_hand=initial_data['wrist_hand_q'][0,7:])
        report['initial_hand_origin']='first_local_synthesis_context_pregrasp; free_initial_state'
        print('FLOATING_SCENE_READY',time.perf_counter()-started,flush=True)
        if args.entry_snapshot:
            acquired=json.loads(args.entry_snapshot.read_text())
            if acquired.get('source_provenance')!=manifest['provenance']:
                raise ValueError('Snapshot source provenance mismatch')
            if not np.isclose(acquired['time_s'],anchor.synth_start/schedule.fps,rtol=0,atol=1e-8):
                raise ValueError('Snapshot must be acquired at this Anchor synth_start')
            acquired=dict(acquired,entry_origin='physically_acquired_snapshot')
            scene.restore_development(acquired);entry=scene.snapshot()
            report['predecessor_snapshot']=str(args.entry_snapshot)
            (out/'acquired_snapshot.json').write_text(json.dumps(entry,indent=2))
        else:
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
            ticks=anchor.synth_start*tpf
            if args.predecessor_run:
                from multipath.anchors.compatibility import boundary_errors
                from multipath.planning.timing import fixed_budget_path
                prior=json.loads((args.predecessor_run/'development_result.json').read_text())
                trace=json.loads((args.predecessor_run/'predecessor'/'actual_trace.json').read_text())
                if (prior['source_provenance']!=manifest['provenance'] or not prior.get('predecessor_transition',{}).get('success')
                    or prior['predecessor_plan']['planner']!='cuRobo_virtual_6DOF'):
                    raise ValueError('Predecessor run is not a passed cuRobo bridge for this source')
                errors=boundary_errors(scene.state(),trace[0])
                if (not errors['scene_match'] or not errors['embodiment_match'] or not errors['contact_match'] or
                    any(value>1e-5 for key,value in errors.items() if isinstance(value,float))):
                    raise ValueError(f'Cached bridge initial state differs: {errors}')
                commands=np.load(args.predecessor_run/'predecessor_commands.npz',allow_pickle=False)['commands']
                if commands.shape!=(ticks,25) or not validate_commands(scene,commands,scene.state())['feasible']:
                    raise ValueError('Cached bridge commands do not match this source budget')
                q0=pose_coordinates(scene.state()['wrist_xyz_wxyz']);path=[q0]
                for pose in commands[:,:7]:path.append(pose_coordinates(pose,path[-1]))
                path=np.asarray(path)
                twist=np.asarray(scene.state()['wrist_twist']);eps=1e-4
                from scipy.spatial.transform import Rotation
                advanced=np.asarray(scene.state()['wrist_xyz_wxyz']).copy();advanced[:3]+=eps*twist[:3]
                advanced[3:]=(Rotation.from_rotvec(eps*twist[3:])*Rotation.from_quat(advanced[[4,5,6,3]])).as_quat()[[3,0,1,2]]
                dq0=(pose_coordinates(advanced,q0)-q0)/eps
                _,timing=fixed_budget_path(path,scene.dt,ticks,scene.dt,np.asarray(scene.cfg['planning']['velocity_limits']),
                                           np.asarray(scene.cfg['planning']['acceleration_limits']),initial_velocity=dq0)
                for configuration in path:
                    if not planner.valid(configuration):raise ValueError('Cached bridge fails current collision model')
                timing.update(planner='cuRobo_virtual_6DOF',cached_validated_predecessor=str(args.predecessor_run),
                              source_time_stretch=False,reference_tracking=False,initial_state_errors=errors)
            elif ticks:
                commands,timing=planner.plan(target,ticks)
            else:
                commands=np.empty((0,25));timing={'budget_s':0.,'planner':'none_initial_anchor'}
            report['predecessor_plan']=timing
            print('FLOATING_PLAN_READY',timing,flush=True)
            np.savez_compressed(out/'predecessor_commands.npz',commands=commands)
            for i,command in enumerate(commands):
                frame=(i+1)*scene.dt*schedule.fps;scene.step_control(command,'RELAXED',frame)
                if (i+1)%tpf==0:
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
        validation,core_entry,core_exit=rollout(scene,anchor,data,controls,entry,out/'replay',collect_tracking_failures=True)
        report['padded_replay']=validation
        if not validation['success'] and not args.replay_only:
            from multipath.anchors.floating_search import refine
            from multipath.anchors.ppo import PPOConfig
            controls=refine(scene,anchor,data,controls,entry,out/'refinement',args.mpc_candidates,args.mpc_iters,
                            use_ppo=not args.mpc_only,ppo_config=PPOConfig(iterations=args.ppo_updates))
            np.savez_compressed(out/'refined_controls.npz',commands=controls)
            validation,core_entry,core_exit=rollout(scene,anchor,data,controls,entry,out/'refined_replay')
            report['refined_padded_replay']=validation
        if not validation['success']:raise RuntimeError('Floating padded interaction failed')
        core=controls[(anchor.core_start-anchor.synth_start)*tpf:(anchor.core_end-anchor.synth_start)*tpf]
        padded_exit=core_exit
        core_result,_,core_exit=rollout(scene,anchor,data,core,core_entry,out/'exported_core',mode='EXPORTED_CORE',core_only=True)
        report['core_replay']=core_result
        if not core_result['success'] or core_exit is None:raise RuntimeError('Exported core physical replay failed')
        from multipath.anchors.compatibility import boundary_errors
        report['core_exit_repeatability']=boundary_errors(core_exit,padded_exit)
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
