"""A real acquired exit -> cuRobo bridge -> next core, without handoff resets.

Local snapshot development only. Success here is not a complete initial-state
rollout; the final oracle executor must still reproduce the whole chain.
"""
import argparse,json,os,time
from pathlib import Path


def main():
    p=argparse.ArgumentParser()
    for name in ('schedule','proposals','from-artifacts','to-artifacts','floating-config','xhand-urdf','object-cache','output-dir'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--from-anchor',required=True);p.add_argument('--to-anchor',required=True)
    p.add_argument('--preflight-only',action='store_true')
    p.add_argument('--source-entry-target',action='store_true',help='Development acquisition of a new physical entry from the next oracle source boundary; the old core is a seed, not proof of compatibility.')
    args=p.parse_args();out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    from multipath.oracle_schedule import OracleSchedule
    from multipath.anchors.artifact import load_validated_anchor
    schedule=OracleSchedule.load(args.schedule);manifest=json.loads((args.proposals/'manifest.json').read_text())
    left=schedule.anchors[args.from_anchor];right=schedule.anchors[args.to_anchor]
    window=next(w for w in schedule.windows if w['start']==left.core_end and w['end']==right.core_start and w['mode']=='RELAXED')
    dt=1/120
    a=load_validated_anchor(args.from_artifacts,left,provenance=manifest['provenance'],fps=schedule.fps,dt=dt)
    b=load_validated_anchor(args.to_artifacts,right,provenance=manifest['provenance'],fps=schedule.fps,dt=dt)
    config=json.loads(args.floating_config.read_text())
    goal=b['entry'];source_target=None
    if args.source_entry_target:
        import numpy as np
        from scipy.spatial.transform import Rotation
        local=np.load(args.proposals/manifest['anchors'][right.id],allow_pickle=False)
        index=right.core_start-right.synth_start;source_target=local['tool_pose'][index]
        wrists=local['tool_pose']@np.linalg.inv(np.asarray(a['exit']['hand_object_transform']))
        lo=max(0,index-1);hi=min(len(wrists)-1,index+1)
        pose=np.r_[wrists[index,:3,3],Rotation.from_matrix(wrists[index,:3,:3]).as_quat()[[3,0,1,2]]]
        twist=np.r_[(wrists[hi,:3,3]-wrists[lo,:3,3])*schedule.fps/(hi-lo),
                    (Rotation.from_matrix(wrists[hi,:3,:3])*Rotation.from_matrix(wrists[lo,:3,:3]).inv()).as_rotvec()*schedule.fps/(hi-lo)]
        goal=dict(wrist_xyz_wxyz=pose.tolist(),wrist_twist=twist.tolist(),
                  hand_q=a['exit']['hand_q'],hand_dq=a['exit']['hand_dq'],
                  command_hand_q=a['exit']['command_hand_q'],contact={'mode':right.entry_contact},
                  scene_id=a['exit']['scene_id'],physically_acquired_state=False,
                  state_origin='oracle_boundary_target_only')
    from multipath.planning.bridge import bridge_precheck
    check=bridge_precheck(a['exit'],goal,(window['end']-window['start'])/schedule.fps,config['planning'])
    report=dict(status='preflight_passed_physics_pending' if check['feasible'] else 'boundary_rejected',development_only=True,full_pipeline_success=False,
        bridge_success=False,next_core_success=False,source_provenance=manifest['provenance'],
        pbs_job_id=os.environ.get('PBS_JOBID'),from_anchor=left.id,to_anchor=right.id,
        source_interval=[window['start'],window['end']],boundary_precheck=check,
        source_time_stretch=False,vlm_used=False,complete_robot_reference_required=False,
        goal_kind='oracle_boundary_target_only' if args.source_entry_target else 'physically_validated_canonical_entry',
        source_boundary_frame=right.core_start)
    import shutil,multipath
    implementation=out/'implementation';implementation.mkdir(exist_ok=True)
    shutil.copy2(__file__,implementation/Path(__file__).name)
    shutil.copytree(Path(multipath.__file__).parent,implementation/'multipath',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
    app=None;scene=None;started=time.perf_counter()
    try:
        if args.preflight_only or not check['feasible']:return
        from multipath.execution.startup import watch_dependency_imports,start_simulation
        watch_dependency_imports(out)
        import numpy as np
        from multipath.execution.floating_scene import FloatingScene
        from multipath.planning.floating import FloatingPlanner
        from multipath.validation.physics import check_retention
        from multipath.anchors.floating import rollout
        app=start_simulation(out)
        first=min(schedule.anchors.values(),key=lambda x:x.core_start)
        data=np.load(args.proposals/manifest['anchors'][first.id],allow_pickle=False)
        initial=data['wrist_hand_q'][0,:7].copy();initial[2]+=max(.1,float(np.linalg.norm(np.ptp(data['human_joints'][0],axis=0))))
        scene=FloatingScene(args,manifest,out/'initialization',initial,initial_hand=data['wrist_hand_q'][0,7:])
        scene.restore_development(a['exit']);scene.begin_recording(out/'bridge',mesh=True)
        scene.capture('INITIAL',window['start']);resets=scene.restore_count
        start=scene.snapshot();carry=window['contact']=='grasped';held=np.asarray(start['hand_object_transform'])
        if carry:check_retention(start,held)
        planner=FloatingPlanner(scene,carry,out/'planner')
        commands,timing=planner.plan(np.asarray(goal['wrist_xyz_wxyz']),round(check['budget_s']/dt),
                                   np.asarray(goal['wrist_twist']),goal_state=goal)
        report['planning']=timing;np.savez_compressed(out/'bridge_commands.npz',commands=commands)
        for k,c in enumerate(commands):
            frame=window['start']+(k+1)*dt*schedule.fps;scene.step_control(c,'RELAXED',frame)
            if carry:check_retention(scene.state(),held)
            if not planner.measured_valid():raise RuntimeError('Actual bridge collision')
        scene.save()
        transition=scene.check_transition(np.asarray(goal['wrist_xyz_wxyz']),right.entry_contact,
            expected_object=source_target,canonical=None if args.source_entry_target else b['entry'])
        report['transition']=transition;report['bridge_success']=transition['success']
        if not transition['success']:raise RuntimeError('Actual bridge endpoint does not match next canonical entry')
        (out/'physically_reached_entry.json').write_text(json.dumps(scene.snapshot(),indent=2))
        next_data=dict(np.load(args.proposals/manifest['anchors'][right.id],allow_pickle=False))
        seed=b['commands']
        if args.source_entry_target:
            from multipath.anchors.floating import project_hand_commands
            seed=project_hand_commands(scene,seed,scene.snapshot(),right)
            report['next_core_commands']='existing validated local core projected as a proposal from new physical entry'
        validation,_,_=rollout(scene,right,next_data,seed,scene.snapshot(),out/'next_core',
            mode='ANCHOR',core_only=True,continue_from_current=True)
        report['next_core']=validation;report['next_core_success']=validation['success']
        report['state_resets_after_start']=scene.restore_count-resets
        report['status']='bridge_and_core_valid' if validation['success'] and scene.restore_count==resets else 'next_core_failed'
    except Exception as error:
        import traceback;traceback.print_exc();report.update(status='failed',error=f'{type(error).__name__}: {error}')
    finally:
        if scene:scene.save()
        report['elapsed_s']=time.perf_counter()-started
        (out/'bridge_result.json').write_text(json.dumps(report,indent=2))
        print('BRIDGE_RESULT',report['status'],flush=True)
        if app:app.close()


if __name__=='__main__':main()
