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
    p.add_argument('--diagnose-takeover',action='store_true',help='Short physical attribution trials, never counted as a connected pipeline.')
    p.add_argument('--diagnose-components',action='store_true',help='Separate force and torque continuity in the diagnostic.')
    p.add_argument('--diagnose-balanced-force',action='store_true',help='Validate a controller-derived force takeover with zero terminal linear response.')
    p.add_argument('--diagnose-load',action='store_true',help='Compare actual carried-object gravity support with the same geometric plan.')
    p.add_argument('--diagnose-dynamics',action='store_true',help='Check measured assembly dynamics feedforward on the same cuRobo path.')
    p.add_argument('--transport-dynamics',action='store_true',help='Execute cuRobo with measured rigid-grasp dynamics feedforward.')
    p.add_argument('--balanced-handoff',action='store_true',help='Balance the translational takeover input inside the source budget.')
    p.add_argument('--bandwidth-handoff',action='store_true',help='Fade drive memory on measured mass/inertia response scales.')
    p.add_argument('--quiet-boundary',action='store_true',help='Choose one source-conditioned boundary with zero proposed terminal velocity, instead of imposing human velocity.')
    p.add_argument('--wrench-handoff',action='store_true',help='Preserve measured predecessor drive response and fade it in the original bridge time.')
    p.add_argument('--boundary-feedback',type=Path,help='Completed real bridge diagnostic directory for canonical boundary refinement.')
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
    from multipath.planning.bridge import bridge_precheck,project_boundary_target
    projected_source_error=None
    if args.source_entry_target:
        budget=(window['end']-window['start'])/schedule.fps
        goal=project_boundary_target(a['exit'],goal,budget,config['planning'],dt=dt)
        if args.quiet_boundary:
            from multipath.planning.bridge import quiet_boundary_target
            goal=quiet_boundary_target(a['exit'],goal,source_target,budget,dt,config['planning'])
        if args.boundary_feedback:
            from multipath.planning.bridge import feedback_boundary_target
            actual=json.loads((args.boundary_feedback/'end_state.json').read_text())
            feedback_report=json.loads((args.boundary_feedback/'diagnostic_result.json').read_text())
            if (not args.wrench_handoff or feedback_report['kind']!='wrench_continuous'
                    or not feedback_report['completed'] or feedback_report['retention_failed'] or feedback_report['error'] is not None
                    or actual.get('source_provenance')!=manifest['provenance']
                    or not np.isclose(actual['time_s'],right.core_start/schedule.fps)):
                raise ValueError('Boundary feedback must match physical source clock and controller')
            previous=np.load(args.boundary_feedback.parents[1]/'bridge_commands.npz')['commands'][-1,:7]
            goal,boundary_feedback=feedback_boundary_target(a['exit'],goal,previous,actual,source_target,budget,dt,config['planning'])
        from multipath.execution.mounted_scene import pose_matrix
        from multipath.validation.physics import core_pose_errors,object_tracking_score
        tool=pose_matrix(goal['wrist_xyz_wxyz'])@np.asarray(a['exit']['hand_object_transform'])
        tool_pose=np.r_[tool[:3,3],Rotation.from_matrix(tool[:3,:3]).as_quat()[[3,0,1,2]]]
        projected_source_error=core_pose_errors({'objects':{'tool':{'pose':tool_pose.tolist()}}},source_target)
    check=bridge_precheck(a['exit'],goal,(window['end']-window['start'])/schedule.fps,config['planning'])
    report=dict(status='preflight_passed_physics_pending' if check['feasible'] else 'boundary_rejected',development_only=True,full_pipeline_success=False,
        bridge_success=False,next_core_success=False,source_provenance=manifest['provenance'],
        pbs_job_id=os.environ.get('PBS_JOBID'),from_anchor=left.id,to_anchor=right.id,
        source_interval=[window['start'],window['end']],boundary_precheck=check,
        source_time_stretch=False,vlm_used=False,complete_robot_reference_required=False,
        goal_kind='oracle_boundary_target_only' if args.source_entry_target else 'physically_validated_canonical_entry',
        source_boundary_frame=right.core_start)
    if args.boundary_feedback:report['boundary_feedback']=dict(boundary_feedback,physical_rollout=str(args.boundary_feedback))
    if args.quiet_boundary:report['canonical_proposal']='quiet_source_conditioned; physical acquisition and core replay required'
    if projected_source_error is not None:
        report['proposed_boundary_source_error']=projected_source_error
        if not object_tracking_score(projected_source_error)['feasible']:
            check['feasible']=False;check['reasons'].append('projected_boundary_misses_oracle_condition')
            report['status']='boundary_rejected'
    import shutil,multipath
    implementation=out/'implementation';implementation.mkdir(exist_ok=True)
    shutil.copy2(__file__,implementation/Path(__file__).name)
    shutil.copytree(Path(multipath.__file__).parent,implementation/'multipath',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
    app=None;scene=None;resets=None;started=time.perf_counter()
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
        if args.source_entry_target:
            from multipath.planning.floating import sphere_world_clearance
            report['goal_clearance_projection']=[]
            for _ in range(12):
                clearances,worst=sphere_world_clearance(goal['wrist_xyz_wxyz'],planner.nominal_spheres,planner.live['cuboid'])
                report['goal_clearance_projection'].append(clearances)
                if worst[0]>=1e-4-1e-8:break
                pose=np.asarray(goal['wrist_xyz_wxyz']).copy();pose[:3]+=(1e-4-worst[0])*worst[1]
                goal['wrist_xyz_wxyz']=pose.tolist()
            else:raise RuntimeError('No collision-free oracle boundary proposal')
            revised=bridge_precheck(start,goal,check['budget_s'],config['planning'])
            target_tool=pose_matrix(goal['wrist_xyz_wxyz'])@np.asarray(start['hand_object_transform'])
            target_pose=np.r_[target_tool[:3,3],Rotation.from_matrix(target_tool[:3,:3]).as_quat()[[3,0,1,2]]]
            source_error=core_pose_errors({'objects':{'tool':{'pose':target_pose.tolist()}}},source_target)
            report['collision_corrected_source_error']=source_error;report['collision_corrected_boundary_precheck']=revised
            if not revised['feasible'] or not object_tracking_score(source_error)['feasible']:
                raise RuntimeError('Collision-free target violates fixed source boundary or time budget')
        commands,timing=planner.plan(np.asarray(goal['wrist_xyz_wxyz']),round(check['budget_s']/dt),
                                   np.asarray(goal['wrist_twist']),goal_state=goal)
        report['planning']=timing;np.savez_compressed(out/'bridge_commands.npz',commands=commands)
        if args.diagnose_takeover:
            from multipath.execution.bridge_diagnostic import run
            report['diagnostic_trials']=run(scene,start,commands,planner,goal,source_target,out/'diagnostic',components=args.diagnose_components,balanced=args.diagnose_balanced_force,loaded=args.diagnose_load,dynamics=args.diagnose_dynamics,bandwidth=args.bandwidth_handoff)
            report['status']='diagnostic_complete';return
        feedforward=np.zeros((len(commands),6))
        if args.transport_dynamics:
            from multipath.execution.transport_dynamics import TransportDynamics
            model=TransportDynamics(scene,carry);feedforward=model.commands(scene,start,commands)
            report['transport_dynamics']=model.report()
            np.savez_compressed(out/'transport_wrenches.npz',wrenches=feedforward)
        if args.wrench_handoff:
            from multipath.execution.wrench_handoff import takeover_bias
            bias=takeover_bias(start,commands[0],scene.cfg['controller'],new_feedforward=feedforward[0])
            report['controller_handoff']=dict(initial_wrench_bias=bias.tolist(),source_time_budget_s=check['budget_s'],end_bias_zero=True)
        force_profile=None
        handoff_profile=None
        if args.bandwidth_handoff:
            if not args.wrench_handoff or not args.transport_dynamics:raise ValueError('Bandwidth takeover requires dynamics and wrench handoff')
            from multipath.execution.wrench_handoff import bandwidth_handoff_profile
            rotation=Rotation.from_quat(np.asarray(start['wrist_xyz_wxyz'])[[4,5,6,3]]).as_matrix()
            handoff_profile=bandwidth_handoff_profile(len(commands),dt,model.mass,rotation@model.inertia@rotation.T,scene.cfg['controller'])
            report['controller_handoff']['profile']=handoff_profile.tolist()
        if args.balanced_handoff:
            if not args.wrench_handoff or not args.transport_dynamics:raise ValueError('Balanced takeover requires dynamics and wrench handoff')
            from multipath.execution.wrench_handoff import balanced_force_profile
            force_profile=balanced_force_profile(len(commands),dt,model.mass,scene.cfg['controller']['wrist_kp'],scene.cfg['controller']['wrist_kd'])
            report['controller_handoff']['force_profile']=force_profile.tolist()
        for k,c in enumerate(commands):
            scene.wrench_bias=feedforward[k].copy()
            if args.wrench_handoff:
                phase=k/max(1,len(commands)-1);scene.wrench_bias+=bias*(1-3*phase**2+2*phase**3)
                if handoff_profile is not None:scene.wrench_bias=feedforward[k]+bias*handoff_profile[k]
                if force_profile is not None:scene.wrench_bias[:3]=feedforward[k,:3]+bias[:3]*force_profile[k]
            frame=window['start']+(k+1)*dt*schedule.fps;scene.step_control(c,'RELAXED',frame)
            report['actual_end_source_frame']=frame
            if carry:
                retention=check_retention(scene.state(),held)
                report.setdefault('retention_checks',[]).append(dict(source_frame=frame,**retention))
            clearances=planner.measured_clearance()
            if min(clearances.values())<0:
                collisions=planner.measured_mesh_collisions()
                refinement=dict(source_frame=frame,proxy_clearance_m=clearances,mesh_collisions=collisions)
                report.setdefault('proxy_overlap_refinements',[]).append(refinement)
                if collisions:
                    report['first_collision']=refinement
                    (out/'failed_state.json').write_text(json.dumps(scene.snapshot(),indent=2))
                    raise RuntimeError('Actual bridge mesh collision')
        scene.save()
        transition=scene.check_transition(np.asarray(goal['wrist_xyz_wxyz']),right.entry_contact,
            expected_object=source_target,canonical=None if args.source_entry_target else b['entry'])
        report['transition']=transition;report['bridge_success']=transition['success']
        if not transition['success']:raise RuntimeError('Actual bridge endpoint does not match next canonical entry')
        (out/'physically_reached_entry.json').write_text(json.dumps(scene.snapshot(),indent=2))
        next_data=dict(np.load(args.proposals/manifest['anchors'][right.id],allow_pickle=False))
        seed=b['commands']
        if args.source_entry_target:
            from multipath.anchors.floating import project_hand_commands,rebase_grasp_commands
            if right.entry_contact=='grasped':
                seed=rebase_grasp_commands(seed,b['entry'],scene.snapshot(),dt)
            seed=project_hand_commands(scene,seed,scene.snapshot(),right)
            report['next_core_commands']='local core proposal adapted to actual reached grasp; requires fresh physical validation'
        validation,_,_=rollout(scene,right,next_data,seed,scene.snapshot(),out/'next_core',
            mode='ANCHOR',core_only=True,continue_from_current=True,controller_handoff=args.transport_dynamics or args.wrench_handoff)
        report['next_core']=validation;report['next_core_success']=validation['success']
        report['state_resets_after_start']=scene.restore_count-resets
        report['status']='bridge_and_core_valid' if validation['success'] and scene.restore_count==resets else 'next_core_failed'
    except Exception as error:
        import traceback;traceback.print_exc();report.update(status='failed',error=f'{type(error).__name__}: {error}')
        if scene:(out/'failed_state.json').write_text(json.dumps(scene.snapshot(),indent=2))
    finally:
        if scene:scene.save()
        if resets is not None:
            report['state_resets_after_start']=scene.restore_count-resets
            report['actual_executed_duration_s']=scene.ticks*scene.dt-window['start']/schedule.fps
        report['elapsed_s']=time.perf_counter()-started
        (out/'bridge_result.json').write_text(json.dumps(report,indent=2))
        print('BRIDGE_RESULT',report['status'],flush=True)
        if app:app.close()


if __name__=='__main__':main()
