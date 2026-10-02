"""Oracle V0: one world, local interaction commands, fixed-time cuRobo bridges."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
from multipath.anchors.artifact import save_anchor, load_validated_anchor
from multipath.anchors.synthesis import local_commands, fit_timed_controls, hand_limits_from_urdf
from multipath.execution.transition import check_transition
from multipath.planning.timing import fixed_budget_path, parameterize_path
from multipath.validation.physics import check_retention, core_pose_errors
from multipath.validation.task import evaluate_task


def execute(scene,schedule,manifest,proposal_dir,report):
    from curobo.types.robot import JointState
    from curobo.types.math import Pose
    from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
    tpf=round(1/(schedule.fps*scene.dt))
    if abs(tpf*scene.dt*schedule.fps-1)>1e-8:raise ValueError('Physics/source clocks not commensurate')
    proposals={a:dict(np.load(proposal_dir/p,allow_pickle=False)) for a,p in manifest['anchors'].items()}
    stop_anchor=getattr(scene.args,'through_anchor',None)
    required=[a for a in schedule.anchors.values() if not stop_anchor or a.core_start<=schedule.anchors[stop_anchor].core_start]
    canonical={}
    if scene.args.validated_anchors:
        for a in required:
            if getattr(scene.args,'continue_development',False) and not (scene.args.validated_anchors/a.id/'report.json').exists():continue
            canonical[a.id]=load_validated_anchor(scene.args.validated_anchors,a)
    all_canonical=len(canonical)==len(schedule.anchors)
    entry_offsets={}
    held=None;hand_command=np.array(scene.state()['hand_q']);initial_q=np.array(scene.state()['arm_q'])
    report.update(source='oracle',source_fps=schedule.fps,vlm_used=False,complete_robot_reference_required=False,
                  source_duration_s=schedule.num_frames/schedule.fps,state_resets_between_segments=0,
                  relaxed_reference_tracking=False,anchors=[],segments=[],task_success=False,
                  full_pipeline_success=False,physical_validation_stage='final_continuous_rollout' if all_canonical else 'continuous_prefix_development')
    for wi,w in enumerate(schedule.windows):
        info=dict(w,budget_s=(w['end']-w['start'])/schedule.fps,success=False)
        report['segments'].append(info)
        print('ORACLE_SEGMENT',info,flush=True)
        start=scene.state();qstart=np.array(start['arm_q'])
        if w['mode']=='RELAXED':
            carry=w['contact']=='grasped'
            if carry:
                if held is None:raise RuntimeError('No physically acquired grasp boundary')
                check_retention(start,held)
            later=next((v for v in schedule.windows[wi+1:] if v['mode']=='ANCHOR'),None)
            expected_object=None;canonical_entry=None
            if later:
                a=schedule.anchors[later['anchor']];data=proposals[a.id]
                idx=a.core_start-a.synth_start
                wrist=data['wrist_hand_q'][idx,:7]
                if carry:
                    expected_object=data['tool_pose'][idx]
                    goal=expected_object@np.linalg.inv(held)
                    wrist=np.r_[goal[:3,3],Rotation.from_matrix(goal[:3,:3]).as_quat()[[3,0,1,2]]]
                if a.id in canonical:
                    canonical_entry=canonical[a.id]['entry']
                    qgoal=np.array(canonical_entry['arm_q'])
                    from multipath.execution.mounted_scene import pose_matrix
                    expected_object=pose_matrix(canonical_entry['objects']['tool']['pose'])
                    if np.max(np.abs(np.array(canonical_entry['hand_q'])-hand_command))>.10:
                        raise RuntimeError('Next Anchor requires incompatible hand configuration; resynthesize it')
                else:qgoal=None
            else:qgoal=initial_q.copy()
            planner=scene.planner(carry)
            def js(q):return JointState.from_position(scene.tensor.to_device(np.array(q).copy())[None],joint_names=planner.joint_names)
            if qgoal is None:
                offsets=[np.zeros(3)]
                if not carry:
                    extent=float(np.linalg.norm(scene.meshes['tool'].extents))
                    offsets += [np.array([0.,0.,height]) for height in np.linspace(0,extent,9)[1:]]
                candidates=[];qgoal=None
                r=Rotation.from_quat(np.array(wrist)[[4,5,6,3]])*scene.mount.inv()
                for offset in offsets:
                    target=Pose(position=scene.tensor.to_device((wrist[:3]+offset).copy())[None],
                                quaternion=scene.tensor.to_device(r.as_quat()[[3,0,1,2]])[None])
                    sol=planner.ik_solver.solve_single(target,num_seeds=40,
                           retract_config=scene.tensor.to_device(qstart.copy())[None],
                           seed_config=scene.tensor.to_device(qstart.copy())[None,None])
                    feasible=False
                    if bool(sol.success.item()):
                        candidate=sol.solution.detach().cpu().numpy().reshape(-1).copy()
                        feasible=scene.collision_valid(planner,candidate)
                    candidates.append({'offset_xyz':offset.tolist(),'full_geometry_valid':feasible})
                    if feasible:
                        qgoal=candidate;entry_offsets[a.id]=offset;break
                info['entry_geometry_search']=candidates
                if qgoal is None:raise RuntimeError('No mounted collision-free Anchor entry within one tool extent')
            startjs=js(qstart)
            startjs.velocity=scene.tensor.to_device(np.array(start['arm_dq']))[None]
            result=planner.plan_single_js(startjs,js(qgoal),MotionGenPlanConfig(max_attempts=12,enable_graph_attempt=3))
            info['plan_status']=str(result.status)
            if not bool(result.success.item()):raise RuntimeError(f'cuRobo relaxed {w["start"]}:{w["end"]}: {result.status}')
            raw=result.get_interpolated_plan().position.detach().cpu().numpy().reshape(-1,7)
            if np.max(np.abs(raw[0]-qstart))>.04:raise RuntimeError('Planner mutated or mismatched bridge start')
            raw_dt=float(result.interpolation_dt)
            np.savez_compressed(scene.out/f'relaxed_plan_{w["start"]}_{w["end"]}.npz',q=raw,dt=raw_dt,start_q=qstart,goal_q=qgoal)
            try:
                path,timing=fixed_budget_path(raw,raw_dt,(w['end']-w['start'])*tpf,scene.dt,
                                             scene.velocity,scene.acceleration,initial_velocity=start['arm_dq'])
            except RuntimeError as timing_error:
                info['uniform_timing_rejection']=str(timing_error)
                path,timing=parameterize_path(raw,(w['end']-w['start'])*tpf,scene.dt,
                                             scene.velocity,scene.acceleration,initial_velocity=start['arm_dq'])
                timing['curobo_duration_s']=(len(raw)-1)*raw_dt
            info.update(timing)
            # Validate every resampled configuration with real scene, self and carried geometry.
            for configuration in path:
                if not scene.collision_valid(planner,configuration):
                    raise RuntimeError('Resampled path collision check failed')
            info['collision_model']={'self':True,'real_scene':True,'carried_object':carry,'virtual_obstacles':False}
            retention=[]
            path_velocity=np.gradient(path,scene.dt,axis=0)
            path_velocity[0]=start['arm_dq'];path_velocity[-1]=0.
            for j,q in enumerate(path[1:]):
                scene.step(q,hand_command,'RELAXED',w['start']+(j+1)/tpf,path_velocity[j+1])
                if (j+1)%tpf==0:
                    if carry:retention.append(check_retention(scene.state(),held))
                    # Collision feasibility of measured arm state uses same attached geometry.
                    valid=scene.collision_valid(planner,scene.state()['arm_q'])
                    if not bool(valid):raise RuntimeError('Measured relaxed state violates collision model')
            info['retention']=retention
            transition=check_transition(scene,path[-1],w['contact'],expected_object,canonical_entry)
            info['transition']=transition
            if not transition['success']:raise RuntimeError('Relaxed/Anchor handoff mismatch')
        else:
            start['entry_origin']='initial_state' if w['start']==0 else 'predecessor_rollout'
            start['entry_provenance']={'trace':str(scene.out/'actual_trace.json'),
                                      'source_frame':w['start'],'predecessor_segments':report['segments'][:-1]}
            a=schedule.anchors[w['anchor']];data=proposals[a.id];commands=[];validation={'success':False,'handoff_verified':False,
                   'synthesis_method':'replay','context_used':'padded local retarget and command smoothing',
                   'fallback_status':'not_needed','core_pose_errors':[]}
            try:
                if start['contact']['mode']!=a.entry_contact:raise RuntimeError('Anchor entry contact mode mismatch')
                if a.id in canonical:
                    local=canonical[a.id]['report']['validation']
                    for key in ('padded_rollout_complete','core_replay_success','state_resets_within_padded_rollout','state_resets_within_core_replay'):
                        validation[key]=local[key]
                    validation['local_artifact_evidence']=str(scene.args.validated_anchors/a.id/'report.json')
                    handoff=check_transition(scene,canonical[a.id]['entry']['arm_q'],a.entry_contact,canonical=canonical[a.id]['entry'])
                    if not handoff['success']:raise RuntimeError('Canonical Anchor entry mismatch')
                    commands=canonical[a.id]['commands'].copy()
                else:
                    commands=local_commands(scene,a,data,held if a.entry_contact=='grasped' else None,entry_offsets.get(a.id))
                if a.id not in canonical:
                    joint_limits=np.asarray(scene.robot._articulation_view.get_dof_limits()).reshape(-1,2)[scene.arm]
                    commands,fitting=fit_timed_controls(commands[:,:7],commands[:,7:],np.array(start['arm_q']),
                         np.array(start['arm_dq']),np.array(start['hand_q']),schedule.fps,scene.dt,
                         scene.velocity,scene.acceleration,joint_limits,start_hand_dq=np.array(start['hand_dq']),
                         hand_velocity_limits=hand_limits_from_urdf(scene),
                         hand_startup_duration=(a.core_start-a.synth_start)/schedule.fps,
                         hand_joint_limits=np.asarray(scene.robot._articulation_view.get_dof_limits()).reshape(-1,2)[scene.fingers])
                    validation.update(fitting)
                validation.update(control_dt=scene.dt,source_fps=schedule.fps)
                for k,cmd in enumerate(commands):
                    frame=a.core_start+(k+1)/tpf
                    scene.step(cmd[:7],cmd[7:19],'ANCHOR',frame,cmd[19:26])
                    if (k+1)%tpf==0:
                        measured=scene.state()
                        err=core_pose_errors(measured,data['tool_pose'][min(int(frame)-a.synth_start,len(data['tool_pose'])-1)])
                        validation['core_pose_errors'].append(err)
                        if err['position_m']>.05 or err['rotation_rad']>.35:
                            raise RuntimeError('Local Anchor object interaction fidelity failed')
                        if a.entry_contact=='grasped' and a.exit_contact=='grasped':check_retention(measured,held)
                hand_command=commands[-1,7:19].copy()
                transition=check_transition(scene,commands[-1,:7],a.exit_contact,data['tool_pose'][min(a.core_end-a.synth_start,len(data['tool_pose'])-1)],canonical[a.id]['exit'] if a.id in canonical else None)
                validation['handoff']=transition
                if not transition['success']:raise RuntimeError('Anchor exit/contact validation failed')
                held=np.array(scene.state()['hand_object_transform']) if a.exit_contact=='grasped' else None
                validation.update(success=True,handoff_verified=True)
            except Exception as error:
                validation.update(error=str(error),fallback_status='mounted_MPC_required; floating-wrist EgoEngine backend cannot establish mounted reachability')
                raise
            finally:
                artifact=save_anchor(scene.out/'anchors',a,start,scene.state(),commands,validation,manifest['provenance'])
                report['anchors'].append(artifact)
            info['transition']=transition
        actual=scene.state();info['actual_end_s']=actual['time_s']
        if abs(actual['time_s']-w['end']/schedule.fps)>1e-8:raise RuntimeError('Source clock changed')
        info['success']=True
        (scene.out/'progress.json').write_text(json.dumps(report,indent=2))
        if w.get('anchor')==stop_anchor and stop_anchor:
            report.update(status='prefix_complete',prefix_success=True,actual_duration_s=scene.ticks*scene.dt,
                          full_pipeline_success=False)
            return
    task=evaluate_task(scene.state(),manifest,schedule.metadata['task_contract'],report['anchors'])
    report.update(task=task,task_success=task['success'],actual_duration_s=scene.ticks*scene.dt,
                  full_pipeline_success=bool(task['success'] and all_canonical),
                  development_success=bool(task['success']),
                  status=('complete' if all_canonical else 'development_complete') if task['success'] else 'task_failed')
