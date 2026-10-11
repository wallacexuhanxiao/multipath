"""First joint local Anchor experiment from two actual physical approaches.

A passed single-hand artifact is an initialization only. Both the padded
rollout and the exported two-hand core must pass fresh continuous PhysX.
"""
import argparse,json,os
from dataclasses import replace
from pathlib import Path


def main():
    p=argparse.ArgumentParser()
    for name in ('schedule','proposals','floating-config','xhand-urdf','left-xhand-urdf',
                 'object-cache','right-prefix-run','right-anchor-artifacts','output-dir'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--left-padded-controls',type=Path,help='Compiled local two-hand trial proposal; fresh physics is still required')
    p.add_argument('--left-prefix-run',type=Path,help='Freeze a passed supporting-hand cuRobo predecessor for this source')
    p.add_argument('--right-boundary-feedback-run',type=Path,help='Rejected joint bridge collision fit for local exit resynthesis')
    p.add_argument('--bridge-feedback-run',type=Path,help='Rejected physical bridge; refine handoff or collision predictions from its measured tracking error')
    p.add_argument('--bridge-physical-seed',type=Path,help='Refine an existing cuRobo bridge with short fresh physical trials, preserving path and timing')
    p.add_argument('--bridge-physics-seconds',type=float,default=180.)
    p.add_argument('--bridge-physical-program',type=Path,help='Reuse a physically compiled bridge; final continuous execution revalidates it')
    p.add_argument('--right-core-run',type=Path,help='Freeze a physically passed right core while repairing the other hand')
    p.add_argument('--carry-context-run',type=Path,help='Actual failed joint bridge whose planned controls provide outgoing synthesis context')
    p.add_argument('--left-ppo-checkpoint',type=Path,help='Continue a small PPO optimizer for the identical local residual reference')
    p.add_argument('--left-ppo-seconds',type=float,default=240.,help='Bound the local PPO compute budget, independently of source time')
    p.add_argument('--continue-oracle',action='store_true',help='Continue the reached joint core exit with cuRobo and local oracle interactions')
    p.add_argument('--core-tracking-policy',choices=('strict','diagnostic'),default='diagnostic',
                   help='Record per-frame CORE reference deviations; keep physical and boundary checks')
    args=p.parse_args();out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    report=dict(bimanual_anchor_success=False,full_pipeline_success=False,task_success=False,
                source='oracle',source_time_stretch=False,vlm_used=False,pbs_job_id=os.environ.get('PBS_JOBID'))
    from multipath.execution.startup import watch_dependency_imports,start_simulation
    watch_dependency_imports(out);scene=None;app=None
    try:
        import numpy as np
        from multipath.oracle_schedule import OracleSchedule
        from multipath.execution.floating_scene import FloatingScene
        from multipath.planning.floating import FloatingPlanner,pose_coordinates
        from multipath.execution.wrench_handoff import segment_wrenches
        from multipath.anchors.floating import local_controls,stopping_context,AcquiredGraspProposal,validate_commands,outgoing_carry_context
        from multipath.anchors.mpc import WindowEvaluation
        from multipath.planning.timing import physics_ticks_per_frame
        schedule=OracleSchedule.load(args.schedule)
        anchor=min(schedule.anchors.values(),key=lambda a:a.core_start)
        manifest=json.loads((args.proposals/'manifest.json').read_text())
        import hashlib
        if manifest['provenance']['schedule_sha256']!=hashlib.sha256(args.schedule.read_bytes()).hexdigest():
            raise ValueError('Supporting proposals use a different static oracle schedule')
        data=dict(np.load(args.proposals/manifest['anchors'][anchor.id],allow_pickle=False))
        left_contact=schedule.metadata['hand_contact_schedule'][anchor.id]['left']
        left_anchor=replace(anchor,entry_contact=left_contact['entry'],exit_contact=left_contact['exit'])
        left_role=manifest['hands']['left']['object_role']
        left_data=dict(data,wrist_hand_q=data['left_wrist_hand_q'],tool_pose=data[left_role+'_pose'])
        previous=json.loads((args.right_prefix_run/'pipeline_result.json').read_text())
        if any(previous['source_provenance'][k]!=manifest['provenance'][k] for k in ('triplet','sequence')):
            raise ValueError('Right initialization belongs to a different source')
        right_initial=previous['initial_state']
        initial=np.asarray(right_initial['command_wrist']);initial_hand=right_initial['command_hand_q']
        prefix=np.load(args.right_prefix_run/f'relaxed_0_{anchor.core_start}.npz',allow_pickle=False)['commands']
        seed=np.load(args.right_anchor_artifacts/anchor.id/'commands.npz',allow_pickle=False)['commands']
        tpf=physics_ticks_per_frame(schedule.fps,FloatingScene.dt)
        if len(prefix)!=anchor.core_start*tpf or len(seed)!=(anchor.core_end-anchor.core_start)*tpf:
            raise ValueError('Right-hand seeds differ from oracle source timing')
        left_initial=data['left_wrist_hand_q'][0,:7].copy()
        left_initial[2]+=max(.1,float(np.linalg.norm(np.ptp(data['left_human_joints'][0],axis=0))))
        app=start_simulation(out)
        scene=FloatingScene(args,manifest,out/'padded',initial,initial_hand=initial_hand,
            supporting_initial={'left':dict(urdf=args.left_xhand_urdf,pose=left_initial,
                hand=data['left_wrist_hand_q'][0,7:],object_role=left_role)})
        left=scene.supporting_hands['left']
        left.surface_supported_object=bool(schedule.metadata.get('object_support',{}).get(left_role))
        report.update(embodiment=scene.embodiment,source_provenance=manifest['provenance'],
            core_interval=[anchor.core_start,anchor.core_end],synthesis_interval=[anchor.synth_start,anchor.synth_end],
            source_fps=schedule.fps,target_fixture=False,initial_state=scene.snapshot(),
            right_seed_is_physical_proof=False)
        report['retention_sampling']=dict(relative_pose_and_native_touch_hz=1/scene.dt,
            thumb_and_non_thumb_opposition_hz=schedule.fps,native_dropouts_recorded=True)
        if args.left_prefix_run:
            from multipath.anchors.boundary_feedback import same_source_seed
            cached=json.loads((args.left_prefix_run/'bimanual_result.json').read_text())
            if not cached.get('bimanual_anchor_success') or not same_source_seed(cached['source_provenance'],manifest['provenance']):
                raise ValueError('Supporting predecessor cache is not a validated Anchor of this source')
            if cached['core_interval']!=[anchor.core_start,anchor.core_end] or cached['synthesis_interval']!=[anchor.synth_start,anchor.synth_end]:
                raise ValueError('Supporting predecessor cache has different interaction intervals')
            old=cached['initial_state']['hands']['left'];actual=left.snapshot()
            if cached['initial_state']['scene_id']!=scene.scene_id:
                raise ValueError('Frozen supporting predecessor has a different physical scene')
            for key in ['wrist_xyz_wxyz','wrist_twist','hand_q','hand_dq','command_wrist','command_twist','command_hand_q']:
                if not np.allclose(old[key],actual[key],rtol=0,atol=1e-6):
                    raise ValueError('Frozen supporting predecessor has a different actual initial state')
            for role,target in old['objects'].items():
                for key in ('pose','velocity'):
                    if not np.allclose(target[key],actual['objects'][role][key],rtol=0,atol=1e-6):
                        raise ValueError('Frozen supporting predecessor has different actual object state')
            left_prefix=np.load(args.left_prefix_run/'left_predecessor_commands.npz',allow_pickle=False)['commands']
            if left_prefix.shape!=(anchor.synth_start*tpf,25) or not validate_commands(left,left_prefix,actual)['feasible']:
                raise ValueError('Frozen supporting predecessor violates the original source clock/hand limits')
            left_timing=dict(cached['left_predecessor_plan'],frozen_physical_predecessor=str(args.left_prefix_run))
        else:
            planner=FloatingPlanner(left,False,out/'left_initial_planner',goal_hand=data['left_wrist_hand_q'][0,7:])
            target=data['left_wrist_hand_q'][0,:7].copy();reference=target.copy()
            for height in np.linspace(0,float(np.linalg.norm(left.meshes['tool'].extents)),9):
                target=reference.copy();target[2]+=height
                if planner.valid(pose_coordinates(target)):break
            else:raise RuntimeError('No free supporting-hand synthesis entry')
            left_prefix,left_timing=planner.plan(target,anchor.synth_start*tpf)
        report['left_predecessor_plan']=left_timing
        np.savez_compressed(out/'left_predecessor_commands.npz',commands=left_prefix)
        prefix_wrenches=segment_wrenches(scene,scene.snapshot(),prefix,free_motion=True)
        left_wrenches=segment_wrenches(left,left.snapshot(),left_prefix,free_motion=True)
        left_padded=None;left_drive=None;left_acquisition=None
        right_core=np.asarray(seed).copy();right_baseline=None;right_core_drive=None
        if args.right_core_run:
            if args.right_boundary_feedback_run:raise ValueError('Select either frozen right core or new exit resynthesis')
            from multipath.anchors.boundary_feedback import validated_joint_core_seed
            right_core,report['right_core_seed']=validated_joint_core_seed(scene,anchor,right_core,args.right_core_run)
            right_core_drive=np.load(args.right_core_run/'joint_commands.npz',allow_pickle=False)['right_core_wrenches'].copy()
            if right_core_drive.shape!=(len(right_core),6):raise ValueError('Compiled core wrench program has a different source clock')
            report['right_core_seed']['controller_program_retained']=True
        if args.bridge_physical_seed:
            if not args.right_core_run or not args.continue_oracle:
                raise ValueError('Bridge physical refinement requires a compiled local core and oracle continuation')
            from multipath.execution.bridge_physics import replay_predecessor,refine_bridge
            from multipath.execution.bimanual_executor import continue_oracle
            cached=json.loads((args.right_core_run/'bimanual_result.json').read_text())
            if not cached.get('bimanual_anchor_success'):
                raise ValueError('Bridge predecessor needs a physically validated padded/core artifact')
            compiled=dict(np.load(args.right_core_run/'joint_commands.npz',allow_pickle=False))
            left_data=dict(left_data,anchor_contacts=dict(entry_contact=left_anchor.entry_contact,
                                                          exit_contact=left_anchor.exit_contact))
            def replay(folder,mesh=False):
                return replay_predecessor(scene,anchor,data,left_data,prefix,left_prefix,compiled,folder,mesh=mesh)
            replay(out/'bridge_initial_predecessor')
            report['core_exit']=scene.snapshot()
            np.savez_compressed(out/'joint_commands.npz',**compiled)
            report.update(padded_rollout=dict(success=True,cached_physical_artifact=str(args.right_core_run),
                                              fresh_revalidation=False),
                          core_replay=dict(success=True,entry_physically_reachable=True,
                                           state_resets_within_rollout=0),bimanual_anchor_success=True)
            next_anchor=next((a for a in sorted(schedule.anchors.values(),key=lambda a:a.core_start)
                              if a.core_start>anchor.core_end),None)
            def next_entry_precheck():
                if next_anchor is None or next_anchor.entry_contact!=next_anchor.exit_contact or next_anchor.entry_contact!='grasped':
                    return dict(scope='no persistent-grasp core fit required')
                from multipath.anchors.floating import source_grasp_core_controls
                next_data=dict(np.load(args.proposals/manifest['anchors'][next_anchor.id],allow_pickle=False))
                _,fit=source_grasp_core_controls(scene,next_anchor,next_data,scene.snapshot())
                return dict(anchor=next_anchor.id,feasible=True,local_nominal_fit=fit,
                            physically_validated_core=False)
            program,repair=refine_bridge(scene,schedule,manifest,args.bridge_physical_seed,replay,
                                        out/'bridge_physics',max_seconds=args.bridge_physics_seconds,
                                        validated_program=args.bridge_physical_program,
                                        next_entry_precheck=next_entry_precheck)
            report['bridge_physical_refinement']=repair
            if program is None:raise RuntimeError('No physically feasible bridge candidate within the compute budget')
            # Candidate trials are local development. The exported final run
            # starts again at initial and performs every source step in order.
            report['core_exit']=replay(out/'exported_core',mesh=True)
            report['continuous_candidate']=continue_oracle(scene,schedule,manifest,args.proposals,out,
                bridge_programs={anchor.core_end:program},core_tracking_policy=args.core_tracking_policy)
            return
        if args.right_boundary_feedback_run:
            from multipath.anchors.boundary_feedback import collision_exit_proposal,recorded_core_commands
            right_baseline=recorded_core_commands(args.right_boundary_feedback_run,anchor,tpf)
            right_core,report['right_exit_resynthesis']=collision_exit_proposal(
                scene,anchor,right_core,args.right_boundary_feedback_run,
                source_boundary=data['tool_pose'][anchor.core_end-anchor.synth_start])
            condition=report['right_exit_resynthesis'].get('source_boundary_fit',{})
            if condition.get('method')=='joint_anchor_exit_and_next_oracle_entry':
                scene.outgoing_oracle_boundary=dict(source_target=condition['next_source_object_boundary'],
                    budget_s=condition['relaxed_budget_s'],core_end=anchor.core_end)
        right_post=stopping_context(right_core[-1],(anchor.synth_end-anchor.core_end)*tpf,scene.dt)
        right_baseline_padded=(None if right_baseline is None else np.vstack((right_baseline,
            stopping_context(right_baseline[-1],len(right_post),scene.dt))))
        if args.carry_context_run:
            from multipath.anchors.boundary_feedback import same_source_seed
            context_report=json.loads((args.carry_context_run/'continuous_result.json').read_text())
            if not same_source_seed(context_report['source_provenance'],manifest['provenance']):
                raise ValueError('Joint carry context uses another source')
            window=next(w for w in schedule.windows if w['start']==anchor.core_end)
            context=np.load(args.carry_context_run/f'segment_{window["start"]}_{window["end"]}.npz',allow_pickle=False)
            post_ticks=(anchor.synth_end-anchor.core_end)*tpf
            if window['mode']!='RELAXED' or len(context['right'])<post_ticks:
                raise ValueError('Joint carry context does not cover this local post-padding')
            right_post=context['right'][:post_ticks].copy()
            left.post_context_commands=context['left'][:post_ticks].copy()
            context_anchor=json.loads((args.carry_context_run/'bimanual_result.json').read_text())
            if not context_anchor.get('bimanual_anchor_success') or context_anchor['core_exit']['scene_id']!=scene.scene_id:
                raise ValueError('Carry context must originate at a validated same-scene physical core exit')
            left.post_context_entry=context_anchor['core_exit']['hands']['left']
            left.validated_core_reference=False
            left.require_transport_opposition=True
            report['outgoing_context']=dict(physical_run=str(args.carry_context_run),
                mode='planned cuRobo bridge load; unvalidated commands, fresh joint physics required',
                acquisition='opposing native forces before grasp-relative takeover; final load Replay required',
                retention_checked_each_physics_step=True)
        right_padded=np.vstack((prefix[anchor.synth_start*tpf:],right_core,right_post))
        objectives=None;core_entry=None;core_exit=None;padded_failure=None;right_drive=None
        from multipath.validation.physics import check_retention,check_relaxed_tool_target_contact
        post_held=None;post_contact_start=None
        for tick in range(anchor.synth_end*tpf):
            frame=(tick+1)/tpf
            if tick==anchor.synth_start*tpf:
                left_start=left.snapshot()
                left_padded=(np.load(args.left_padded_controls,allow_pickle=False)['left_padded'].copy()
                    if args.left_padded_controls else local_controls(left,left_anchor,left_data,left_start))
                if left_padded.shape!=((anchor.synth_end-anchor.synth_start)*tpf,25):
                    raise ValueError('Supporting proposal differs from the padded source clock')
                left.freeze_carry_core=left_anchor.exit_contact=='grasped'
                from multipath.anchors.floating import project_hand_commands
                left_padded=project_hand_commands(left,left_padded,left_start,left_anchor)
                if not validate_commands(left,left_padded,left_start)['feasible']:
                    raise RuntimeError('Supporting local command precheck failed')
                left_drive=segment_wrenches(left,left_start,left_padded)
                left_acquisition=None if args.left_padded_controls else AcquiredGraspProposal(left,left_anchor,left_data)
                objectives=[WindowEvaluation(scene,anchor,data,scene.snapshot()),
                            WindowEvaluation(left,left_anchor,left_data,left_start)]
            if tick==anchor.core_start*tpf:
                core_entry=scene.snapshot()
                right_drive=segment_wrenches(scene,core_entry,np.vstack((right_core,right_post)),
                    baseline_commands=right_baseline_padded)
                if right_core_drive is not None:right_drive[:len(right_core)]=right_core_drive
            if right_core_drive is not None and not args.carry_context_run and tick==anchor.core_end*tpf:
                right_drive[len(right_core):]=segment_wrenches(scene,scene.snapshot(),right_post)
            if args.carry_context_run and tick==anchor.core_end*tpf:
                right_drive[len(right_core):]=segment_wrenches(scene,scene.snapshot(),right_post,transport=True)
                begin=(anchor.core_end-anchor.synth_start)*tpf
                left_padded[begin:]=outgoing_carry_context(left,left_padded[begin-1],len(left_padded)-begin)
                left_drive[begin:]=segment_wrenches(left,left.snapshot(),left_padded[begin:],transport=True)
                post_held={'right':np.asarray(scene.state()['hand_object_transform']),
                           'left':np.asarray(left.state()['hand_object_transform'])}
                post_contact_start=scene.tool_target_contact_steps
            if tick<anchor.core_start*tpf:
                command=prefix[tick];scene.wrench_bias=prefix_wrenches[tick]
            else:
                k=tick-anchor.core_start*tpf;command=right_padded[(anchor.core_start-anchor.synth_start)*tpf+k]
                scene.wrench_bias=right_drive[k]
            if tick<anchor.synth_start*tpf:
                left_command=left_prefix[tick];left.wrench_bias=left_wrenches[tick]
            else:
                k=tick-anchor.synth_start*tpf;left_command=left_padded[k]
                if frame<=anchor.core_end and left_acquisition is not None:
                    try:left_command=left_acquisition.command(frame,left_command)
                    except (RuntimeError,ValueError) as error:
                        padded_failure=dict(source_frame=scene.ticks/tpf,reason=str(error),hand='left',
                            stage='acquisition_controller',requested_source_frame=frame)
                        break
                left_padded[k]=left_command;left.wrench_bias=left_drive[k]
            scene.step_control(command,'RELAXED' if frame<=anchor.core_start else 'BIMANUAL_PADDED',frame,
                               supporting_commands={'left':left_command})
            if post_held:
                for side,hand in [('right',scene),('left',left)]:
                    try:check_retention(hand.state(),post_held[side],require_opposition=(tick+1)%tpf==0)
                    except RuntimeError as error:padded_failure=dict(source_frame=frame,reason=str(error),hand=side);break
                if padded_failure:break
                try:check_relaxed_tool_target_contact(scene,post_contact_start)
                except RuntimeError as error:
                    padded_failure=dict(source_frame=frame,reason=str(error),hand='right');break
            if frame==anchor.core_end:core_exit=scene.snapshot()
            if frame==anchor.core_start:core_entry=scene.snapshot()
            if (tick+1)%tpf==0 and objectives:
                for side,objective,c in zip(('right','left'),objectives,(command,left_command)):
                    if not objective.observe(frame,c) and padded_failure is None:
                        padded_failure=dict(objective.failure,hand=side)
                if padded_failure:break
                if frame==anchor.core_end:
                    left.validated_core_reference=True
                    if left_acquisition is not None and not args.carry_context_run:
                        begin=(anchor.core_end-anchor.synth_start)*tpf
                        compiled=project_hand_commands(left,left_padded,left_start,left_anchor)
                        left_padded[begin:]=compiled[begin:]
        scene.save()
        if hasattr(left,'last_acquisition_registration'):
            report['supporting_acquisition_registration']=left.last_acquisition_registration
        report['padded_rollout']=dict(success=padded_failure is None and scene.ticks==anchor.synth_end*tpf,
                                     failure=padded_failure,actual_end_frame=scene.ticks/tpf,state_resets_within_rollout=0)
        if not report['padded_rollout']['success'] and padded_failure and padded_failure['hand']=='right':
            from multipath.anchors.bimanual_synthesis import refine_primary
            acquired=report.get('right_exit_resynthesis',{}).get('physical_grasp_frame')
            value=refine_primary(scene,anchor,data,right_padded,
                {'left':dict(anchor=left_anchor,data=left_data,commands=left_padded)},out/'right_refinement',
                baseline_commands=right_baseline_padded,seconds=args.left_ppo_seconds,
                frozen_prefix_ticks=None if acquired is None else round((acquired-anchor.synth_start)*tpf))
            right_padded=value['commands'];report['padded_rollout']=value['validation']
            report['right_refinement']=value['report']
            begin=(anchor.core_start-anchor.synth_start)*tpf;end=(anchor.core_end-anchor.synth_start)*tpf
            right_core=right_padded[begin:end].copy();right_post=right_padded[end:].copy()
            right_drive=value['primary_wrenches'][begin:];left_drive=value['companion_wrenches']['left']
        if not report['padded_rollout']['success']:
            if padded_failure is None or padded_failure['hand']!='left':
                raise RuntimeError(f'Joint padded Anchor failed outside the supporting interaction: {padded_failure}')
            if args.left_padded_controls and not args.carry_context_run and not args.right_boundary_feedback_run:
                raise RuntimeError('Compiled supporting program failed fresh Replay; inspect predecessor/input reuse before retraining')

            def primary_control(frame):
                nonlocal right_drive
                tick=round(frame*tpf)-1
                if tick<anchor.core_start*tpf:return prefix[tick],prefix_wrenches[tick]
                if right_drive is None:
                    right_drive=segment_wrenches(scene,scene.snapshot(),np.vstack((right_core,right_post)),
                        baseline_commands=right_baseline_padded)
                    # A compiled companion CORE includes its executed drive
                    # program. Keep it in repair/evaluation trials just as in
                    # the initial padded Replay; pose commands alone changed
                    # the frozen interaction while optimizing the other hand.
                    if right_core_drive is not None:
                        right_drive[:len(right_core)]=right_core_drive
                k=tick-anchor.core_start*tpf
                if args.carry_context_run and k==len(right_core):
                    right_drive[k:]=segment_wrenches(scene,scene.snapshot(),right_post,transport=True)
                return np.vstack((right_core,right_post))[k],right_drive[k]
            left.primary_control=primary_control

            def acquire(folder):
                nonlocal prefix_wrenches,left_wrenches,right_drive
                scene.begin_initial_trial(folder,mesh=False);right_drive=None
                prefix_wrenches=segment_wrenches(scene,scene.snapshot(),prefix,free_motion=True)
                left_wrenches=segment_wrenches(left,left.snapshot(),left_prefix,free_motion=True)
                for tick in range(anchor.synth_start*tpf):
                    scene.wrench_bias=prefix_wrenches[tick];left.wrench_bias=left_wrenches[tick]
                    scene.step_control(prefix[tick],'RELAXED',(tick+1)/tpf,supporting_commands={'left':left_prefix[tick]})
                return left.snapshot()

            attempts=[];accepted={}
            def fresh_padded(candidate):
                nonlocal left_drive
                start=acquire(out/'left_fresh_entry');folder=out/'fresh_padded'/f'candidate_{len(attempts):03d}'
                scene.begin_recording(folder,mesh=True);scene.capture('SYNTHESIS_CONTEXT',anchor.synth_start)
                drives=segment_wrenches(left,start,candidate);left_drive=drives
                checks=[WindowEvaluation(scene,anchor,data,scene.snapshot()),WindowEvaluation(left,left_anchor,left_data,start)]
                failure=None
                held=None;contact_start=None
                for k,c in enumerate(candidate):
                    if args.carry_context_run and k==(anchor.core_end-anchor.synth_start)*tpf:
                        candidate[k:]=outgoing_carry_context(left,candidate[k-1],len(candidate)-k)
                        drives[k:]=segment_wrenches(left,left.snapshot(),candidate[k:],transport=True)
                        held={'right':np.asarray(scene.state()['hand_object_transform']),
                              'left':np.asarray(left.state()['hand_object_transform'])}
                        contact_start=scene.tool_target_contact_steps
                    left.wrench_bias=drives[k];frame=anchor.synth_start+(k+1)/tpf
                    left.step_control(c,'BIMANUAL_PADDED',frame)
                    if held:
                        for side,hand in [('right',scene),('left',left)]:
                            try:check_retention(hand.state(),held[side],require_opposition=(k+1)%tpf==0)
                            except RuntimeError as error:failure=dict(source_frame=frame,reason=str(error),hand=side);break
                        if failure:break
                        try:check_relaxed_tool_target_contact(scene,contact_start)
                        except RuntimeError as error:
                            failure=dict(source_frame=frame,reason=str(error),hand='right');break
                    if (k+1)%tpf==0:
                        right_command,_=primary_control(frame)
                        for side,check,command in zip(('right','left'),checks,(right_command,c)):
                            if not check.observe(frame,command) and failure is None:failure=dict(check.failure,hand=side)
                        if failure:break
                value=dict(success=failure is None and scene.ticks==anchor.synth_end*tpf,
                    failure=failure,actual_end_frame=scene.ticks/tpf,state_resets_within_rollout=0)
                attempts.append(value);scene.save()
                if value['success']:accepted.update(commands=np.asarray(candidate).copy(),validation=value)
                print('BIMANUAL_FRESH_PADDED',len(attempts),value,flush=True)
                return value['success']

            from multipath.anchors.floating_search import refine,FloatingResidualEnv
            from multipath.anchors.mpc import RefinementRequired
            from multipath.anchors.ppo import LocalPPOSolver,PPOConfig
            from multipath.anchors.floating import project_hand_commands
            start=acquire(out/'left_mpc_entry')
            initial=(local_controls(left,left_anchor,left_data,start)
                if padded_failure.get('stage')=='acquisition_controller' else left_padded)
            proposal=project_hand_commands(left,initial,start,left_anchor)
            try:
                proposal=refine(left,left_anchor,left_data,proposal,start,out/'left_mpc',candidates=16,iterations=2,use_ppo=False)
            except RefinementRequired as error:
                report['left_mpc_failure']=str(error)
                if hasattr(error,'controls'):proposal=project_hand_commands(left,error.controls,start,left_anchor)
            passed=fresh_padded(proposal)
            if not passed and args.carry_context_run and core_exit is not None:
                from multipath.anchors.floating_search import carry_preload_candidates
                report['carry_preload_search']=[]
                for candidate,details in carry_preload_candidates(left,left_anchor,left_padded,start,core_exit['hands']['left']):
                    passed=fresh_padded(candidate);details['physical_acceptance']=passed
                    report['carry_preload_search'].append(details)
                    if passed:
                        report['supporting_refinement']='contact-directed MPC; fresh joint outgoing-load Replay'
                        break
            if not passed:
                if args.carry_context_run or getattr(left,'validated_core_reference',False):
                    # A failed MPC candidate is not the already validated
                    # acquisition. Start carry repair from the reached CORE
                    # program, with the real outgoing bridge as context.
                    # Otherwise PPO can spend its whole budget relearning a
                    # grasp lost by the rejected MPC initialization.
                    proposal=project_hand_commands(left,left_padded,start,left_anchor)
                    report['supporting_ppo_reference']='physically executed Replay core plus planned outgoing context'
                def reached_env(*values):
                    return FloatingResidualEnv(*values,entry_acquirer=lambda:acquire(out/'left_ppo_entry'),controller_handoff=True)
                solver=LocalPPOSolver(left,left_anchor,left_data,out/'left_ppo',PPOConfig(iterations=60,max_seconds=args.left_ppo_seconds),
                    env_factory=reached_env,accept_controls=fresh_padded,warm_start=args.left_ppo_checkpoint)
                candidate=solver(proposal,start,0)
                if candidate is None:raise RuntimeError('Supporting Replay/MPC/residual PPO did not pass fresh joint Replay')
            left_padded=accepted['commands'];report['padded_rollout']=accepted['validation']
            report['fresh_padded_attempts']=attempts
            report.setdefault('supporting_refinement','existing floating MPC then small residual PPO; both hands physically execute')
        begin=(anchor.core_start-anchor.synth_start)*tpf;end=(anchor.core_end-anchor.synth_start)*tpf
        left_core=left_padded[begin:end].copy()
        np.savez_compressed(out/'joint_commands.npz',right=right_core,left=left_core,
                            right_padded=right_padded,left_padded=left_padded,
                            left_padded_wrenches=left_drive,left_core_wrenches=left_drive[begin:end],
                            right_core_wrenches=right_drive[:len(right_core)],
                            right_padded_wrenches=np.vstack((prefix_wrenches[anchor.synth_start*tpf:],right_drive)))
        # Reacquire the same core entry by physical execution from initial.
        scene.begin_initial_trial(out/'exported_core',mesh=True)
        prefix_wrenches=segment_wrenches(scene,scene.snapshot(),prefix,free_motion=True)
        left_wrenches=segment_wrenches(left,left.snapshot(),left_prefix,free_motion=True)
        for tick in range(anchor.core_start*tpf):
            scene.wrench_bias=prefix_wrenches[tick]
            if tick<anchor.synth_start*tpf:c=left_prefix[tick];left.wrench_bias=left_wrenches[tick]
            else:
                # A refined trajectory requires its own takeover profile.
                # Reusing the old baseline's profile changes the reached entry.
                if tick==anchor.synth_start*tpf:left_drive=segment_wrenches(left,left.snapshot(),left_padded)
                k=tick-anchor.synth_start*tpf;c=left_padded[k];left.wrench_bias=left_drive[k]
            scene.step_control(prefix[tick],'RELAXED',(tick+1)/tpf,supporting_commands={'left':c})
        reached=scene.snapshot();left_start=left.snapshot()
        checks=[WindowEvaluation(scene,anchor,data,reached),WindowEvaluation(left,left_anchor,left_data,left_start)]
        # CORE is a slice of the validated controller program as well as its
        # poses/finger targets. A new takeover here changes the known motion.
        drives=[right_drive[:len(right_core)],left_drive[begin:end]]
        report['core_controller_program']='source-clock slice of padded commands and wrenches; no new takeover'
        core_failure=None
        for k,(right_command,left_command) in enumerate(zip(right_core,left_core)):
            scene.wrench_bias=drives[0][k];left.wrench_bias=drives[1][k]
            frame=anchor.core_start+(k+1)/tpf
            scene.step_control(right_command,'BIMANUAL_EXPORTED_CORE',frame,supporting_commands={'left':left_command})
            if (k+1)%tpf==0:
                for side,check,c in zip(('right','left'),checks,(right_command,left_command)):
                    if not check.observe(frame,c) and core_failure is None:core_failure=dict(check.failure,hand=side)
                if core_failure:break
        report['core_replay']=dict(success=core_failure is None and scene.ticks==anchor.core_end*tpf,
                                   failure=core_failure,entry_physically_reachable=True,state_resets_within_rollout=0)
        report['core_entry']=reached;report['core_exit']=scene.snapshot()
        report['bimanual_anchor_success']=bool(report['padded_rollout']['success'] and report['core_replay']['success'])
        if hasattr(scene,'last_outgoing_boundary_precheck'):
            report['outgoing_nominal_precheck']=scene.last_outgoing_boundary_precheck
        if not report['core_replay']['success']:raise RuntimeError(f'Joint exported core failed: {core_failure}')
        if args.continue_oracle:
            from multipath.execution.bimanual_executor import continue_oracle
            if left.surface_supported_object and len(left_padded[end:]):
                left.support_hold_program=dict(commands=left_padded[end:].copy(),wrenches=left_drive[end:].copy())
            report['continuous_candidate']=continue_oracle(scene,schedule,manifest,args.proposals,out,
                boundary_feedback_run=args.bridge_feedback_run,core_tracking_policy=args.core_tracking_policy)
    except Exception as error:
        import traceback
        traceback.print_exc();report['error']=f'{type(error).__name__}: {error}'
    finally:
        if scene is not None:scene.save()
        (out/'bimanual_result.json').write_text(json.dumps(report,indent=2))
        print('BIMANUAL_RESULT',report.get('bimanual_anchor_success'),report.get('error'),flush=True)
        if app is not None:app.close()


if __name__=='__main__':main()
