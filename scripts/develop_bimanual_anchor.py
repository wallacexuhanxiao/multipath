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
        from multipath.anchors.floating import local_controls,stopping_context,AcquiredGraspProposal,validate_commands
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
        report.update(embodiment=scene.embodiment,source_provenance=manifest['provenance'],
            core_interval=[anchor.core_start,anchor.core_end],synthesis_interval=[anchor.synth_start,anchor.synth_end],
            source_fps=schedule.fps,target_fixture=False,initial_state=scene.snapshot(),
            right_seed_is_physical_proof=False)
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
        right_core=np.asarray(seed).copy()
        right_post=stopping_context(right_core[-1],(anchor.synth_end-anchor.core_end)*tpf,scene.dt)
        right_padded=np.vstack((prefix[anchor.synth_start*tpf:],right_core,right_post))
        objectives=None;core_entry=None;core_exit=None;padded_failure=None;right_drive=None
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
                right_drive=segment_wrenches(scene,core_entry,np.vstack((right_core,right_post)))
            if tick<anchor.core_start*tpf:
                command=prefix[tick];scene.wrench_bias=prefix_wrenches[tick]
            else:
                k=tick-anchor.core_start*tpf;command=right_padded[(anchor.core_start-anchor.synth_start)*tpf+k]
                scene.wrench_bias=right_drive[k]
            if tick<anchor.synth_start*tpf:
                left_command=left_prefix[tick];left.wrench_bias=left_wrenches[tick]
            else:
                k=tick-anchor.synth_start*tpf;left_command=left_padded[k]
                if frame<=anchor.core_end and left_acquisition is not None:left_command=left_acquisition.command(frame,left_command)
                left_padded[k]=left_command;left.wrench_bias=left_drive[k]
            scene.step_control(command,'RELAXED' if frame<=anchor.core_start else 'BIMANUAL_PADDED',frame,
                               supporting_commands={'left':left_command})
            if frame==anchor.core_end:core_exit=scene.snapshot()
            if frame==anchor.core_start:core_entry=scene.snapshot()
            if (tick+1)%tpf==0 and objectives:
                for side,objective,c in zip(('right','left'),objectives,(command,left_command)):
                    if not objective.observe(frame,c) and padded_failure is None:
                        padded_failure=dict(objective.failure,hand=side)
                if padded_failure:break
        scene.save()
        report['padded_rollout']=dict(success=padded_failure is None and scene.ticks==anchor.synth_end*tpf,
                                     failure=padded_failure,actual_end_frame=scene.ticks/tpf,state_resets_within_rollout=0)
        if not report['padded_rollout']['success']:
            if padded_failure is None or padded_failure['hand']!='left':
                raise RuntimeError('Joint padded Anchor failed outside the supporting interaction')

            def primary_control(frame):
                nonlocal right_drive
                tick=round(frame*tpf)-1
                if tick<anchor.core_start*tpf:return prefix[tick],prefix_wrenches[tick]
                if right_drive is None:
                    right_drive=segment_wrenches(scene,scene.snapshot(),np.vstack((right_core,right_post)))
                k=tick-anchor.core_start*tpf
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
                for k,c in enumerate(candidate):
                    left.wrench_bias=drives[k];frame=anchor.synth_start+(k+1)/tpf
                    left.step_control(c,'BIMANUAL_PADDED',frame)
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
            proposal=project_hand_commands(left,left_padded,start,left_anchor)
            try:
                proposal=refine(left,left_anchor,left_data,proposal,start,out/'left_mpc',candidates=16,iterations=2,use_ppo=False)
            except RefinementRequired as error:
                report['left_mpc_failure']=str(error)
                if hasattr(error,'controls'):proposal=project_hand_commands(left,error.controls,start,left_anchor)
            if not fresh_padded(proposal):
                def reached_env(*values):
                    return FloatingResidualEnv(*values,entry_acquirer=lambda:acquire(out/'left_ppo_entry'),controller_handoff=True)
                solver=LocalPPOSolver(left,left_anchor,left_data,out/'left_ppo',PPOConfig(iterations=60,max_seconds=240.),
                    env_factory=reached_env,accept_controls=fresh_padded)
                candidate=solver(proposal,start,0)
                if candidate is None:raise RuntimeError('Supporting Replay/MPC/residual PPO did not pass fresh joint Replay')
            left_padded=accepted['commands'];report['padded_rollout']=accepted['validation']
            report['fresh_padded_attempts']=attempts
            report['supporting_refinement']='existing floating MPC then small residual PPO; both hands physically execute'
        begin=(anchor.core_start-anchor.synth_start)*tpf;end=(anchor.core_end-anchor.synth_start)*tpf
        left_core=left_padded[begin:end].copy()
        np.savez_compressed(out/'joint_commands.npz',right=right_core,left=left_core,
                            right_padded=right_padded,left_padded=left_padded,
                            left_padded_wrenches=left_drive,left_core_wrenches=left_drive[begin:end],
                            right_core_wrenches=right_drive[:len(right_core)])
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
        if not report['core_replay']['success']:raise RuntimeError(f'Joint exported core failed: {core_failure}')
    except Exception as error:
        import traceback
        traceback.print_exc();report['error']=f'{type(error).__name__}: {error}'
    finally:
        if scene is not None:scene.save()
        (out/'bimanual_result.json').write_text(json.dumps(report,indent=2))
        print('BIMANUAL_RESULT',report.get('bimanual_anchor_success'),report.get('error'),flush=True)
        if app is not None:app.close()


if __name__=='__main__':main()
