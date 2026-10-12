"""Raw TACO + static oracle schedule -> local synthesis -> actual full rollout.

Each development attempt starts at the physical task initial state. Only an
observed failure selects a repair. Earlier outputs provide commands/feedback,
never assigned runtime states. Final success requires a fresh artifact rollout.
"""
import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path


def main():
    p=argparse.ArgumentParser()
    for name in ('schedule','mount-report','floating-config','xhand-urdf','object-cache','output-dir'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--triplet',required=True);p.add_argument('--sequence',required=True)
    p.add_argument('--sim-prefix',type=Path,help='JSON argv prefix, e.g. Singularity + Isaac Python; default current Python.')
    p.add_argument('--retarget-python',default=sys.executable)
    p.add_argument('--prepared-proposals',type=Path,help='Optional cached local oracle proposals; no physical acceptance is reused.')
    p.add_argument('--seed-artifacts',type=Path,help='Optional local warm start of this source; not required.')
    p.add_argument('--initial-bridge-run',type=Path,help='Optional physically passed initial cuRobo segment of this source.')
    p.add_argument('--boundary-feedback-run',type=Path,help='Previously executed bridge feedback; commands must be planned and reexecuted.')
    p.add_argument('--resume-diversity-bank',type=Path,help='Optional passed local bridge candidates; full rollouts remain mandatory.')
    p.add_argument('--template-artifacts',type=Path,help='Optional validated local interaction library; new-source initialization only.')
    p.add_argument('--template-proposals',type=Path,help='Local source data associated with the interaction library.')
    p.add_argument('--max-attempts',type=int,default=10)
    p.add_argument('--mpc-candidates',type=int,default=16);p.add_argument('--mpc-iters',type=int,default=2)
    p.add_argument('--ppo-updates',type=int,default=20)
    p.add_argument('--solutions-per-interval',type=int,default=3,help='1 stops after V0; otherwise run interval diversity and full validation')
    args=p.parse_args();out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    if args.solutions_per_interval<1:p.error('Request at least one solution')
    if bool(args.template_artifacts)!=bool(args.template_proposals):p.error('Template artifact and source data must be supplied together')
    from multipath.oracle_schedule import OracleSchedule
    schedule=OracleSchedule.load(args.schedule)
    sim=json.loads(args.sim_prefix.read_text()) if args.sim_prefix else [sys.executable]
    root=Path(__file__).resolve().parents[1]
    report=dict(status='running',success=False,triplet=args.triplet,sequence=args.sequence,
        source='oracle',vlm_used=False,source_duration_s=schedule.num_frames/schedule.fps,
        full_robot_reference_required=False,seed_artifacts=str(args.seed_artifacts) if args.seed_artifacts else None,
        interaction_template=str(args.template_artifacts) if args.template_artifacts else None,
        initial_bridge_cache=str(args.initial_bridge_run) if args.initial_bridge_run else None,
        template_source_data=str(args.template_proposals) if args.template_proposals else None,
        solutions_per_interval=args.solutions_per_interval,
        steps=[],repairs=[],started_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'))
    started=time.monotonic()
    def save():
        report['elapsed_s']=time.monotonic()-started
        (out/'end_to_end_result.json').write_text(json.dumps(report,indent=2))
    def run(name,script,params,physical=False,result=None,key=None):
        log=out/(name+'.log');step=dict(name=name,log=str(log),status='running',result_path=str(result) if result else None)
        report['steps'].append(step);save();print('E2E_BEGIN',name,flush=True)
        command=(sim if physical else [args.retarget_python])+[str(root/'scripts'/script),*map(str,params)]
        then=time.monotonic()
        with log.open('w') as stream:
            process=subprocess.run(command,cwd=root,stdout=stream,stderr=subprocess.STDOUT)
        step.update(returncode=process.returncode,elapsed_s=time.monotonic()-then,status='finished')
        value=json.loads(result.read_text()) if result and result.exists() else None
        # Kit can return zero after a failed interaction. JSON is authoritative.
        step['success']=bool(value.get(key)) if key and value else process.returncode==0 and not key
        save();print('E2E_END',name,step['success'],round(step['elapsed_s'],2),flush=True)
        if key and value is None:raise RuntimeError(f'{name}: physical result missing; see {log}')
        if not physical and process.returncode and not (key and value):
            raise RuntimeError(f'{name}: process failed; see {log}')
        return value
    common=['--schedule',args.schedule,'--proposals',out/'proposals','--floating-config',args.floating_config,
        '--xhand-urdf',args.xhand_urdf,'--object-cache',args.object_cache]
    seeds=out/'local_seeds';seeds.mkdir(exist_ok=True)
    if args.seed_artifacts:shutil.copytree(args.seed_artifacts,seeds,dirs_exist_ok=True)
    connected=[];initial_bridge=args.initial_bridge_run;boundary_feedback=args.boundary_feedback_run;geometry_feedback=None;candidate=None;carry_context={}
    template_active=args.template_artifacts is not None
    template_entry_rank=0
    def raw_initialization(reason):
        nonlocal template_active,initial_bridge,boundary_feedback,geometry_feedback
        template_active=False;initial_bridge=None;boundary_feedback=None;geometry_feedback=None
        report.setdefault('initialization_fallbacks',[]).append(dict(
            reason=reason,method='raw_local_oracle_retarget',physical_acceptance=False))
        save()
        prepared=json.loads((out/'proposals'/'manifest.json').read_text())
        if any(v.get('redundant_surface_IK_skipped') for v in prepared.get('retarget_details',{}).values()):
            run('prepare_raw_fallback','prepare_oracle_anchors.py',['--schedule',args.schedule,
                '--triplet',args.triplet,'--sequence',args.sequence,'--mount-report',args.mount_report,
                '--object-cache',args.object_cache,'--output-dir',out/'proposals'])
    try:
        if args.prepared_proposals:
            import hashlib
            cached=json.loads((args.prepared_proposals/'manifest.json').read_text());source=cached['provenance']
            if (source['triplet']!=args.triplet or source['sequence']!=args.sequence or
                    source['schedule_sha256']!=hashlib.sha256(args.schedule.read_bytes()).hexdigest() or
                    source.get('full_robot_reference_used') or source.get('vlm_used')):
                raise ValueError('Cached proposals do not match this raw source/oracle schedule')
            shutil.copytree(args.prepared_proposals,out/'proposals',dirs_exist_ok=True)
            report['steps'].append(dict(name='prepare',status='finished',success=True,elapsed_s=0.,
                cached_local_proposals=str(args.prepared_proposals),physical_acceptance=False))
            save()
        else:
            prepare_args=['--schedule',args.schedule,'--triplet',args.triplet,
                '--sequence',args.sequence,'--mount-report',args.mount_report,'--object-cache',args.object_cache,'--output-dir',out/'proposals']
            if template_active:prepare_args+=['--template-artifacts',args.template_artifacts,'--template-proposals',args.template_proposals]
            run('prepare','prepare_oracle_anchors.py',prepare_args)
        # Acquire canonical entries through the same actual cuRobo predecessor
        # used in the final schedule. Its last padding frames provide physical
        # synthesis context; a separate synth_start bridge can create a CORE
        # entry/drive history that the final bridge cannot reproduce.
        for attempt in range(args.max_attempts):
            candidate=out/f'candidate_{attempt:02d}'
            params=[*common,'--seed-artifacts',seeds,'--output-dir',candidate]
            if template_active:params+=['--template-artifacts',args.template_artifacts,'--template-proposals',args.template_proposals,
                '--template-free-entry-rank',template_entry_rank]
            if initial_bridge:params+=['--initial-bridge-run',initial_bridge]
            if boundary_feedback:params+=['--boundary-feedback-run',boundary_feedback]
            if geometry_feedback:params+=['--exit-geometry-feedback-run',geometry_feedback]
            try:
                value=run(f'candidate_{attempt:02d}','develop_connected_oracle.py',params,True,
                    candidate/'pipeline_result.json','task_success')
            except RuntimeError as error:
                if template_active and 'physical result missing' in str(error):
                    raw_initialization(str(error));continue
                raise
            connected.append(candidate)
            if value.get('task_success'):break
            failed=next((s for s in value.get('segments',[]) if not s.get('success')),None)
            if failed is None and value.get('continuous_execution_success') and not value.get('task_success'):
                # Every segment may pass while the released object misses the
                # stricter final task condition. Repair the final interaction,
                # whose local post-context must enforce that same condition.
                failed=dict(next(w for w in reversed(schedule.windows) if w['mode']=='ANCHOR'))
                failed['failure']=dict(reason='final_task_state',task=value['task'])
            if failed is None:raise RuntimeError(value.get('error','No repairable segment'))
            # A supplied cache may have been rejected after a physics/config
            # change. Reuse the newly executed bridge on the next repair;
            # repeatedly retaining the rejected cache replans the same prefix.
            if value['segments'][0].get('success'):initial_bridge=candidate
            repair=dict(attempt=attempt,window=[failed['start'],failed['end']],
                error=value.get('error') or failed.get('failure'),physical_run=str(candidate))
            report['repairs'].append(repair);save()
            if template_active and failed['start']==0:
                count=max((v.get('free_pregrasp_count',1) for v in value.get('interaction_initialization',{}).values()),default=1)
                if template_entry_rank+1<count:
                    template_entry_rank+=1;initial_bridge=None;boundary_feedback=None;geometry_feedback=None
                    repair.update(method='next_physically_acquired_free_library_boundary',free_entry_rank=template_entry_rank)
                else:
                    raw_initialization(value.get('error') or str(failed.get('failure')))
                    repair['method']='failed_optional_library_proposal_fallback_to_raw_local_retarget'
                save();continue
            if failed.get('contact')=='grasped' and (candidate/f'relaxed_{failed["start"]}_{failed["end"]}.npz').exists():
                carry_context[failed['start']]=candidate
            if failed['mode']=='ANCHOR':
                anchor_id=failed['anchor'];local=out/f'local_{anchor_id}_{attempt:02d}'
                details=json.loads((out/'proposals'/'manifest.json').read_text()).get('retarget_details',{}).get(anchor_id,{})
                preceding=next((s for s in value['segments'] if s['end']==failed['start'] and s['mode']=='RELAXED'),None)
                geometry_path=(candidate/f'planner_{preceding["start"]}'/'goal_boundary_geometry.json' if preceding else None)
                # Registering a GT-tool-frame grasp into the actual resting
                # tool frame can rotate a distal phalange below the table.
                # Refit this observed geometry failure before physical search.
                if (details.get('method')=='local_mesh_opposition_surface_IK' and
                        'physical_geometry_entry' not in details and geometry_path and geometry_path.exists() and
                        json.loads(geometry_path.read_text()).get('obstacle')=='table'):
                    geometry=out/f'geometry_{anchor_id}_{attempt:02d}'
                    run(f'geometry_{anchor_id}_{attempt:02d}','refit_free_anchor_geometry.py',
                        ['--schedule',args.schedule,'--proposals',out/'proposals','--entry-run',candidate,
                         '--anchor-id',anchor_id,'--xhand-urdf',args.xhand_urdf,'--object-cache',args.object_cache,
                         '--output-dir',geometry],result=geometry/'geometry_result.json',key='proposal_refitted')
                    repair['method']='local_grasp_geometry_refit_against_actual_tool_and_table'
                    save();continue
                local_value=run(f'local_{anchor_id}_{attempt:02d}','develop_reached_anchor.py',
                    [*common,'--anchor-id',anchor_id,'--entry-run',candidate,'--output-dir',local,
                     '--mpc-candidates',args.mpc_candidates,'--mpc-iters',args.mpc_iters,'--ppo-updates',args.ppo_updates],
                    True,local/'development_result.json','core_success')
                if not local_value.get('core_success'):
                    raise RuntimeError(f'Local {anchor_id} failed: {local_value.get("error")}')
                shutil.copytree(local/'anchors'/anchor_id,seeds/anchor_id,dirs_exist_ok=True)
                repair.update(method='local_Replay_MPPI_residual_PPO_fresh_padded_and_core',artifact=str(local/'anchors'/anchor_id))
            elif failed.get('transition') and not failed['transition']['success'] and failed.get('contact')=='grasped':
                boundary_feedback=candidate;repair['method']='measured_boundary_feedback'
            elif failed.get('contact')=='grasped' and ('Persistent grasp constraint failed' in value.get('error','')
                    or 'No collision-free trajectory in source time' in value.get('error','')
                    or 'No collision-free oracle boundary proposal' in value.get('error','')
                    or 'Floating cuRobo failed:' in value.get('error','')
                    or failed.get('boundary_precheck',{}).get('feasible') is False):
                preceding=next(w for w in schedule.windows if w['end']==failed['start'])
                if preceding['mode']!='ANCHOR':raise RuntimeError('Carry failure has no preceding interaction Anchor')
                anchor_id=preceding['anchor'];local=out/f'local_{anchor_id}_{attempt:02d}'
                context=carry_context.get(failed['start']);context_args=['--bridge-context-run',context] if context else []
                valid_seed=seeds/anchor_id/'report.json'
                if (context is None and 'No collision-free trajectory in source time' in value.get('error','')
                        and valid_seed.exists() and json.loads(valid_seed.read_text()).get('local_rollout_success')):
                    # An unchanged replay repeats the same incompatible exit.
                    # Fit its acquired tail and next entry jointly, then require
                    # fresh padded/core physics before replanning the bridge.
                    context_args=['--outgoing-boundary-run',candidate]
                local_value=run(f'local_{anchor_id}_{attempt:02d}','develop_reached_anchor.py',
                    [*common,'--anchor-id',anchor_id,'--entry-run',candidate,*context_args,
                     '--output-dir',local,'--mpc-candidates',args.mpc_candidates,'--mpc-iters',args.mpc_iters,
                     '--ppo-updates',args.ppo_updates],True,local/'development_result.json','core_success')
                if not local_value.get('core_success'):raise RuntimeError(f'Carry-conditioned {anchor_id} failed: {local_value.get("error")}')
                import numpy as np
                previous=seeds/anchor_id
                fresh=local/'anchors'/anchor_id
                unchanged=False
                if (previous/'commands.npz').exists():
                    old=np.load(previous/'commands.npz')['commands'];new=np.load(fresh/'commands.npz')['commands']
                    old_exit=json.loads((previous/'exit.json').read_text());new_exit=json.loads((fresh/'exit.json').read_text())
                    unchanged=(old.shape==new.shape and np.allclose(old,new,atol=1e-8,rtol=0)
                        and all(np.allclose(old_exit[k],new_exit[k],atol=1e-8,rtol=0)
                                for k in ['wrist_xyz_wxyz','wrist_twist','hand_q','hand_dq','hand_object_transform',
                                          'command_wrist','command_twist','command_hand_q','controller_wrench_bias'])
                        and all(np.allclose(old_exit['objects'][role][key],new_exit['objects'][role][key],atol=1e-8,rtol=0)
                                for role in old_exit['objects'] for key in ['pose','velocity']))
                shutil.copytree(local/'anchors'/anchor_id,seeds/anchor_id,dirs_exist_ok=True)
                boundary_feedback=None
                repair.update(method='preceding_Anchor_resynthesis_with_actual_carry_context',artifact=str(local/'anchors'/anchor_id))
                if unchanged:
                    repair['unchanged_core_and_actual_exit']=True;save()
                    raise RuntimeError('Carry-conditioned synthesis returned the same core and physical exit; repair the boundary/controller instead of repeating it')
            elif failed.get('contact')=='free' and 'Actual mesh intersects a planning obstacle at entry' in value.get('error',''):
                geometry_feedback=candidate;repair['method']='measured_free_exit_geometry_feedback'
            elif failed.get('contact')=='free' and failed['start']==0 and any(text in value.get('error','') for text in
                ['Actual RELAXED mesh collision','No collision-free trajectory in source time','Floating cuRobo failed:']):
                boundary_feedback=candidate;repair['method']='free_Anchor_entry_resynthesis_from_actual_bridge_feedback'
            else:
                raise RuntimeError(f'Unresolved physical failure at {repair["window"]}: {value.get("error")}')
            save()
        else:raise RuntimeError('Physical repair budget exhausted; no successful trajectory exported')
        run('export','export_connected_anchors.py',['--run-dir',candidate,'--schedule',args.schedule])
        final=out/'final'
        value=run('final','run_oracle_v0.py',[*common,'--embodiment','floating_xhand',
            '--validated-anchors',candidate/'anchors','--planned-bridges',candidate,'--output-dir',final],
            True,final/'pipeline_result.json','full_pipeline_success')
        if not value.get('full_pipeline_success'):raise RuntimeError(f'Fresh full rollout failed: {value.get("error")}')
        report.update(success=False,v0_success=True,status='v0_complete',candidate_run=str(candidate),final_run=str(final),
            actual_duration_s=value['actual_duration_s'],state_resets_between_segments=value['state_resets_between_segments'],
            task=value['task'],video_ready_for_render=True,complete_rollout_count=1)
        save()
        if args.solutions_per_interval>1:
            diversity=out/'diversity'
            report['diversity_run']=str(diversity);save()
            params=['--v0-run',out,'--schedule',args.schedule,'--floating-config',args.floating_config,
                '--xhand-urdf',args.xhand_urdf,'--object-cache',args.object_cache,'--output-dir',diversity,
                '--solutions-per-interval',args.solutions_per_interval]
            if args.sim_prefix:params+=['--sim-prefix',args.sim_prefix]
            if args.resume_diversity_bank:params+=['--resume-bank',args.resume_diversity_bank]
            result=run('diversity','run_oracle_diversity.py',params,False,diversity/'diversity_result.json','success')
            report['complete_rollout_count']=len(result.get('full_runs',[]))
            if not result.get('success'):raise RuntimeError(f'Diversity failed: {result.get("error")}')
            report['complete_rollout_count']=result['complete_rollout_count']
        report.update(success=True,status='complete')
    except Exception as error:
        report.update(success=False,status='failed',error=str(error));print('E2E_FAILED',str(error),flush=True)
    finally:save()
    print('E2E_RESULT',report['status'],report['elapsed_s'],flush=True)
    if not report['success']:raise SystemExit(1)


if __name__=='__main__':main()
