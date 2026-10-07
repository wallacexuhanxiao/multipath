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
    p.add_argument('--seed-artifacts',type=Path,help='Optional local warm start of this source; not required.')
    p.add_argument('--initial-bridge-run',type=Path,help='Optional physically passed initial cuRobo segment of this source.')
    p.add_argument('--template-artifacts',type=Path,help='Optional validated local interaction library; new-source initialization only.')
    p.add_argument('--template-proposals',type=Path,help='Local source data associated with the interaction library.')
    p.add_argument('--max-attempts',type=int,default=10)
    p.add_argument('--mpc-candidates',type=int,default=16);p.add_argument('--mpc-iters',type=int,default=2)
    p.add_argument('--ppo-updates',type=int,default=20)
    args=p.parse_args();out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
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
        if not physical and process.returncode:raise RuntimeError(f'{name}: process failed; see {log}')
        return value
    common=['--schedule',args.schedule,'--proposals',out/'proposals','--floating-config',args.floating_config,
        '--xhand-urdf',args.xhand_urdf,'--object-cache',args.object_cache]
    seeds=out/'local_seeds';seeds.mkdir(exist_ok=True)
    if args.seed_artifacts:shutil.copytree(args.seed_artifacts,seeds,dirs_exist_ok=True)
    connected=[];initial_bridge=args.initial_bridge_run;boundary_feedback=None;geometry_feedback=None;candidate=None;carry_context={}
    try:
        run('prepare','prepare_oracle_anchors.py',['--schedule',args.schedule,'--triplet',args.triplet,
            '--sequence',args.sequence,'--mount-report',args.mount_report,'--output-dir',out/'proposals'])
        # Acquire canonical entries through the same actual cuRobo predecessor
        # used in the final schedule. Its last padding frames provide physical
        # synthesis context; a separate synth_start bridge can create a CORE
        # entry/drive history that the final bridge cannot reproduce.
        for attempt in range(args.max_attempts):
            candidate=out/f'candidate_{attempt:02d}'
            params=[*common,'--seed-artifacts',seeds,'--output-dir',candidate]
            if args.template_artifacts:params+=['--template-artifacts',args.template_artifacts,'--template-proposals',args.template_proposals]
            if initial_bridge:params+=['--initial-bridge-run',initial_bridge]
            if boundary_feedback:params+=['--boundary-feedback-run',boundary_feedback]
            if geometry_feedback:params+=['--exit-geometry-feedback-run',geometry_feedback]
            value=run(f'candidate_{attempt:02d}','develop_connected_oracle.py',params,True,
                candidate/'pipeline_result.json','task_success')
            connected.append(candidate)
            if value.get('task_success'):break
            failed=next((s for s in value.get('segments',[]) if not s.get('success')),None)
            if failed is None:raise RuntimeError(value.get('error','No repairable segment'))
            if value['segments'][0].get('success') and initial_bridge is None:initial_bridge=candidate
            repair=dict(attempt=attempt,window=[failed['start'],failed['end']],error=value.get('error'),physical_run=str(candidate))
            report['repairs'].append(repair);save()
            if failed.get('contact')=='grasped' and (candidate/f'relaxed_{failed["start"]}_{failed["end"]}.npz').exists():
                carry_context[failed['start']]=candidate
            if failed['mode']=='ANCHOR':
                anchor_id=failed['anchor'];local=out/f'local_{anchor_id}_{attempt:02d}'
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
                    or failed.get('boundary_precheck',{}).get('feasible') is False):
                preceding=next(w for w in schedule.windows if w['end']==failed['start'])
                if preceding['mode']!='ANCHOR':raise RuntimeError('Carry failure has no preceding interaction Anchor')
                anchor_id=preceding['anchor'];local=out/f'local_{anchor_id}_{attempt:02d}'
                context=carry_context.get(failed['start']);context_args=['--bridge-context-run',context] if context else []
                local_value=run(f'local_{anchor_id}_{attempt:02d}','develop_reached_anchor.py',
                    [*common,'--anchor-id',anchor_id,'--entry-run',candidate,*context_args,
                     '--output-dir',local,'--mpc-candidates',args.mpc_candidates,'--mpc-iters',args.mpc_iters,
                     '--ppo-updates',args.ppo_updates],True,local/'development_result.json','core_success')
                if not local_value.get('core_success'):raise RuntimeError(f'Carry-conditioned {anchor_id} failed: {local_value.get("error")}')
                shutil.copytree(local/'anchors'/anchor_id,seeds/anchor_id,dirs_exist_ok=True)
                boundary_feedback=None
                repair.update(method='preceding_Anchor_resynthesis_with_actual_carry_context',artifact=str(local/'anchors'/anchor_id))
            elif failed.get('contact')=='free' and 'Actual mesh intersects a planning obstacle at entry' in value.get('error',''):
                geometry_feedback=candidate;repair['method']='measured_free_exit_geometry_feedback'
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
        report.update(success=True,status='complete',candidate_run=str(candidate),final_run=str(final),
            actual_duration_s=value['actual_duration_s'],state_resets_between_segments=value['state_resets_between_segments'],
            task=value['task'],video_ready_for_render=True)
    except Exception as error:
        report.update(status='failed',error=str(error));print('E2E_FAILED',str(error),flush=True)
    finally:save()
    print('E2E_RESULT',report['status'],report['elapsed_s'],flush=True)
    if not report['success']:raise SystemExit(1)


if __name__=='__main__':main()
