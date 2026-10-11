"""A passed Oracle V0 -> three interval solutions -> continuous PhysX runs.

The baseline counts as solution 0. Other solutions exclude only previous
paths of the same RELAXED interval. A local bank never counts as a full run.
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
    for name in ('v0-run','schedule','floating-config','xhand-urdf','object-cache','output-dir'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--sim-prefix',type=Path)
    p.add_argument('--solutions-per-interval',type=int,default=3)
    p.add_argument('--attempts-per-solution',type=int,default=24)
    p.add_argument('--resume-bank',type=Path,help='Keep passed local interval solutions and search only missing ones')
    p.add_argument('--resume-diversity-run',type=Path,help='Preserve already successful full runs and repair the first failed remaining solution')
    p.add_argument('--boundary-feedback-run',type=Path,help='Previously observed continuous carry-boundary failure; refine its preceding Anchor first')
    p.add_argument('--validated-anchors',type=Path,help='Optional already physically refined local artifacts of this source')
    p.add_argument('--executed-prefix-run',type=Path,help='Reuse the actually passed prefix for its solution index; replay from initial state')
    p.add_argument('--full-attempts',type=int,default=6)
    args=p.parse_args()
    if args.solutions_per_interval<2:p.error('Diversity requires at least two solutions')
    out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    root=Path(__file__).resolve().parents[1]
    v0=json.loads((args.v0_run/'end_to_end_result.json').read_text())
    final=Path(v0['final_run']);candidate=Path(v0['candidate_run'])
    passed=json.loads((final/'pipeline_result.json').read_text())
    if not ((v0.get('v0_success') or v0.get('success')) and passed.get('full_pipeline_success') and
            passed.get('task_success') and passed.get('state_resets_between_segments')==0):
        raise ValueError('Diversity requires a successful continuous Oracle V0')
    sim=json.loads(args.sim_prefix.read_text()) if args.sim_prefix else [sys.executable]
    report=dict(success=False,status='running',source_provenance=passed['source_provenance'],
        v0_run=str(args.v0_run),baseline_run=str(final),source_duration_s=passed['source_duration_s'],
        solutions_per_interval=args.solutions_per_interval,intervals_share_exclusions=False,
        full_runs=[dict(index=0,run=str(final),full_pipeline_success=True)],steps=[])
    started=time.monotonic()
    def save():
        report['elapsed_s']=time.monotonic()-started
        (out/'diversity_result.json').write_text(json.dumps(report,indent=2))
    def run(name,script,params,result,key):
        step=dict(name=name,status='running');report['steps'].append(step);save()
        print('DIVERSITY_BEGIN',name,flush=True);then=time.monotonic()
        with (out/(name+'.log')).open('w') as stream:
            process=subprocess.run([*sim,str(root/'scripts'/script),*map(str,params)],
                cwd=root,stdout=stream,stderr=subprocess.STDOUT)
        value=json.loads(result.read_text()) if result.exists() else None
        step.update(returncode=process.returncode,elapsed_s=time.monotonic()-then,
            success=bool(value and (value.get(key)=='complete' if key=='status' else value.get(key))),
            status='finished',result=str(result));save()
        print('DIVERSITY_END',name,step['success'],round(step['elapsed_s'],2),flush=True)
        if value is None:raise RuntimeError(f'{name}: result missing; see {name}.log')
        return value
    common=['--schedule',args.schedule,'--proposals',args.v0_run/'proposals',
        '--floating-config',args.floating_config,'--xhand-urdf',args.xhand_urdf,
        '--object-cache',args.object_cache,'--validated-anchors',args.validated_anchors or candidate/'anchors',
        '--planned-bridges',candidate]
    def bank_params():
        return [*common,'--baseline-run',final,'--solutions-per-interval',args.solutions_per_interval,
                '--attempts-per-solution',args.attempts_per_solution]
    def refine_boundary(entry_run,selected_bank,name):
        actual=json.loads((entry_run/'pipeline_result.json').read_text())
        failed=next(s for s in actual['segments'] if not s.get('success'))
        windows=json.loads(args.schedule.read_text())['windows']
        preceding=next(w for w in windows if w['end']==failed['start'])
        if failed['mode']!='RELAXED' or failed.get('contact')!='grasped' or preceding['mode']!='ANCHOR':
            raise RuntimeError('Boundary feedback has no passed preceding interaction Anchor')
        folder=out/name
        params=list(common);position=params.index('--planned-bridges');del params[position:position+2]
        params+=['--entry-run',entry_run,'--baseline-run',candidate,'--bridge-bank',selected_bank,
            '--anchor-id',preceding['anchor'],'--output-dir',folder]
        for entry in report['full_runs'][1:]:
            params+=['--exclude-actual-run',entry['run']]
        result=run(name,'refine_diverse_anchor.py',params,folder/'development_result.json','success')
        if not result.get('success'):raise RuntimeError(f'Physical boundary refinement failed: {result.get("error")}')
        common[common.index('--validated-anchors')+1]=folder/'anchors'
        report.setdefault('physical_boundary_refinements',[]).append(str(folder))
        save()
    def refine_anchor(entry_run,name):
        actual=json.loads((entry_run/'pipeline_result.json').read_text())
        failed=next(s for s in actual['segments'] if not s.get('success'))
        geometry_failure=(failed['mode']=='RELAXED' and failed.get('contact')=='free' and
            'Actual mesh intersects a planning obstacle at entry' in actual.get('error',''))
        if geometry_failure:
            windows=json.loads(args.schedule.read_text())['windows']
            failed=next(w for w in windows if w['end']==failed['start'])
            if failed['mode']!='ANCHOR':raise ValueError('Free-exit repair needs a preceding interaction Anchor')
        elif failed['mode']!='ANCHOR':raise ValueError('Anchor repair needs an actual interaction failure')
        folder=out/name
        params=list(common);position=params.index('--validated-anchors');old_anchors=Path(params[position+1]);del params[position:position+2]
        params+=['--entry-run',entry_run,'--anchor-id',failed['anchor'],'--output-dir',folder]
        if geometry_failure:params+=['--exit-geometry-feedback-run',entry_run]
        result=run(name,'develop_reached_anchor.py',params,folder/'development_result.json','core_success')
        if not result.get('core_success'):raise RuntimeError(f'Local interaction repair failed: {result.get("error")}')
        combined=folder/'combined_anchors';shutil.copytree(old_anchors,combined)
        shutil.copytree(folder/'anchors',combined,dirs_exist_ok=True)
        common[common.index('--validated-anchors')+1]=combined
        report.setdefault('physical_anchor_repairs',[]).append(dict(anchor=failed['anchor'],entry_run=str(entry_run),artifact=str(combined)))
        save()
    try:
        resumed=None
        if args.resume_diversity_run:
            if not args.resume_bank:raise ValueError('A resumed full diversity run requires its local bridge bank')
            resumed=json.loads((args.resume_diversity_run/'diversity_result.json').read_text())
            if resumed['source_provenance']!=passed['source_provenance']:raise ValueError('Resumed full source differs')
            if args.validated_anchors is None:
                anchor_ids=[w['anchor'] for w in json.loads(args.schedule.read_text())['windows']
                    if w['mode']=='ANCHOR']
                # Keep the latest physically validated canonical refinements.
                # Starting again from the original V0 discards repaired exits.
                for step in reversed(resumed.get('steps',[])):
                    if not step.get('success') or not step.get('result'):continue
                    folder=Path(step['result']).parent
                    roots=[folder/'combined_anchors',folder/'anchors']
                    latest=next((r for r in roots if all((r/anchor_id/'commands.npz').exists()
                        for anchor_id in anchor_ids)),None)
                    if latest is not None:
                        common[common.index('--validated-anchors')+1]=latest
                        report['resumed_validated_anchors']=str(latest)
                        break
            for entry in resumed['full_runs'][1:]:
                actual=json.loads((Path(entry['run'])/'pipeline_result.json').read_text())
                if not (actual.get('full_pipeline_success') and actual.get('diverse_solution_success') and
                        actual.get('state_resets_between_segments')==0 and actual['source_provenance']==passed['source_provenance'] and
                        abs(actual['actual_duration_s']-passed['source_duration_s'])<1e-8):
                    raise ValueError('Resumed full trajectory lacks continuous physical evidence')
                report['full_runs'].append(entry)
            report['resumed_from']=str(args.resume_diversity_run);save()
        if args.boundary_feedback_run:
            if not args.resume_bank:raise ValueError('Boundary feedback requires its local bridge bank')
            refine_boundary(args.boundary_feedback_run,args.resume_bank,'boundary_feedback')
        bank=out/'bank_00'
        params=[*bank_params(),'--output-dir',bank]
        if args.resume_bank:params+=['--resume-bank',args.resume_bank]
        local=run('bank_00','explore_oracle_bridges.py',params,
            bank/'bridge_bank.json','status')
        available=min((len(i['solutions']) for i in local['intervals']),default=0)
        if local['status']=='failed' or available<2:raise RuntimeError('No alternative solution available in every interval')
        accepted=[Path(entry['run']) for entry in report['full_runs'][1:]]
        for index in range(1,min(args.solutions_per_interval,available)):
            if any(entry['index']==index for entry in report['full_runs']):continue
            prefix=None
            if resumed:
                attempts=[Path(step['result']).parent for step in resumed.get('steps',[])
                    if step['name'].startswith(f'full_solution_{index}_attempt_') and step.get('result')]
                previous=attempts[-1] if attempts else args.resume_diversity_run/f'full_solution_{index}_attempt_0'
                if (previous/'pipeline_result.json').exists():
                    actual=json.loads((previous/'pipeline_result.json').read_text())
                    failed=next((s for s in actual.get('segments',[]) if not s.get('success')),None)
                    if failed and (failed['mode']=='ANCHOR' or (failed.get('contact')=='free' and
                            'Actual mesh intersects a planning obstacle at entry' in actual.get('error',''))):
                        refine_anchor(previous,f'anchor_feedback_{index}');prefix=previous
            if args.executed_prefix_run:
                prior=json.loads((args.executed_prefix_run/'pipeline_result.json').read_text())
                if prior.get('bridge_solution')==index:prefix=args.executed_prefix_run
            for attempt in range(args.full_attempts):
                folder=out/f'full_solution_{index}_attempt_{attempt}'
                params=[*common,'--embodiment','floating_xhand','--bridge-bank',bank,
                    '--bridge-solution',index,'--output-dir',folder]
                for directory in accepted:params+=['--exclude-actual-run',directory]
                if prefix:params+=['--executed-prefix-run',prefix]
                actual=run(folder.name,'run_oracle_v0.py',params,folder/'pipeline_result.json','diverse_solution_success')
                if actual.get('full_pipeline_success') and actual.get('diverse_solution_success'):
                    accepted.append(folder)
                    report['full_runs'].append(dict(index=index,run=str(folder),full_pipeline_success=True,
                        actual_duration_s=actual['actual_duration_s'],state_resets_between_segments=actual['state_resets_between_segments'],
                        task=actual['task'],intervals=[dict(window=[s['start'],s['end']],diversity=s['diversity'])
                            for s in actual['segments'] if s['mode']=='RELAXED']))
                    save();break
                failed=next((s for s in actual.get('segments',[]) if not s.get('success')),None)
                if not failed or attempt+1==args.full_attempts:
                    raise RuntimeError(f'Continuous solution {index} failed: {actual.get("error")}')
                if failed['mode']=='ANCHOR':
                    refine_anchor(folder,f'anchor_solution_{index}_{attempt}');prefix=folder;continue
                if failed['mode']!='RELAXED':raise RuntimeError(f'Unsupported failed segment: {failed}')
                if failed.get('contact')=='free' and 'Actual mesh intersects a planning obstacle at entry' in actual.get('error',''):
                    refine_anchor(folder,f'free_exit_solution_{index}_{attempt}');prefix=folder;continue
                if failed.get('contact')=='grasped' and 'fixed-time boundary trajectory' in actual.get('error',''):
                    refine_boundary(folder,bank,f'boundary_solution_{index}_{attempt}')
                    prefix=None
                    continue
                # Repair only the observed failing bridge, from the actually
                # executed predecessor. Other accepted interval banks survive.
                repaired=out/f'bank_solution_{index}_repair_{attempt}'
                params=[*bank_params(),'--resume-bank',bank,'--predecessor-run',folder,
                    '--repair-window-start',failed['start'],'--output-dir',repaired]
                for directory in accepted:params+=['--exclude-actual-run',directory]
                local=run(repaired.name,'explore_oracle_bridges.py',params,repaired/'bridge_bank.json','status')
                if local['status']=='failed' or any(len(i['solutions'])<=index for i in local['intervals']):
                    raise RuntimeError('Observed bridge repair exhausted its search budget for this solution')
                bank=repaired;prefix=folder
            else:raise RuntimeError('Continuous solution budget exhausted')
        if len(report['full_runs'])!=args.solutions_per_interval:
            raise RuntimeError(f'Only {len(report["full_runs"])} complete solutions; local interval counts: '
                f'{[(i["window"],len(i["solutions"])) for i in local["intervals"]]}')
        report.update(success=True,status='complete',bridge_bank=str(bank),
            complete_rollout_count=len(report['full_runs']),all_cartesian_combinations_verified=False)
    except Exception as error:
        report.update(status='failed',error=str(error));print('DIVERSITY_FAILED',str(error),flush=True)
    finally:save()
    if not report['success']:raise SystemExit(1)


if __name__=='__main__':main()
