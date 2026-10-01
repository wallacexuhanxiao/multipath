"""Serial search: exclude accepted paths, simulate candidates, retain successes."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from multipath.path_exclusion import build_exclusions


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--accepted-run',type=Path,action='append',required=True)
    parser.add_argument('--additional-solutions',type=int,default=1)
    parser.add_argument('--attempts-per-solution',type=int,default=3)
    parser.add_argument('--half-size',type=float,default=.025)
    parser.add_argument('--size-mode',choices=['fixed','distance'],default='fixed')
    parser.add_argument('--distance-peak-ratio',type=float,default=.45)
    parser.add_argument('--target-arc-ratio',type=float,default=.75)
    parser.add_argument('--max-half-size',type=float,default=.08)
    parser.add_argument('--tapered-obstacles','--adaptive-size',dest='adaptive_size',action='store_true')
    parser.add_argument('--exclusion-scope',choices=['wrist'],default='wrist')
    parser.add_argument('--shrink-on-retry',action='store_true')
    parser.add_argument('--window-policy',choices=['all','longest-budget'],default='all')
    parser.add_argument('--window-start',type=int,help='Explore one cuRobo interval independently')
    parser.add_argument('--window-end',type=int)
    parser.add_argument('--output-root',type=Path,required=True)
    parser.add_argument('runner_args',nargs=argparse.REMAINDER)
    args=parser.parse_args()
    if args.additional_solutions<1 or args.attempts_per_solution<1:
        parser.error('solution and attempt counts must be positive')
    if (args.window_start is None)!=(args.window_end is None):
        parser.error('--window-start and --window-end must be supplied together')
    runner_args=args.runner_args
    if runner_args and runner_args[0]=='--':runner_args=runner_args[1:]
    if any(x.split('=')[0] in ('--output-dir','--path-exclusions','--start-at-bc','--grasp-only') for x in runner_args):
        parser.error('runner arguments must specify a full rollout without output/exclusion overrides')
    accepted=[p.resolve() for p in args.accepted_run]
    def exclusions_for(runs,multiplier):
        return build_exclusions(runs,start=args.window_start,end=args.window_end,
                                half_size=args.half_size,adaptive_size=args.adaptive_size,
                                exclusion_scope=args.exclusion_scope,window_policy=args.window_policy,
                                size_mode=args.size_mode,peak_ratio=args.distance_peak_ratio,
                                target_arc_ratio=args.target_arc_ratio,max_half_size=args.max_half_size,
                                size_multiplier=multiplier)
    exclusions_for(accepted,1.)
    args.output_root.mkdir(parents=True,exist_ok=True)
    manifest_path=args.output_root/'solutions.json'
    if manifest_path.exists():raise FileExistsError('Use a new output root to preserve the previous search')
    manifest={'status':'running','size_mode':args.size_mode,
              'accepted_runs':[str(p) for p in accepted],'candidates':[]}
    def save():manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
    save()
    for _ in range(args.additional_solutions):
        solution_index=len(accepted)+1
        for attempt in range(1,args.attempts_per_solution+1):
            multiplier=(.8 if args.size_mode=='distance' else .5)**(attempt-1) if args.shrink_on_retry else 1.
            peak=args.half_size*multiplier if args.size_mode=='fixed' else None
            exclusions=exclusions_for(accepted,multiplier) if peak is None else build_exclusions(
                accepted,start=args.window_start,end=args.window_end,half_size=peak,
                adaptive_size=args.adaptive_size,exclusion_scope=args.exclusion_scope,
                window_policy=args.window_policy)
            exclusion_path=(args.output_root/f'exclusions_for_{solution_index:02d}_attempt_{attempt:02d}.json').resolve()
            exclusion_path.write_text(json.dumps(exclusions,indent=2)+'\n')
            out=(args.output_root/f'solution_{solution_index:02d}_attempt_{attempt:02d}').resolve()
            out.mkdir()
            command=[sys.executable,str(Path(__file__).with_name('run_mounted_bc_curobo.py')),
                     *runner_args,'--path-exclusions',str(exclusion_path),'--output-dir',str(out)]
            item={'run':str(out),'exclusion_config':str(exclusion_path),'peak_half_size_m':peak,
                  'interval_peak_half_sizes_m':[[s['window'],s.get('distance_profile',{}).get('requested_max_half_size_m',peak)]
                                                for s in exclusions['segments']],
                  'excluded_runs':[str(p) for p in accepted],'status':'running'}
            manifest['candidates'].append(item);save()
            print(f'Searching solution {solution_index}, attempt {attempt}',flush=True)
            with (out/'run.log').open('w') as log:
                process=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
            report_path=out/'pipeline_result.json'
            report=json.loads(report_path.read_text()) if report_path.exists() else {}
            passed=(process.returncode==0 and report.get('full_pipeline_success')
                    and report.get('reference_semantics_validated') and report.get('diverse_solution_success'))
            item.update({'status':'accepted' if passed else 'rejected','returncode':process.returncode,
                         'error':report.get('error'),'physical_task_success':report.get('task_success',False)})
            if passed:
                accepted.append(out);manifest['accepted_runs']=[str(p) for p in accepted]
            save()
            if passed:break
        else:
            manifest['status']='search_exhausted';save()
            raise SystemExit(1)
    manifest['status']='complete';save()
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':main()
