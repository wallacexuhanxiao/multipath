"""Download successful continuous solutions, render and compare one source."""
import argparse
import json
import os
import subprocess
import sys
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('--host',default='nus-hopper')
    p.add_argument('--remote-run',required=True);p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--render-python',default=sys.executable);args=p.parse_args()
    out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    subprocess.run(['rsync','-a',f'{args.host}:{args.remote_run}/diversity_result.json',str(out)+'/'],check=True)
    report=json.loads((out/'diversity_result.json').read_text());folders=[];collection=[]
    env=dict(os.environ,MUJOCO_GL='cgl' if sys.platform=='darwin' else 'egl',MPLCONFIGDIR=str(out/'matplotlib_cache'))
    scripts=Path(__file__).parent
    for entry in report['full_runs']:
        folder=out/f'solution_{entry["index"]}';folder.mkdir(exist_ok=True)
        subprocess.run(['rsync','-a','--exclude=implementation/','--include=*/',
            '--include=*.json','--include=*.npz','--exclude=*',f'{args.host}:{entry["run"]}/',str(folder)+'/'],check=True)
        actual=json.loads((folder/'pipeline_result.json').read_text())
        if not (actual.get('full_pipeline_success') and actual.get('state_resets_between_segments')==0 and
                actual['source_provenance']==report['source_provenance'] and
                (entry['index']==0 or actual.get('diverse_solution_success'))):
            raise ValueError('A comparison solution lacks continuous actual-state evidence')
        subprocess.run([args.render_python,str(scripts/'render_mounted_actual.py'),'--run-dir',str(folder)],env=env,check=True)
        folders.append(folder)
        collection.append(dict(index=entry['index'],run=str(folder),actual_duration_s=actual['actual_duration_s'],
            task=actual['task'],video=str(folder/'floating_xhand_actual.mp4'),
            intervals=[dict(window=[s['start'],s['end']],budget_s=s['budget_s'],diversity=s.get('diversity'))
                       for s in actual['segments'] if s['mode']=='RELAXED']))
    if len(folders)>1:
        runs=[item for folder in folders for item in ('--run-dir',str(folder))]
        subprocess.run([args.render_python,str(scripts/'compare_diverse_actual.py'),*runs,
            '--stack','vertical','--output',str(out/'solutions_stacked.mp4')],env=env,check=True)
        subprocess.run([args.render_python,str(scripts/'plot_oracle_diversity.py'),*runs,
            '--output',str(out/'actual_wrist_paths.png')],env=env,check=True)
    (out/'collection.json').write_text(json.dumps(dict(status=report['status'],success=report['success'],
        requested_solutions=report['solutions_per_interval'],successful_full_rollouts=collection),indent=2))
    print('DIVERSITY_VIDEOS_READY',len(folders),str(out),flush=True)


if __name__=='__main__':main()
