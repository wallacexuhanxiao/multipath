"""Scientific comparison of actually executed wrist paths, by oracle interval."""
import argparse,json
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser();p.add_argument('--run-dir',type=Path,action='append',default=[])
    p.add_argument('--local-bank',type=Path,help='Plot independently validated local trials, explicitly labeled as local evidence')
    p.add_argument('--baseline-run',type=Path)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    records=[];intervals=None;source=None;local_paths={}
    if args.local_bank:
        if args.run_dir or not args.baseline_run:p.error('local-bank requires baseline-run and no run-dir')
        bank=json.loads((args.local_bank/'bridge_bank.json').read_text())
        baseline=json.loads((args.baseline_run/'pipeline_result.json').read_text())
        if not baseline['full_pipeline_success'] or baseline['source_provenance']!=bank['source_provenance']:
            raise ValueError('Local bank and successful baseline differ')
        trace=json.loads((args.baseline_run/'actual_trace.json').read_text())
        intervals=[]
        for item in bank['intervals']:
            a,b=item['window'];intervals.append((a,b,item['budget_s']))
            paths=[np.asarray([r['wrist_xyz_wxyz'][:3] for r in trace if a<=r['source_frame']<=b])]
            for solution in item['solutions'][1:]:
                stem=args.local_bank/solution['artifact']
                artifact=json.loads(stem.with_suffix('.json').read_text())
                if not artifact['physical_validation_success'] or not artifact['entry']['physically_acquired_state']:
                    raise ValueError('Plot requires actual physically validated local paths')
                paths.append(np.load(stem.with_suffix('.npz'))['actual_wrist_path'])
            local_paths[a]=paths
    elif not args.run_dir:p.error('Provide actual full runs or a local bank')
    for directory in args.run_dir:
        report=json.loads((directory/'pipeline_result.json').read_text())
        if not report.get('full_pipeline_success'):raise ValueError('Only successful full actual rollouts can be compared')
        if source is not None and report['source_provenance']!=source:raise ValueError('Sources differ')
        source=report['source_provenance']
        windows=[s for s in report['segments'] if s['mode']=='RELAXED']
        current=[(w['start'],w['end'],w['budget_s']) for w in windows]
        if intervals is not None and current!=intervals:raise ValueError('Source intervals differ')
        intervals=current
        trace=json.loads((directory/'actual_trace.json').read_text());paths={}
        for a,b,_ in current:
            file=directory/f'relaxed_{a}_{b}.npz'
            if file.exists():
                with np.load(file,allow_pickle=False) as data:
                    if 'actual_wrist_path' in data:paths[a]=data['actual_wrist_path'].copy()
            if a not in paths:
                paths[a]=np.asarray([r['wrist_xyz_wxyz'][:3] for r in trace if a<=r['source_frame']<=b])
        records.append(paths)
    rows=2 if args.local_bank and len(intervals)==4 else 1
    cols=int(np.ceil(len(intervals)/rows))
    fig,axes=plt.subplots(rows,cols,figsize=(5.5*cols,4.7*rows),squeeze=False)
    colors=['#168aad','#ed4c78','#f6b83e','#7b52ab']
    for ax,(a,b,budget) in zip(axes.ravel(),intervals):
        paths=local_paths[a] if args.local_bank else [record[a] for record in records]
        merged=np.concatenate(paths);center=merged.mean(0)
        _,_,basis=np.linalg.svd(merged-center,full_matrices=False)
        for i,path in enumerate(paths):
            xy=(path-center)@basis[:2].T*1000
            ax.plot(xy[:,0],xy[:,1],lw=2,color=colors[i%len(colors)],label=f'Solution {i+1}')
            ax.scatter(*xy[0],s=28,marker='o',color=colors[i%len(colors)])
            ax.scatter(*xy[-1],s=35,marker='x',color=colors[i%len(colors)])
        ax.set_title(f'RELAXED [{a}, {b})\n{budget:.3f} s unchanged')
        ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2)
        ax.set_xlabel('Common spatial projection 1 (mm)')
        ax.set_ylabel('Common spatial projection 2 (mm)')
        ax.legend(frameon=False,fontsize=9)
    fig.suptitle('Independent local PhysX trials (not complete-rollout evidence)' if args.local_bank else
                'Actual PhysX wrist trajectories; same oracle interactions and source clock')
    fig.tight_layout();args.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,dpi=180,bbox_inches='tight')
    fig.savefig(args.output.with_suffix('.pdf'),bbox_inches='tight')


if __name__=='__main__':main()
