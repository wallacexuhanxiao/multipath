"""Compare prior and new measured wrist curves, including generated obstacles."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from multipath.path_exclusion import exclusion_segments,transport_points


def main():
    p=argparse.ArgumentParser();p.add_argument('--run-dir',type=Path,required=True);args=p.parse_args()
    report=json.loads((args.run_dir/'pipeline_result.json').read_text())
    trace=json.loads((args.run_dir/'actual_trace.json').read_text())
    segments=exclusion_segments(report['path_exclusions'])
    fig,axes=plt.subplots(len(segments),3,figsize=(13,4.5*len(segments)),squeeze=False,layout='constrained')
    for row,segment in enumerate(segments):
        path=transport_points(trace,*segment['window'])
        for ax,(a,b,title) in zip(axes[row],[(0,1,'Top view'),(0,2,'Side view'),(1,2,'Front view')]):
            for i,source in enumerate(segment['sources']):
                old=np.array(source['wrist_path_xyz'])
                ax.plot(old[:,a],old[:,b],color='#168f9c',lw=2,label='Prior measured path' if i==0 else None)
            ax.plot(path[:,a],path[:,b],color='#e27816',lw=2.4,label='New measured path')
            for i,box in enumerate(segment['cuboid'].values()):
                c=np.array(box['pose'][:3]);h=np.array(box['dims'])/2
                ax.add_patch(Rectangle((c[a]-h[a],c[b]-h[b]),2*h[a],2*h[b],
                                      facecolor='#d3347d',edgecolor='#b32463',alpha=.2,
                                      label='Automatic virtual obstacles' if i==0 else None))
            ax.scatter(path[[0,-1],a],path[[0,-1],b],c=['#2b8b46','#6249a3'],s=35,zorder=5)
            ax.set(title=f'{title} | interval {segment["window"]}',xlabel=f'{"xyz"[a]} (m)',ylabel=f'{"xyz"[b]} (m)')
            ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2)
        axes[row,0].legend(fontsize=8,loc='best')
    fig.suptitle('Automatic path exclusion: measured PhysX trajectories',fontsize=14)
    output=args.run_dir/'path_diversity.png';fig.savefig(output,dpi=170);plt.close(fig)
    print(output.resolve())


if __name__=='__main__':main()
