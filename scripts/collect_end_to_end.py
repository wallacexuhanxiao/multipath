"""Download full actual-state results and render local, source-aligned videos."""
import argparse
import json
import os
import subprocess
import sys
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('--host',default='nus-hopper')
    p.add_argument('--remote-run',action='append',required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--render-python',default=sys.executable)
    args=p.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True)
    renderer=Path(__file__).with_name('render_mounted_actual.py');results=[]
    for remote in args.remote_run:
        folder=args.output_dir/Path(remote).name;folder.mkdir(exist_ok=True)
        subprocess.run(['rsync','-a',f'{args.host}:{remote}/end_to_end_result.json',str(folder)+'/'],check=True)
        report=json.loads((folder/'end_to_end_result.json').read_text())
        subprocess.run(['rsync','-a',f'{args.host}:{remote}/run.log',str(folder)+'/'],check=True)
        if not report.get('success'):
            results.append(dict(sequence=report['sequence'],success=False,error=report.get('error'),run=str(folder)))
            continue
        final=folder/'final';final.mkdir(exist_ok=True)
        subprocess.run(['rsync','-a','--include=*/','--include=*.json','--include=*.npz','--exclude=*',
            f'{args.host}:{remote}/final/',str(final)+'/'],check=True)
        env=dict(os.environ,MUJOCO_GL='cgl') if sys.platform=='darwin' else dict(os.environ,MUJOCO_GL='egl')
        subprocess.run([args.render_python,str(renderer),'--run-dir',str(final)],env=env,check=True)
        physical=json.loads((final/'pipeline_result.json').read_text())
        source=folder/'source.mp4'
        subprocess.run(['scp',f'{args.host}:{physical["source_provenance"]["source_video"]}',str(source)],check=True)
        import cv2,numpy as np
        actual=final/'floating_xhand_actual.mp4';a=cv2.VideoCapture(str(actual));s=cv2.VideoCapture(str(source))
        fps=a.get(cv2.CAP_PROP_FPS);count=int(a.get(cv2.CAP_PROP_FRAME_COUNT))
        if (int(s.get(cv2.CAP_PROP_FRAME_COUNT))!=count or abs(s.get(cv2.CAP_PROP_FPS)-fps)>1e-6):
            raise ValueError('Source and actual video clocks differ')
        comparison=folder/'source_vs_actual.mp4'
        writer=cv2.VideoWriter(str(comparison),cv2.VideoWriter_fourcc(*'mp4v'),fps,(960,1440))
        if not writer.isOpened():raise RuntimeError('Comparison writer failed')
        for index in range(count):
            ok,x=s.read();good,y=a.read()
            if not ok or not good:raise RuntimeError('Video decode ended early')
            scale=min(960/x.shape[1],720/x.shape[0]);resized=cv2.resize(x,(round(x.shape[1]*scale),round(x.shape[0]*scale)))
            tile=np.zeros((720,960,3),np.uint8);h,w=resized.shape[:2];tile[(720-h)//2:(720-h)//2+h,(960-w)//2:(960-w)//2+w]=resized
            cv2.putText(tile,f'TACO {report["sequence"]} | source frame {index}',(15,28),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,240,240),2)
            writer.write(np.vstack((tile,y)))
        writer.release();a.release();s.release()
        verify=cv2.VideoCapture(str(comparison));decoded=0
        while verify.read()[0]:decoded+=1
        verify.release()
        if decoded!=count:raise RuntimeError('Comparison video is incomplete')
        results.append(dict(sequence=report['sequence'],run=str(folder.resolve()),success=True,source_duration_s=report['source_duration_s'],
            actual_duration_s=physical['actual_duration_s'],state_resets_between_segments=physical['state_resets_between_segments'],
            task=physical['task'],actual_video=str(actual.resolve()),source_comparison=str(comparison.resolve())))
        print('LOCAL_VIDEO_READY',report['sequence'],str(comparison.resolve()),flush=True)
    index=args.output_dir/'collection.json'
    previous=json.loads(index.read_text()) if index.exists() else []
    by_run={}
    for entry in previous+results:
        run=Path(entry['run']).resolve() if entry.get('run') else Path(entry['actual_video']).parents[1]
        by_run[str(run)]=dict(entry,run=str(run))
    index.write_text(json.dumps(list(by_run.values()),indent=2))


if __name__=='__main__':main()
