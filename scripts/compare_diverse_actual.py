"""Compare actual-state rollouts at their recorded timestamps, without retiming."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--run-dir',type=Path,action='append',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--stack',choices=['horizontal','vertical'],default='horizontal')
    args=parser.parse_args()
    if len(args.run_dir)<2:parser.error('Provide at least two actual rollouts')
    captures=[];labels=[];count=None;fps=None;sample_offsets=[]
    for index,directory in enumerate(args.run_dir):
        report=json.loads((directory/'pipeline_result.json').read_text())
        trace=json.loads((directory/'actual_trace.json').read_text())
        name='floating_xhand_actual.mp4' if report.get('embodiment')=='floating_xhand' else 'mounted_xhand_bc_curobo_actual.mp4'
        cap=cv2.VideoCapture(str(directory/name))
        n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));rate=cap.get(cv2.CAP_PROP_FPS)
        offset=1 if report.get('source')=='oracle' and report.get('embodiment')=='floating_xhand' else 0
        if not (report.get('source_clock') or report.get('source')=='oracle') or n+offset!=len(trace) or not rate>0:
            raise ValueError(f'{directory}: missing source-time actual video')
        if not np.allclose([r['time_s'] for r in trace],np.arange(n+offset)/rate,atol=1e-6):
            raise ValueError('Recorded clock does not match video')
        if count is not None and (count!=n or abs(fps-rate)>.001):
            raise ValueError('Rollouts must share frame count and fps')
        count=n;fps=rate;captures.append(cap)
        passed=report.get('full_pipeline_success') and (not report.get('path_exclusions') or report.get('diverse_solution_success'))
        # Full initial-state renders sample t=0 for source frame 0. N+1 trace
        # boundaries still describe N video frames; that is not a time offset.
        sample_offsets.append(0 if passed and trace[0].get('source_frame')==0 else offset)
        label = 'Baseline' if index == 0 else (f'Solution {index + 1}' if passed else f'Candidate {index}')
        labels.append(f"{label} | {'checks passed' if passed else 'REJECTED'}")
    args.output.parent.mkdir(parents=True,exist_ok=True)
    size=(640*len(captures),520) if args.stack=='horizontal' else (640,520*len(captures))
    writer=cv2.VideoWriter(str(args.output),cv2.VideoWriter_fourcc(*'mp4v'),fps,size)
    if not writer.isOpened():raise RuntimeError('Cannot create comparison')
    for frame in range(count):
        canvas=np.zeros((size[1],size[0],3),np.uint8)
        for index,cap in enumerate(captures):
            ok,im=cap.read()
            if not ok:raise RuntimeError('Missing actual frame')
            x=640*index if args.stack=='horizontal' else 0
            y=520*index if args.stack=='vertical' else 0
            canvas[y+40:y+520,x:x+640]=cv2.resize(im,(640,480))
            cv2.putText(canvas,f'{labels[index]} | {(frame+sample_offsets[index])/fps:.2f}s',(x+12,y+27),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
        writer.write(canvas)
    writer.release()
    for cap in captures:cap.release()
    cap=cv2.VideoCapture(str(args.output));decoded=0
    while cap.read()[0]:decoded+=1
    cap.release()
    if decoded!=count:raise RuntimeError('Comparison verification failed')
    print(json.dumps({'video':str(args.output.resolve()),'frames':count,'fps':fps,'duration_s':count/fps}))


if __name__=='__main__':main()
