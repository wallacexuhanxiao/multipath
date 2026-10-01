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
    args=parser.parse_args()
    if len(args.run_dir)<2:parser.error('Provide at least two actual rollouts')
    captures=[];labels=[];count=None;fps=None
    for index,directory in enumerate(args.run_dir):
        report=json.loads((directory/'pipeline_result.json').read_text())
        trace=json.loads((directory/'actual_trace.json').read_text())
        cap=cv2.VideoCapture(str(directory/'mounted_xhand_bc_curobo_actual.mp4'))
        n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));rate=cap.get(cv2.CAP_PROP_FPS)
        if not report.get('source_clock') or n!=len(trace) or not rate>0:
            raise ValueError(f'{directory}: missing source-time actual video')
        if not np.allclose([r['time_s'] for r in trace],np.arange(n)/rate,atol=1e-6):
            raise ValueError('Recorded clock does not match video')
        if count is not None and (count!=n or abs(fps-rate)>.001):
            raise ValueError('Rollouts must share frame count and fps')
        count=n;fps=rate;captures.append(cap)
        passed=report.get('full_pipeline_success') and (not report.get('path_exclusions') or report.get('diverse_solution_success'))
        label = 'Baseline' if index == 0 else (f'Solution {index + 1}' if passed else f'Candidate {index}')
        labels.append(f"{label} | {'checks passed' if passed else 'REJECTED'}")
    args.output.parent.mkdir(parents=True,exist_ok=True)
    writer=cv2.VideoWriter(str(args.output),cv2.VideoWriter_fourcc(*'mp4v'),fps,(640*len(captures),520))
    if not writer.isOpened():raise RuntimeError('Cannot create comparison')
    for frame in range(count):
        canvas=np.zeros((520,640*len(captures),3),np.uint8)
        for index,cap in enumerate(captures):
            ok,im=cap.read()
            if not ok:raise RuntimeError('Missing actual frame')
            canvas[40:,640*index:640*(index+1)]=cv2.resize(im,(640,480))
            cv2.putText(canvas,f'{labels[index]} | {frame/fps:.2f}s',(640*index+12,27),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
        writer.write(canvas)
    writer.release()
    for cap in captures:cap.release()
    cap=cv2.VideoCapture(str(args.output));decoded=0
    while cap.read()[0]:decoded+=1
    cap.release()
    if decoded!=count:raise RuntimeError('Comparison verification failed')
    print(json.dumps({'video':str(args.output.resolve()),'frames':count,'fps':fps,'duration_s':count/fps}))


if __name__=='__main__':main()
