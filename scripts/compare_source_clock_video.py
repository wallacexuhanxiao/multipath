"""Frame-for-frame source/actual comparison; never rescale either clock."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--source-video',type=Path,required=True)
    p.add_argument('--run-dir',type=Path,required=True)
    args=p.parse_args()
    report=json.loads((args.run_dir/'pipeline_result.json').read_text())
    trace=json.loads((args.run_dir/'actual_trace.json').read_text())
    if not report.get('source_clock'):raise ValueError('Comparison requires original-clock physics')
    source=cv2.VideoCapture(str(args.source_video))
    actual=cv2.VideoCapture(str(args.run_dir/'mounted_xhand_bc_curobo_actual.mp4'))
    n=int(source.get(cv2.CAP_PROP_FRAME_COUNT));fps=source.get(cv2.CAP_PROP_FPS)
    if int(actual.get(cv2.CAP_PROP_FRAME_COUNT))!=n or len(trace)!=n:
        raise ValueError('Only a complete full-rate actual recording can be compared')
    if abs(actual.get(cv2.CAP_PROP_FPS)-fps)>.01:raise ValueError('Video clocks do not match')
    if not np.allclose([r['time_s'] for r in trace],np.arange(n)/fps,atol=1e-6):
        raise ValueError('Actual samples do not match source timestamps')
    output=args.run_dir/'source_vs_actual_4p8s.mp4'
    writer=cv2.VideoWriter(str(output),cv2.VideoWriter_fourcc(*'mp4v'),fps,(1120,400))
    if not writer.isOpened():raise RuntimeError('Cannot open comparison writer')
    for i in range(n):
        ok,a=source.read();ok2,b=actual.read()
        if not ok or not ok2:raise RuntimeError('Missing video frame')
        frame=np.zeros((400,1120,3),dtype=np.uint8)
        frame[40:,:640]=cv2.resize(a,(640,360));frame[40:,640:]=cv2.resize(b,(480,360))
        cv2.putText(frame,f'Source | frame {i} | {i/fps:.2f}s',(12,27),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1)
        cv2.putText(frame,'Measured PhysX | same clock',(652,27),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
        writer.write(frame)
    writer.release();source.release();actual.release()
    check=cv2.VideoCapture(str(output));count=int(check.get(cv2.CAP_PROP_FRAME_COUNT));ok,_=check.read();check.release()
    if not ok or count!=n:raise RuntimeError('Comparison video verification failed')
    print(json.dumps({'video':str(output.resolve()),'frames':n,'fps':fps,'duration_s':n/fps}))


if __name__=='__main__':main()
