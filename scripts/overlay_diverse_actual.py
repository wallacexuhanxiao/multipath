"""Overlay two source-synchronized actual-state videos and their measured wrist paths."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def load(directory):
    report=json.loads((directory/'pipeline_result.json').read_text())
    trace=json.loads((directory/'actual_trace.json').read_text())
    capture=cv2.VideoCapture(str(directory/'mounted_xhand_bc_curobo_actual.mp4'))
    frames=int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps=capture.get(cv2.CAP_PROP_FPS)
    accepted=(report.get('full_pipeline_success') and report.get('reference_semantics_validated')
              and (not report.get('path_exclusions') or report.get('diverse_solution_success')))
    if not accepted or frames!=len(trace) or fps<=0:
        raise ValueError(f'{directory}: missing successful actual-state video')
    if not np.allclose([item['time_s'] for item in trace],np.arange(frames)/fps,atol=1e-6):
        raise ValueError(f'{directory}: actual trace is not source-clock aligned')
    return capture,trace,frames,fps,report


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--baseline',type=Path,required=True)
    parser.add_argument('--candidate',type=Path,required=True)
    parser.add_argument('--window-start',type=int,required=True)
    parser.add_argument('--window-end',type=int,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    base,base_trace,n,fps,base_report=load(args.baseline)
    other,other_trace,m,other_fps,other_report=load(args.candidate)
    if n!=m or abs(fps-other_fps)>.001:
        raise ValueError('Both actual videos must have the same frame count and rate')
    for key in ('source_sha256','mount_xyzw','scene_translation_xyz'):
        if base_report[key]!=other_report[key]:
            raise ValueError(f'Actual videos use different {key}')
    path=[]
    for trace in (base_trace,other_trace):
        path.append(np.asarray([item['wrist_xyz_wxyz'][:3] for item in trace
                                if args.window_start<=item['source_frame']<=args.window_end],dtype=float))
    if min(map(len,path))<2:raise ValueError('Selected interval has no measured wrist path')
    xy=np.concatenate([p[:,:2] for p in path]);lo=xy.min(axis=0);hi=xy.max(axis=0)
    span=max(float(np.max(hi-lo)),.02);mid=(lo+hi)/2
    def project(points):
        points=np.asarray(points)
        return np.rint(np.c_[780+(points[:,0]-mid[0])*240/span,
                               205-(points[:,1]-mid[1])*240/span]).astype(np.int32)
    base_line,other_line=map(project,[p[:,:2] for p in path])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    writer=cv2.VideoWriter(str(args.output),cv2.VideoWriter_fourcc(*'mp4v'),fps,(960,720))
    if not writer.isOpened():raise RuntimeError('Cannot create overlay video')
    cyan=(255,235,30);magenta=(210,50,255)
    for frame in range(n):
        ok_a,a=base.read();ok_b,b=other.read()
        if not ok_a or not ok_b:raise RuntimeError('Missing actual frame')
        if a.shape[:2]!=(720,960) or b.shape[:2]!=(720,960):
            raise ValueError('Actual videos must share the fixed render camera')
        # Both videos use the same camera and source timestamp. Mixing pixels
        # makes the two measured poses visible together without resimulation.
        canvas=cv2.addWeighted(a,.5,b,.5,0)
        canvas[:72]=0;canvas[660:]=0
        cv2.putText(canvas,f'Actual-state overlay | {frame/fps:.2f}s | interval [{args.window_start},{args.window_end})',
                    (17,29),cv2.FONT_HERSHEY_SIMPLEX,.62,(255,255,255),1,cv2.LINE_AA)
        cv2.putText(canvas,'Cyan: baseline wrist    Magenta: candidate wrist',
                    (17,56),cv2.FONT_HERSHEY_SIMPLEX,.53,(245,245,245),1,cv2.LINE_AA)
        cv2.rectangle(canvas,(614,75),(945,340),(18,18,18),-1)
        cv2.rectangle(canvas,(614,75),(945,340),(190,190,190),1)
        cv2.putText(canvas,'Measured wrist XY (top view)',(626,97),cv2.FONT_HERSHEY_SIMPLEX,.45,(230,230,230),1,cv2.LINE_AA)
        cv2.polylines(canvas,[base_line],False,cyan,2,cv2.LINE_AA)
        cv2.polylines(canvas,[other_line],False,magenta,2,cv2.LINE_AA)
        source_frame=base_trace[frame]['source_frame']
        if args.window_start<=source_frame<=args.window_end:
            a_xyz=np.asarray(base_trace[frame]['wrist_xyz_wxyz'][:3])
            b_xyz=np.asarray(other_trace[frame]['wrist_xyz_wxyz'][:3])
            for xyz,color in ((a_xyz,cyan),(b_xyz,magenta)):
                cv2.circle(canvas,tuple(project(xyz[None,:2])[0]),6,color,-1,cv2.LINE_AA)
            distance=np.linalg.norm(a_xyz-b_xyz)*100
            cv2.putText(canvas,f'Current wrist separation: {distance:.2f} cm',
                        (17,690),cv2.FONT_HERSHEY_SIMPLEX,.62,(255,255,255),1,cv2.LINE_AA)
        else:
            cv2.putText(canvas,'Paths shown only for the selected cuRobo interval',
                        (17,690),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1,cv2.LINE_AA)
        writer.write(canvas)
    writer.release();base.release();other.release()
    check=cv2.VideoCapture(str(args.output));decoded=0
    while check.read()[0]:decoded+=1
    check.release()
    if decoded!=n:raise RuntimeError('Overlay frame count mismatch')
    print(json.dumps({'video':str(args.output.resolve()),'frames':n,'fps':fps,'duration_s':n/fps}))


if __name__=='__main__':main()
