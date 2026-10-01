"""Generate runtime events from an arbitrary reference, source video and VLM output."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from multipath.reference_events import compile_reference_schedule
from multipath.schedule import Schedule


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--reference',type=Path,required=True)
    parser.add_argument('--vlm-schedule',type=Path,required=True)
    parser.add_argument('--video',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    video=cv2.VideoCapture(str(args.video));fps=video.get(cv2.CAP_PROP_FPS)
    frames=int(video.get(cv2.CAP_PROP_FRAME_COUNT));video.release()
    reference=dict(np.load(args.reference,allow_pickle=False))
    if frames!=len(reference['qpos_right_actual']):raise ValueError('Video/reference frame count mismatch')
    result=compile_reference_schedule(reference,json.loads(args.vlm_schedule.read_text()),fps)
    result['inputs']={name:{'path':str(path.resolve()),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
                      for name,path in [('reference',args.reference),('vlm_schedule',args.vlm_schedule),('video',args.video)]}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    Schedule.load(args.output)
    print(json.dumps({'output':str(args.output.resolve()),'events':{k:result[k] for k in
                     ('grasp_contact_frame','grasp_closed_frame','release_frame')},'schedule':result['schedule']}))


if __name__=='__main__':main()
