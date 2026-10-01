"""Design a distance-scaled exclusion tube from one recorded wrist interval.

Only a full successful rollout can seed a new accepted solution. A failed
rollout may seed an explicitly labeled diagnostic planner probe.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from multipath.path_exclusion import (build_exclusions,interval_distance_profile,
                                      resample,tapered_obstacles_along_path,transport_points)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--window-start',type=int,required=True)
    parser.add_argument('--window-end',type=int,required=True)
    parser.add_argument('--diagnostic-only',action='store_true')
    parser.add_argument('--peak-ratio',type=float,default=.45)
    parser.add_argument('--target-arc-ratio',type=float,default=.75)
    parser.add_argument('--max-half-size',type=float,default=.08)
    parser.add_argument('--size-multiplier',type=float,default=1.)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    directory=args.run_dir
    report=json.loads((directory/'pipeline_result.json').read_text())
    if report.get('full_pipeline_success') and not args.diagnostic_only:
        config=build_exclusions([directory],start=args.window_start,end=args.window_end,
                                adaptive_size=True,exclusion_scope='wrist',size_mode='distance',
                                peak_ratio=args.peak_ratio,target_arc_ratio=args.target_arc_ratio,
                                max_half_size=args.max_half_size,size_multiplier=args.size_multiplier)
    else:
        if not args.diagnostic_only:
            raise ValueError('Failed physical rollouts require --diagnostic-only')
        window=[w for w in report['windows'] if w['controller']=='CUROBO' and
                [w['start'],w['end']]==[args.window_start,args.window_end]]
        if len(window)!=1 or 'end_time_s' not in window[0]:
            raise ValueError('Requested motion interval did not execute completely')
        trace_file=directory/'actual_trace.json'
        trace=json.loads(trace_file.read_text())
        path=transport_points(trace,args.window_start,args.window_end)
        profile=interval_distance_profile(path,args.peak_ratio,args.target_arc_ratio,
                                          args.max_half_size,args.size_multiplier)
        boxes,stats=tapered_obstacles_along_path(path,profile['requested_max_half_size_m'],oriented=True)
        endpoint=min(trace,key=lambda item:abs(item['source_frame']-args.window_end))
        metadata={'run':directory.name,'pbs_job_id':report.get('pbs_job_id'),
                  'trace_sha256':hashlib.sha256(trace_file.read_bytes()).hexdigest()}
        segment={'window':[args.window_start,args.window_end],
                 'distance_profile':profile,'minimum_arc_distance_m':profile['minimum_arc_distance_m'],
                 'boundary_wrist_xyz_wxyz':endpoint['wrist_xyz_wxyz'],
                 'boundary_arm_q':endpoint['arm_q'],
                 'sources':[{**metadata,'wrist_path_xyz':resample(path).tolist(),
                             'placement_diagnostics':stats}],
                 'cuboid':boxes}
        config={'version':2,'kind':'virtual_planner_obstacles','exclusion_scope':'wrist',
                'window_policy':'explicit','size_mode':'distance','reference_only':True,
                'source_status':report['status'],'half_size_m':None,
                'scene':{key:report[key] for key in ('source_sha256','mount_xyzw','scene_translation_xyz')},
                'sources':[metadata],'segments':[segment]}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(config,indent=2)+'\n')
    segment=config['segments'][0]
    print(json.dumps({'output':str(args.output.resolve()),'reference_only':bool(config.get('reference_only')),
                      'window':segment['window'],'distance_profile':segment['distance_profile'],
                      'effective_peak_half_size_m':max(box['dims'][1]/2 for box in segment['cuboid'].values()),
                      'obstacle_count':len(segment['cuboid'])}))


if __name__=='__main__':main()
