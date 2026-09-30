"""Summarize measured continuous execution independently of planner status."""
import argparse,json
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser();p.add_argument('--run-dir',type=Path,required=True);args=p.parse_args()
    records=json.loads((args.run_dir/'actual_trace.json').read_text())
    result={'recorded_frames':len(records),'modes':{}}
    if records:
        initial=np.array(records[0]['tool_xyz_wxyz'][:3])
        for mode in sorted(set(r['mode'] for r in records)):
            data=[r for r in records if r['mode']==mode]
            errors=np.array([np.abs(np.asarray(r['arm_q'])-np.asarray(r['command'])[:7]) for r in data])
            objs=np.array([r['tool_xyz_wxyz'][:3] for r in data])
            result['modes'][mode]={'frames':len(data),'max_arm_error_rad':float(errors.max()),
                'max_object_lift_m':float(objs[:,2].max()-initial[2]),'final_object_xyz':objs[-1].tolist()}
        result['duration_s']=records[-1]['time_s']
    (args.run_dir/'actual_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
