"""Initialize local Anchor resynthesis with a source-conditioned slow boundary.

Only command proposals change. A fresh padded/core PhysX validation is required;
the script never creates or overwrites a physical state artifact.
"""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from multipath.anchors.artifact import load_validated_anchor
from multipath.anchors.floating import controls_from_poses
from multipath.oracle_schedule import OracleSchedule


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--schedule',type=Path,required=True);p.add_argument('--proposals',type=Path,required=True)
    p.add_argument('--artifact-root',type=Path,required=True);p.add_argument('--anchor-id',required=True)
    p.add_argument('--controls',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--blend-frames',type=int,default=9)
    args=p.parse_args();schedule=OracleSchedule.load(args.schedule);anchor=schedule.anchors[args.anchor_id]
    manifest=json.loads((args.proposals/'manifest.json').read_text())
    artifact=load_validated_anchor(args.artifact_root,anchor,provenance=manifest['provenance'])
    state=artifact['exit'];dt=artifact['report']['validation']['control_dt'];tpf=round(1/(schedule.fps*dt))
    commands=np.load(args.controls,allow_pickle=False)['commands'].copy()
    end=(anchor.core_end-anchor.synth_start)*tpf;begin=end-args.blend_frames*tpf
    if begin<=(anchor.core_start-anchor.synth_start)*tpf or len(commands)!=(anchor.synth_end-anchor.synth_start)*tpf:
        raise ValueError('Boundary shaping must remain inside this local Anchor core/context')
    data=np.load(args.proposals/manifest['anchors'][anchor.id],allow_pickle=False)
    index=min(anchor.core_end-anchor.synth_start,len(data['tool_pose'])-1)
    target=data['tool_pose'][index]@np.linalg.inv(np.asarray(state['hand_object_transform']))
    pose0=commands[begin-1,:7];r0=Rotation.from_quat(pose0[[4,5,6,3]])
    offset=(Rotation.from_matrix(target[:3,:3])*r0.inv()).as_rotvec()
    duration=(end-begin)*dt;s=np.arange(end-begin+1)/(end-begin)
    h=10*s**3-15*s**4+6*s**5;t=s-6*s**3+8*s**4-3*s**5
    xyz=pose0[:3]+h[:,None]*(target[:3,3]-pose0[:3])+duration*t[:,None]*commands[begin-1,19:22]
    rot=Rotation.from_rotvec(h[:,None]*offset+duration*t[:,None]*commands[begin-1,22:25])*r0
    poses=np.c_[xyz,rot.as_quat()[:,[3,0,1,2]]]
    replacement=controls_from_poses(poses,np.vstack((commands[begin-1,7:19],commands[begin:end,7:19])),dt)
    commands[begin:end]=replacement
    # Post-padding supplies contact context; no reference pose is required there.
    commands[end:,:7]=commands[end-1,:7];commands[end:,19:25]=0
    args.output_dir.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(args.output_dir/'commands.npz',commands=commands)
    (args.output_dir/'proposal.json').write_text(json.dumps(dict(anchor_id=anchor.id,
        purpose='local_boundary_resynthesis_proposal_only',boundary_source_frame=anchor.core_end,
        blend_frames=args.blend_frames,target='local_source_object_boundary_with_actual_grasp',
        physically_validated=False,source_time_stretch=False,source_provenance=manifest['provenance']),indent=2))


if __name__=='__main__':main()
