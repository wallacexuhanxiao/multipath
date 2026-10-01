"""Align a TACO object reference to its XHand retarget frame for source-clock runs.

The paired MANO joints determine one rigid coordinate transform. This only
builds a kinematic reference; the mounted arm and grasp require PhysX replay.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def rigid_alignment(source, target):
    source=np.asarray(source,dtype=float).reshape(-1,3)
    target=np.asarray(target,dtype=float).reshape(-1,3)
    if source.shape!=target.shape or len(source)<3:
        raise ValueError('Paired MANO joints must have matching Nx3 shapes')
    a=source.mean(axis=0);b=target.mean(axis=0)
    u,_,vt=np.linalg.svd((source-a).T@(target-b))
    rotation=u@np.diag([1.,1.,np.linalg.det(u@vt)])@vt
    offset=b-a@rotation
    errors=np.linalg.norm(source@rotation+offset-target,axis=1)
    return rotation,offset,errors


def convert_poses(poses, rotation, offset):
    result=np.asarray(poses,dtype=float).copy()
    result[:,:3,:3]=rotation.T@result[:,:3,:3]
    result[:,:3,3]=result[:,:3,3]@rotation+offset
    return result


def pose7(poses):
    return np.c_[poses[:,:3,3],Rotation.from_matrix(poses[:,:3,:3]).as_quat()]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--retarget',type=Path,required=True)
    parser.add_argument('--taco-reference',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    with np.load(args.retarget,allow_pickle=False) as hand,np.load(args.taco_reference,allow_pickle=False) as taco:
        n=len(hand['qpos_right'])
        if len(taco['tool_pose_robot'])!=n or len(taco['right_mano_joints_robot'])!=n:
            raise ValueError('Retarget and TACO reference must share the original source frames')
        rotation,offset,error=rigid_alignment(taco['right_mano_joints_robot'],hand['mano_joints_base_right'])
        if np.percentile(error,99)>.01:
            raise ValueError(f'MANO coordinate alignment exceeds 1 cm: {np.percentile(error,99):.4f} m')
        tool=convert_poses(taco['tool_pose_robot'],rotation,offset)
        target=convert_poses(taco['target_pose_robot'],rotation,offset)
        qright=hand['qpos_right'].copy();qleft=hand['qpos_left'].copy()
        if qright.shape!=(n,19) or qleft.shape!=(n,19):raise ValueError('Expected floating XHand qpos19')
        args.output.parent.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(args.output,qpos_right=qright,qpos_right_actual=qright,
                            qpos_left=qleft,qpos_left_actual=qleft,
                            object_pose_gt_tool=tool,object_pose_gt_target=target,
                            object_pose_actual_tool_xyzw=pose7(tool),
                            object_pose_actual_target_xyzw=pose7(target),frames=np.arange(n),
                            tool_mesh_id=taco['tool_mesh_id'],target_mesh_id=taco['target_mesh_id'])
        metadata={'source_kind':'kinematic retarget and TACO object poses; not a physical rollout',
                  'retarget':str(args.retarget),'taco_reference':str(args.taco_reference),
                  'retarget_sha256':hashlib.sha256(args.retarget.read_bytes()).hexdigest(),
                  'taco_reference_sha256':hashlib.sha256(args.taco_reference.read_bytes()).hexdigest(),
                  'frames':n,'tool_mesh_id':str(taco['tool_mesh_id']),
                  'target_mesh_id':str(taco['target_mesh_id']),
                  'alignment_rotation_row':rotation.tolist(),'alignment_offset_xyz':offset.tolist(),
                  'mano_alignment_mean_m':float(error.mean()),
                  'mano_alignment_p99_m':float(np.percentile(error,99))}
    args.output.with_suffix('.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps({'output':str(args.output.resolve()),**metadata}))


if __name__=='__main__':main()
