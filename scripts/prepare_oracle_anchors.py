"""Retarget ONLY padded oracle intervals directly from human TACO observations."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from multipath.oracle_schedule import OracleSchedule


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--schedule',type=Path,required=True)
    p.add_argument('--triplet',required=True)
    p.add_argument('--sequence',required=True)
    p.add_argument('--mount-report',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--object-cache',type=Path,help='Optional existing meshes for local object-aware grasp retargeting.')
    args=p.parse_args()
    from egoengine.taco import load_sequence, mano_forward, compute_pseudo_base
    from egoengine.taco.loader import HandData
    from egoengine.retarget import HandRetargeter
    schedule=OracleSchedule.load(args.schedule)
    seq=load_sequence(args.triplet,args.sequence)
    if seq.n_frames!=schedule.num_frames or abs(seq.fps-schedule.fps)>1e-5:
        raise ValueError('Oracle/source clocks differ')
    out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    mount=json.loads(args.mount_report.read_text())
    transform=np.linalg.inv(compute_pseudo_base(seq))
    transform[:3,3]+=np.array(mount['scene_translation_xyz'])
    hand=seq.hands['right']
    flat=HandData(side='right',pose=np.zeros((1,48),np.float32),trans=np.zeros((1,3),np.float32),betas=hand.betas,frame_ids=np.zeros(1,np.int64))
    _,calibration,_=mano_forward(flat)
    provenance={'triplet':args.triplet,'sequence':args.sequence,'input':'raw TACO MANO and object poses',
                'source_video':seq.video_path,'schedule_sha256':hashlib.sha256(args.schedule.read_bytes()).hexdigest(),
                'full_robot_reference_used':False,'vlm_used':False,'transform_world_to_robot':transform.tolist()}
    initial={}; final={}
    for role,obj in seq.objects.items():
        def pose(frame):
            t=transform@obj.poses[frame]
            return np.r_[t[:3,3],Rotation.from_matrix(t[:3,:3]).as_quat()[[3,0,1,2]]].tolist()
        initial[role]={'id':obj.object_id,'pose':pose(0)}
        final[role]=pose(seq.n_frames-1)
    paths={};retarget_details={};table_z=None
    if args.object_cache:
        import trimesh
        table_z=min(((transform@obj.poses[0])[:3,:3]@
            trimesh.load(args.object_cache/obj.object_id/(obj.object_id+'_m.obj'),force='mesh').vertices.T).T[:,2].min()
            +(transform@obj.poses[0])[2,3] for obj in seq.objects.values())-.001
    for anchor in schedule.anchors.values():
        s,e=anchor.synth_start,anchor.synth_end
        local=HandData(side='right',pose=hand.pose[s:e],trans=hand.trans[s:e],betas=hand.betas,frame_ids=hand.frame_ids[s:e])
        _,joints,_=mano_forward(local)
        joints=joints@transform[:3,:3].T+transform[:3,3]
        result=HandRetargeter('right',calibration[0]).retarget(joints)
        arrays={'source_frames':np.arange(s,e),'wrist_hand_q':result.qpos,'human_joints':joints}
        arrays['raw_wrist_hand_q']=np.array(result.qpos,copy=True)
        for role,obj in seq.objects.items():arrays[role+'_pose']=transform[None]@obj.poses[s:e]
        if args.object_cache:
            import trimesh
            from egoengine.retarget.ik import SIM_URDF
            from multipath.anchors.grasp_retarget import surface_grasp_prior
            oid=initial['tool']['id'];mesh=trimesh.load(args.object_cache/oid/(oid+'_m.obj'),force='mesh')
            arrays['wrist_hand_q'],detail=surface_grasp_prior(anchor,arrays['source_frames'],arrays['wrist_hand_q'],
                joints,arrays['tool_pose'],mesh,SIM_URDF['right'],fps=schedule.fps,table_z=table_z,
                geometry_cache=out/'geometry_cache')
            if detail:retarget_details[anchor.id]=detail
        path=out/(anchor.id+'.npz');np.savez_compressed(path,**arrays)
        paths[anchor.id]=path.name
        print(anchor.id,s,e,'retargeted',flush=True)
    manifest={'provenance':provenance,'initial_objects':initial,'final_objects':final,
              'anchors':paths,'mount_xyzw':mount['candidate_mount_quaternion_xyzw'],
              'interaction_constraints':schedule.metadata.get('interaction_constraints',{}),
              'retarget_details':retarget_details,
              'fps':schedule.fps,'num_frames':schedule.num_frames,
              'physical_validation':False,'kind':'local_kinematic_proposals'}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    (out/'oracle_schedule.json').write_text(args.schedule.read_text())

if __name__=='__main__':main()
