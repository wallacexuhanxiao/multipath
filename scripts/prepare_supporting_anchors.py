"""Add a second hand's local proposals to an existing oracle source.

Only padded Anchor windows are retargeted. No complete robot reference or
automatically inferred contact interval is created.
"""
import argparse,hashlib,json,shutil
from dataclasses import replace
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser()
    for name in ('schedule','proposals','object-cache','output-dir'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--existing-supporting-proposals',type=Path,help='Reuse identical local retarget arrays after oracle metadata edits')
    args=p.parse_args()
    from egoengine.taco import load_sequence,mano_forward
    from egoengine.taco.loader import HandData
    from egoengine.retarget import HandRetargeter
    from egoengine.retarget.ik import SIM_URDF
    from multipath.oracle_schedule import OracleSchedule
    from multipath.anchors.grasp_retarget import surface_grasp_prior
    import trimesh
    schedule=OracleSchedule.load(args.schedule)
    original=json.loads((args.proposals/'manifest.json').read_text())
    assignments=schedule.metadata['hands']
    if set(assignments)!={'right','left'}:
        raise ValueError('The bimanual oracle must declare both hand/object assignments')
    if args.existing_supporting_proposals:
        previous=json.loads((args.existing_supporting_proposals/'manifest.json').read_text())
        for key in ('triplet','sequence','transform_world_to_robot'):
            if previous['provenance'][key]!=original['provenance'][key]:raise ValueError('Cached local proposals use another source')
        if previous['hands']!=assignments:raise ValueError('Cached local proposals use another hand/object assignment')
        out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
        for anchor in schedule.anchors.values():
            path=args.existing_supporting_proposals/previous['anchors'][anchor.id]
            arrays=np.load(path,allow_pickle=False)
            if not np.array_equal(arrays['source_frames'],np.arange(anchor.synth_start,anchor.synth_end)):
                raise ValueError('Cached local proposals have different synthesis intervals')
            if 'left_wrist_hand_q' not in arrays:raise ValueError('Cached local proposals lack the supporting hand')
            shutil.copy2(path,out/previous['anchors'][anchor.id])
        geometry_cache=args.existing_supporting_proposals/'geometry_cache'
        if geometry_cache.exists():shutil.copytree(geometry_cache,out/'geometry_cache',dirs_exist_ok=True)
        previous['provenance']=dict(previous['provenance'],schedule_sha256=hashlib.sha256(args.schedule.read_bytes()).hexdigest())
        previous['physical_validation']=False
        (out/'manifest.json').write_text(json.dumps(previous,indent=2))
        (out/'oracle_schedule.json').write_text(args.schedule.read_text())
        print('SUPPORT_LOCAL_ARRAYS_REUSED_CURRENT_ORACLE_UNVALIDATED',flush=True)
        return
    seq=load_sequence(original['provenance']['triplet'],original['provenance']['sequence'])
    if seq.n_frames!=schedule.num_frames or abs(seq.fps-schedule.fps)>1e-5:
        raise ValueError('Oracle/source clocks differ')
    transform=np.asarray(original['provenance']['transform_world_to_robot'])
    side='left';role=assignments[side]['object_role'];hand=seq.hands[side]
    flat=HandData(side=side,pose=np.zeros((1,48),np.float32),trans=np.zeros((1,3),np.float32),
                  betas=hand.betas,frame_ids=np.zeros(1,np.int64))
    _,calibration,_=mano_forward(flat)
    meshes={r:trimesh.load(args.object_cache/o['id']/(o['id']+'_m.obj'),force='mesh')
            for r,o in original['initial_objects'].items()}
    from multipath.execution.mounted_scene import pose_matrix
    table_z=min(((pose_matrix(o['pose'])[:3,:3]@meshes[r].vertices.T).T[:,2]+o['pose'][2]).min()
                for r,o in original['initial_objects'].items())-.001
    out=args.output_dir;out.mkdir(parents=True,exist_ok=True);details={}
    for anchor in schedule.anchors.values():
        s,e=anchor.synth_start,anchor.synth_end
        arrays=dict(np.load(args.proposals/original['anchors'][anchor.id],allow_pickle=False))
        if not np.array_equal(arrays['source_frames'],np.arange(s,e)):
            raise ValueError('Existing local proposal has a different padded interval')
        local=HandData(side=side,pose=hand.pose[s:e],trans=hand.trans[s:e],betas=hand.betas,frame_ids=hand.frame_ids[s:e])
        _,joints,_=mano_forward(local);joints=joints@transform[:3,:3].T+transform[:3,3]
        reference=HandRetargeter(side,calibration[0]).retarget(joints).qpos
        contact=schedule.metadata['hand_contact_schedule'][anchor.id][side]
        local_anchor=replace(anchor,entry_contact=contact['entry'],exit_contact=contact['exit'])
        fitted,detail=surface_grasp_prior(local_anchor,arrays['source_frames'],reference,joints,arrays[role+'_pose'],
            meshes[role],SIM_URDF[side],fps=schedule.fps,table_z=table_z,geometry_cache=out/'geometry_cache',side=side)
        arrays[side+'_raw_wrist_hand_q']=reference
        arrays[side+'_wrist_hand_q']=fitted;arrays[side+'_human_joints']=joints
        np.savez_compressed(out/original['anchors'][anchor.id],**arrays)
        if detail:details[anchor.id]=detail
        print('SUPPORT_LOCAL_RETARGET',side,role,anchor.id,s,e,flush=True)
    manifest=dict(original)
    manifest['provenance']=dict(original['provenance'],schedule_sha256=hashlib.sha256(args.schedule.read_bytes()).hexdigest(),
                                hand_assignments=assignments,local_supporting_retarget=True)
    manifest.update(hands=assignments,supporting_retarget_details=details,physical_validation=False)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    (out/'oracle_schedule.json').write_text(args.schedule.read_text())


if __name__=='__main__':main()
