"""Refit a local grasp proposal against a physically reached tool/table frame."""
import argparse,json
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser()
    for name in ('schedule','proposals','entry-run','xhand-urdf','object-cache','output-dir'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--anchor-id',required=True);args=p.parse_args()
    from multipath.oracle_schedule import OracleSchedule
    from multipath.anchors.grasp_retarget import surface_grasp_prior
    from multipath.execution.mounted_scene import pose_matrix
    import trimesh
    schedule=OracleSchedule.load(args.schedule);a=schedule.anchors[args.anchor_id]
    manifest=json.loads((args.proposals/'manifest.json').read_text())
    physical=json.loads((args.entry_run/'pipeline_result.json').read_text())
    state=json.loads((args.entry_run/'cores'/a.id/'entry.json').read_text())
    if (physical['source_provenance']!=manifest['provenance'] or
            not state.get('physically_acquired_state') or state['contact']['mode']!='free' or
            physical.get('state_resets_between_segments')!=0 or
            not np.isclose(state['time_s']*schedule.fps,a.core_start)):
        raise ValueError('Grasp geometry refit requires this source\'s real continuous free CORE entry')
    path=args.proposals/manifest['anchors'][a.id];data=dict(np.load(path,allow_pickle=False))
    oid=manifest['initial_objects']['tool']['id']
    mesh=trimesh.load(args.object_cache/oid/(oid+'_m.obj'),force='mesh')
    reference=data.get('raw_wrist_hand_q',data['wrist_hand_q']).copy()
    if ('raw_wrist_hand_q' not in data and manifest.get('retarget_details',{}).get(a.id,{}).get(
            'pre_acquisition_tool_motion')=='stationary_entry'):
        # Legacy local proposals are expressed in the stationary entry tool
        # frame. Restore source-relative coordinates before fitting again;
        # otherwise the future lift is mistakenly added to the wrist error.
        from scipy.spatial.transform import Rotation
        entry=data['tool_pose'][a.core_start-a.synth_start]
        matrices=np.asarray([tool@np.linalg.inv(entry)@pose_matrix(wrist)
            for tool,wrist in zip(data['tool_pose'],reference[:,:7])])
        reference[:,:3]=matrices[:,:3,3]
        reference[:,3:7]=Rotation.from_matrix(matrices[:,:3,:3]).as_quat()[:,[3,0,1,2]]
    data['wrist_hand_q'],detail=surface_grasp_prior(a,data['source_frames'],reference,data['human_joints'],
        data['tool_pose'],mesh,args.xhand_urdf,fps=schedule.fps,table_z=physical['table_z'],
        physical_entry_tool=pose_matrix(state['objects']['tool']['pose']),geometry_cache=args.proposals/'geometry_cache')
    if detail is None:raise ValueError('This local proposal has no surface-grasp geometry refit')
    detail['physical_geometry_entry']=dict(entry_origin=state['entry_origin'],
        entry_provenance=state['entry_provenance'],tool_pose=state['objects']['tool']['pose'],
        table_z=physical['table_z'],source_frame=a.core_start)
    manifest['retarget_details'][a.id]=detail
    np.savez_compressed(path,**data);(args.proposals/'manifest.json').write_text(json.dumps(manifest,indent=2))
    args.output_dir.mkdir(parents=True,exist_ok=True)
    (args.output_dir/'geometry_result.json').write_text(json.dumps(dict(detail,proposal_refitted=True,
        physical_acceptance=False,oracle_schedule_changed=False,source_time_stretch=False,
        simulator_state_assigned=False),indent=2))
    print('PHYSICAL_GEOMETRY_REFIT',a.id,detail['free_pregrasp_surface_clearance_m'],flush=True)


if __name__=='__main__':main()
