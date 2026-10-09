"""Add contact Jacobian initialization to existing local kinematic proposals."""
import argparse,json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser()
    for name in ('schedule','proposals','xhand-urdf','object-cache','output-report'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    import numpy as np,trimesh
    from multipath.oracle_schedule import OracleSchedule
    from multipath.anchors.grasp_retarget import tool_surface_distance
    from multipath.anchors.pinch import contact_linearization
    from multipath.execution.mounted_scene import pose_matrix
    schedule=OracleSchedule.load(args.schedule);path=args.proposals/'manifest.json'
    manifest=json.loads(path.read_text());tool_id=manifest['initial_objects']['tool']['id']
    mesh=trimesh.load(args.object_cache/tool_id/(tool_id+'_m.obj'),force='mesh')
    distance=tool_surface_distance(mesh,args.proposals/'geometry_cache');report={}
    for anchor in schedule.anchors.values():
        detail=manifest.get('retarget_details',{}).get(anchor.id)
        if detail is None or (anchor.entry_contact,anchor.exit_contact)!=('free','grasped'):continue
        data=np.load(args.proposals/manifest['anchors'][anchor.id],allow_pickle=False)
        closed=data['wrist_hand_q'][anchor.core_end-anchor.synth_start-1]
        tool=data['tool_pose'][anchor.core_start-anchor.synth_start]
        pinch=contact_linearization(args.xhand_urdf,distance,np.linalg.inv(tool)@pose_matrix(closed[:7]),
            closed[7:],detail['opposition_finger'])
        detail['pinch_proposal']=pinch;report[anchor.id]=pinch
    path.write_text(json.dumps(manifest,indent=2));args.output_report.parent.mkdir(parents=True,exist_ok=True)
    args.output_report.write_text(json.dumps(dict(anchors=report,physical_acceptance=False,
        command_trajectories_modified=False,simulator_states_modified=False),indent=2))


if __name__=='__main__':main()
