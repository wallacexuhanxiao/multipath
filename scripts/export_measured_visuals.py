"""Render-only mesh transforms from actual floating PhysX states and URDF FK.

Useful when a saved actual-state artifact lacks its native mesh recording.
The template supplies visual geometry only. No simulation or reference poses
are executed; robot q and all object poses come from the measured trace.
"""
import argparse,json,shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def main():
    p=argparse.ArgumentParser()
    for name in ('trace','entry','template','xhand-urdf','output-dir'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--fps',type=float,required=True)
    args=p.parse_args();out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    from multipath.execution.mounted_scene import pose_matrix
    from multipath.planning.hand_geometry import joint_model,link_poses
    from multipath.xhand import RIGHT_XHAND_FINGER_JOINT_NAMES
    states=json.loads(args.trace.read_text());entry=json.loads(args.entry.read_text())
    entry=dict(entry,mode='INITIAL',source_frame=entry['time_s']*args.fps)
    states=[entry]+[s for s in states if s['time_s']>entry['time_s']]
    if not all(s.get('physically_acquired_state') and s.get('embodiment')=='floating_xhand' for s in states):
        raise ValueError('Only actual floating PhysX states can be rendered')
    times=np.array([s['time_s'] for s in states])
    if not np.allclose(np.diff(times),1/args.fps,atol=1e-8,rtol=0):
        raise ValueError('Measured trace does not match the source frame clock')
    names=json.loads((args.template/'visual_mesh_paths.json').read_text())
    template=np.load(args.template/'visual_actual.npz')['transforms'][0]
    template_state=json.loads((args.template/'actual_trace.json').read_text())[0]
    model=joint_model(args.xhand_urdf);origins={}
    for link in ET.parse(args.xhand_urdf).getroot().findall('link'):
        origin=link.find('visual/origin');m=np.eye(4)
        if origin is not None:
            m[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
            m[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
        origins[link.get('name')]=m
    transforms=[]
    for state in states:
        fk=link_poses(model,dict(zip(RIGHT_XHAND_FINGER_JOINT_NAMES,state['hand_q'])))
        wrist=pose_matrix(state['wrist_xyz_wxyz']);row=[]
        for i,name in enumerate(names):
            if name.startswith('/xhand_right/'):
                link=name.split('/')[2];m=wrist@fk[link]@origins[link]
                m[:3,:3]*=np.linalg.norm(template[i,:3,:3],axis=0)[None,:]
            elif any(name.startswith('/World/'+role+'/') for role in state['objects']):
                role=next(role for role in state['objects'] if name.startswith('/World/'+role+'/'))
                local=np.linalg.inv(pose_matrix(template_state['objects'][role]['pose']))@template[i]
                m=pose_matrix(state['objects'][role]['pose'])@local
            else:m=template[i].copy()
            row.append(m)
        transforms.append(row)
    np.savez_compressed(out/'visual_actual.npz',transforms=np.asarray(transforms))
    for name in ('visual_meshes.npz','visual_mesh_paths.json'):shutil.copyfile(args.template/name,out/name)
    (out/'actual_trace.json').write_text(json.dumps(states))
    template_report=json.loads((args.template/'pipeline_result.json').read_text())
    (out/'pipeline_result.json').write_text(json.dumps(dict(embodiment='floating_xhand',source='oracle',
        source_fps=args.fps,actual_duration_s=float(times[-1]-times[0]),table_z=template_report['table_z'],
        full_pipeline_success=False,validation_scope='local actual-state artifact visualization',
        renderer_runs_physics=False,reconstructed_from_measured_states=True,
        hand_geometry=str(args.xhand_urdf),visual_geometry_template=str(args.template),
        input_measured_trace=str(args.trace)),indent=2))
    print(json.dumps(dict(frames=len(states)-1,duration_s=float(times[-1]-times[0]),
                         source='measured PhysX q/object poses; URDF visual FK')))


if __name__=='__main__':main()
