"""Render recorded PhysX mesh transforms; never advance replay physics."""
import argparse,json,os
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser();p.add_argument('--run-dir',type=Path,required=True);p.add_argument('--stride',type=int,default=1);args=p.parse_args()
    import mujoco,cv2,trimesh
    from scipy.spatial.transform import Rotation
    out=args.run_dir
    geo=np.load(out/'visual_meshes.npz'); transforms=np.load(out/'visual_actual.npz')['transforms']
    trace=json.loads((out/'actual_trace.json').read_text());names=json.loads((out/'visual_mesh_paths.json').read_text())
    report=json.loads((out/'pipeline_result.json').read_text())
    if not len(trace):raise RuntimeError('No actual frames recorded')
    cache=out/'render_meshes';cache.mkdir(exist_ok=True)
    assets=[];bodies=[];rotations=[];positions=[]
    for i,name in enumerate(names):
        t=transforms[:,i];scale=np.linalg.norm(t[0,:3,:3],axis=0)
        mesh=trimesh.Trimesh(vertices=geo[f'v{i}']*scale,faces=geo[f'f{i}'],process=False)
        path=cache/f'{i}.obj';mesh.export(path)
        assets.append(f'<mesh name="m{i}" file="{path}"/>')
        color='.82 .84 .86 1'
        if name.startswith('/xhand_right'):color='.78 .32 .22 1'
        if name.startswith('/World/tool'):color='.90 .65 .20 1'
        if name.startswith('/World/target'):color='.22 .48 .78 1'
        bodies.append(f'<body name="b{i}" mocap="true"><geom type="mesh" mesh="m{i}" rgba="{color}" contype="0" conaffinity="0"/></body>')
        rotations.append(Rotation.from_matrix(t[:,:3,:3]/scale[None,None,:]).as_quat()[:,[3,0,1,2]])
        positions.append(t[:,:3,3])
    z=report['table_z']
    xml=f'''<mujoco><compiler balanceinertia="true"/><visual><global offwidth="960" offheight="720"/><quality shadowsize="2048"/></visual><asset>{''.join(assets)}</asset><worldbody>
    <light pos="1 -2 3" dir="-.2 .4 -1" diffuse=".8 .8 .8"/>
    <geom type="plane" size="3 3 .01" rgba=".16 .18 .22 1"/>
    <geom type="box" pos=".57 0 {z-.02}" size=".475 .5 .02" rgba=".48 .39 .29 1"/>
    {''.join(bodies)}</worldbody></mujoco>'''
    model=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(model)
    renderer=mujoco.Renderer(model,height=720,width=960)
    cam=mujoco.MjvCamera();cam.lookat[:]=[.4,-.03,.4];cam.distance=1.65;cam.azimuth=135;cam.elevation=-25
    positions=np.stack(positions,axis=1);rotations=np.stack(rotations,axis=1)
    video=out/'mounted_xhand_bc_curobo_actual.mp4'
    writer=cv2.VideoWriter(str(video),cv2.VideoWriter_fourcc(*'mp4v'),30/args.stride,(960,720))
    if not writer.isOpened():raise RuntimeError('VideoWriter failed')
    for frame in range(0,len(trace),args.stride):
        data.mocap_pos[:]=positions[frame];data.mocap_quat[:]=rotations[frame]
        mujoco.mj_forward(model,data);renderer.update_scene(data,camera=cam)
        bgr=renderer.render()[:,:,::-1].copy();item=trace[frame]
        cv2.putText(bgr,f"Actual PhysX | {item['mode']} | t={item['time_s']:.2f}s | source {item['source_frame']:.1f}",(15,28),cv2.FONT_HERSHEY_SIMPLEX,.57,(245,245,245),1,cv2.LINE_AA)
        cv2.putText(bgr,'Measured arm, XHand and objects; task success evaluated separately',(15,700),cv2.FONT_HERSHEY_SIMPLEX,.48,(235,235,235),1,cv2.LINE_AA)
        writer.write(bgr)
        if frame==0 or frame==len(trace)-1:cv2.imwrite(str(out/f'actual_frame_{frame:05d}.png'),bgr)
    writer.release();renderer.close()
    cap=cv2.VideoCapture(str(video));count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));ok,_=cap.read();cap.release()
    if not ok or count!=len(range(0,len(trace),args.stride)):raise RuntimeError('Video verification failed')
    print(json.dumps({'video':str(video.resolve()),'frames':count,'source':'measured PhysX mesh world transforms'}))

if __name__=='__main__':main()
