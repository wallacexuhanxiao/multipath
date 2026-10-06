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
    fps=float(report.get('source_fps',30.))
    video_frames=len(trace)
    if report.get('source')=='oracle':
        # N+1 measured boundaries describe N source-frame time intervals.
        video_frames=min(video_frames,max(1,int(np.ceil(report['actual_duration_s']*fps-1e-8))))
    endpoint_samples=report.get('embodiment')=='floating_xhand' and trace and trace[0]['mode']=='INITIAL'
    if endpoint_samples:
        # Initial plus N endpoint samples describe N source-time intervals.
        video_frames=min(video_frames,len(trace)-1)
    exclusions=report.get('path_exclusions')
    overlays=[];overlay_names=[]
    if exclusions:
        for wi,segment in enumerate(exclusions.get('segments',[exclusions])):
            for key,box in segment['cuboid'].items():
                name=f'w{wi}_{key}'
                xyz=' '.join(map(str,box['pose'][:3]));half=' '.join(str(x/2) for x in box['dims'])
                quat=' '.join(map(str,box['pose'][3:]))
                overlays.append(f'<geom name="{name}" type="box" pos="{xyz}" quat="{quat}" size="{half}" rgba="1 .15 .55 .4" contype="0" conaffinity="0"/>')
                overlay_names.append((name,.4,segment['window']))
            for si,source in enumerate(segment['sources']):
                points=source['wrist_path_xyz'][::3]
                for j,(a,b) in enumerate(zip(points,points[1:])):
                    name=f'old_path_{wi}_{si}_{j}';coords=' '.join(map(str,[*a,*b]))
                    if np.linalg.norm(np.asarray(a)-b)<1e-6:continue
                    overlays.append(f'<geom name="{name}" type="capsule" fromto="{coords}" size=".002" rgba=".1 .9 1 .65" contype="0" conaffinity="0"/>')
                    overlay_names.append((name,.65,segment['window']))
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
    {''.join(bodies)}{''.join(overlays)}</worldbody></mujoco>'''
    model=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(model)
    renderer=mujoco.Renderer(model,height=720,width=960)
    cam=mujoco.MjvCamera();cam.lookat[:]=[.4,-.03,.4];cam.distance=1.65;cam.azimuth=135;cam.elevation=-25
    positions=np.stack(positions,axis=1);rotations=np.stack(rotations,axis=1)
    floating=report.get('embodiment')=='floating_xhand'
    video=out/('floating_xhand_actual.mp4' if floating else 'mounted_xhand_bc_curobo_actual.mp4')
    writer=cv2.VideoWriter(str(video),cv2.VideoWriter_fourcc(*'mp4v'),fps/args.stride,(960,720))
    if not writer.isOpened():raise RuntimeError('VideoWriter failed')
    for frame in range(0,video_frames,args.stride):
        # After a development restore, the INITIAL USD mesh capture can lag
        # the native PhysX state. N executed endpoint captures still represent
        # N source intervals at the same fps; use those measured captures.
        sample=frame+1 if endpoint_samples else frame
        item=trace[sample]
        if exclusions:
            active=False
            for name,alpha,window in overlay_names:
                visible=item['mode']=='CUROBO' and window[0]<=item['source_frame']<=window[1]
                active=active or visible
                model.geom_rgba[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,name),3]=alpha if visible else 0
        data.mocap_pos[:]=positions[sample];data.mocap_quat[:]=rotations[sample]
        mujoco.mj_forward(model,data);renderer.update_scene(data,camera=cam)
        bgr=renderer.render()[:,:,::-1].copy();item=trace[sample]
        cv2.putText(bgr,f"Actual PhysX | {item['mode']} | t={item['time_s']:.2f}s | source {item['source_frame']:.1f}",(15,28),cv2.FONT_HERSHEY_SIMPLEX,.57,(245,245,245),1,cv2.LINE_AA)
        caption='Measured floating XHand and objects; local experiment' if floating else 'Measured arm, XHand and objects; task success evaluated separately'
        if report.get('full_pipeline_success'):
            caption='Actual PhysX | Oracle Anchors + cuRobo | fixed source clock | task passed'
        cv2.putText(bgr,caption,(15,700),cv2.FONT_HERSHEY_SIMPLEX,.48,(235,235,235),1,cv2.LINE_AA)
        if exclusions and active:
            cv2.putText(bgr,'Pink: virtual planning obstacles | Cyan: prior measured wrist path',(15,53),cv2.FONT_HERSHEY_SIMPLEX,.5,(245,200,235),1,cv2.LINE_AA)
        writer.write(bgr)
        if frame==0 or frame==video_frames-1:cv2.imwrite(str(out/f'actual_frame_{frame:05d}.png'),bgr)
    writer.release();renderer.close()
    cap=cv2.VideoCapture(str(video));count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));ok,_=cap.read();cap.release()
    if not ok or count!=len(range(0,video_frames,args.stride)):raise RuntimeError('Video verification failed')
    print(json.dumps({'video':str(video.resolve()),'frames':count,'source':'measured PhysX mesh world transforms'}))

if __name__=='__main__':main()
