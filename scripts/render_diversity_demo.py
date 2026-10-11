"""Presentation overlays of successful actual rollouts; slow playback, never physics."""
import argparse
import json
import hashlib
import os
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SIZE = (1920, 1080)
COLORS = [(239, 212, 53), (172, 101, 245), (87, 200, 255)]  # BGR
BG = (18, 23, 30)


def text(image, words, xy, scale=.7, color=(224, 230, 237), thickness=1):
    cv2.putText(image, words, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color,
                thickness, cv2.LINE_AA)


def camera_projection(camera, size):
    # The GL camera is obtained without rendering or advancing any simulation.
    import mujoco
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody/></mujoco>')
    data = mujoco.MjData(model)
    cam = mujoco.MjvCamera()
    for key, value in camera.items():
        if key == 'lookat':
            cam.lookat[:] = value
        else:
            setattr(cam, key, value)
    scene = mujoco.MjvScene(model, maxgeom=1)
    mujoco.mjv_updateScene(model, data, mujoco.MjvOption(), None, cam,
                          mujoco.mjtCatBit.mjCAT_ALL, scene)
    eye = np.mean([c.pos for c in scene.camera], axis=0)
    forward, up = scene.camera[0].forward, scene.camera[0].up
    right = np.cross(forward, up)
    focal = size[1] * scene.camera[0].frustum_near / (
        scene.camera[0].frustum_top - scene.camera[0].frustum_bottom)

    def project(points):
        delta = np.asarray(points) - eye
        depth = delta @ forward
        if np.any(depth <= 0):
            raise ValueError('Recorded wrist is behind the render camera')
        return np.c_[size[0]/2 + focal*(delta @ right)/depth,
                     size[1]/2 - focal*(delta @ up)/depth]
    return project


def load_case(case):
    runs, captures, traces, reports = [], [], [], []
    offset = case.get('recorded_video_boundary_offset', 0)
    for index,name in enumerate(case['run_dirs']):
        run = (ROOT / name).resolve()
        report = json.loads((run/'pipeline_result.json').read_text())
        trace = json.loads((run/'actual_trace.json').read_text())
        if not (report.get('full_pipeline_success') and report.get('task_success')
                and report.get('state_resets_between_segments') == 0):
            raise ValueError(f'{run}: not a successful uninterrupted rollout')
        if reports and report['source_provenance'] != reports[0]['source_provenance']:
            raise ValueError('Overlay requires the same source and world registration')
        video = Path(case['video_paths'][index]) if case.get('video_paths') else run/'floating_xhand_actual.mp4'
        cap = cv2.VideoCapture(str(video))
        count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if not cap.isOpened() or count+offset > len(trace) or fps <= 0:
            raise ValueError(f'{run}: missing actual video or trace samples')
        if not np.allclose([r['time_s'] for r in trace], np.arange(len(trace))/fps, atol=1e-6):
            raise ValueError('Actual trace does not match the recorded video clock')
        if reports and (count != runs[0]['frames'] or fps != runs[0]['fps'] or size != runs[0]['size']):
            raise ValueError('Compared videos must share dimensions, fps and source duration')
        if not np.isclose(count/fps, report['actual_duration_s']):
            raise ValueError('Video is not a complete source-duration clip')
        reports.append(report); traces.append(trace); captures.append(cap)
        runs.append(dict(path=str(run), video_path=str(video.resolve()),frames=count, fps=fps, size=size,
                         actual_trace_sha256=hashlib.sha256((run/'actual_trace.json').read_bytes()).hexdigest()))
    windows = [(s['start'], s['end']) for s in reports[0]['segments'] if s['mode'] == 'RELAXED']
    schedule = [(s['start'], s['end'], s['mode']) for s in reports[0]['segments']]
    if any([(s['start'], s['end'], s['mode']) for s in r['segments']] != schedule for r in reports):
        raise ValueError('Successful solutions use different temporal schedules')
    xyz = [np.asarray([r['wrist_xyz_wxyz'][:3] for r in trace]) for trace in traces]
    return dict(label=case['label'], runs=runs, reports=reports, traces=traces,
                captures=captures, offset=offset, windows=windows, schedule=schedule, xyz=xyz)


def crop_bounds(case, project, aspect):
    points = list(case['xyz'])
    for trace in case['traces']:
        for role in trace[0]['objects']:
            points.append(np.asarray([r['objects'][role]['pose'][:3] for r in trace]))
    pixels = project(np.concatenate(points))
    input_width,input_height=case['runs'][0]['size']
    padding=np.asarray([input_width*.115,input_height*.118])
    lo, hi = pixels.min(0)-padding, pixels.max(0)+padding
    center = (lo+hi)/2
    top,bottom=input_height*.089,input_height*.903
    width = min(float(input_width), max(float(hi[0]-lo[0]), float(hi[1]-lo[1])*aspect, input_width*.48))
    height = width/aspect
    if height > bottom-top:
        height = bottom-top; width = height*aspect
    x = np.clip(center[0]-width/2, 0, input_width-width)
    y = np.clip(center[1]-height/2, top, bottom-height)
    return tuple(map(int, [x, y, x+width, y+height]))


def path_card(paths, window, size):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    start, end = window
    pieces = [p[start:end+1] for p in paths]
    joined = np.concatenate(pieces)
    center = joined.mean(0)
    _, _, basis = np.linalg.svd(joined-center, full_matrices=False)
    coords = [(p-center) @ basis[:2].T*1000 for p in pieces]
    fig, ax = plt.subplots(figsize=(size[0]/100, size[1]/100), dpi=100)
    fig.patch.set_facecolor('#171e28'); ax.set_facecolor('#171e28')
    for xy, color in zip(coords, COLORS):
        rgb = np.asarray(color[::-1])/255
        ax.plot(*xy.T, color=rgb, alpha=.55, lw=1.7)
        ax.scatter(*xy[0], color=[rgb], s=14, marker='o')
        ax.scatter(*xy[-1], color=[rgb], s=23, marker='x')
    span = np.ptp(np.concatenate(coords), axis=0)
    margin = max(float(span.max())*.12, .8)
    all_xy = np.concatenate(coords)
    ax.set_xlim(all_xy[:,0].min()-margin, all_xy[:,0].max()+margin)
    ax.set_ylim(all_xy[:,1].min()-margin, all_xy[:,1].max()+margin)
    ax.set_aspect('equal', adjustable='box')
    ax.set_title(f'cuRobo [{start}, {end})', color='#e3e9f1', fontsize=11, pad=6)
    ax.tick_params(colors='#9daabe', labelsize=8, length=2)
    for spine in ax.spines.values():spine.set_color('#4b5669')
    ax.grid(alpha=.15, color='#bec7d5')
    ax.set_xlabel('Shared motion plane (mm)', color='#b8c4d4', fontsize=8, labelpad=2)
    fig.subplots_adjust(left=.16, right=.94, top=.83, bottom=.23)
    fig.canvas.draw()
    background = np.asarray(fig.canvas.buffer_rgba())[:,:,:3][:,:,::-1].copy()
    pixels = [np.c_[ax.transData.transform(xy)[:,0], size[1]-1-ax.transData.transform(xy)[:,1]]
              for xy in coords]
    plt.close(fig)
    return background, pixels


def draw_cards(canvas, cards, case, source_sample, current, boxes):
    for index, (background, paths) in enumerate(cards):
        tile = background.copy(); start, end = case['windows'][index]
        if start <= source_sample <= end and current == index:
            length = int(source_sample-start)+1
            for path, color in zip(paths, COLORS):
                xy = np.rint(path[:length]).astype(np.int32)
                if len(xy)>1:cv2.polylines(tile,[xy],False,color,3,cv2.LINE_AA)
                cv2.circle(tile,tuple(xy[-1]),5,color,-1,cv2.LINE_AA)
            cv2.rectangle(tile,(1,1),(tile.shape[1]-2,tile.shape[0]-2),(184,225,180),2)
        x,y,w,h = boxes[index];canvas[y:y+h,x:x+w] = tile


def scene_view(images, case, project, frame, current, box, overlay):
    x,y,w,h = box
    crop = crop_bounds(case, project, w/h)
    x0,y0,x1,y1 = crop
    image = np.mean(images,axis=0).astype(np.uint8) if overlay else images[0].copy()
    view = cv2.resize(image[y0:y1,x0:x1],(w,h),interpolation=cv2.INTER_AREA)
    if overlay:
        for image,color in zip(images,COLORS):
            b,g,r = cv2.split(image[y0:y1,x0:x1])
            mask = ((r>60)&(r.astype(float)>g*1.75)&(r.astype(float)>b*1.6)).astype(np.uint8)*255
            mask=cv2.resize(mask,(w,h),interpolation=cv2.INTER_NEAREST)
            contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(view,[c for c in contours if cv2.contourArea(c)>60],-1,color,1,cv2.LINE_AA)
    if current is not None:
        start,_ = case['windows'][current]
        sample = frame+case['offset']
        for points,color in zip(case['xyz'],COLORS):
            pixels = (project(points[start:sample+1])-[x0,y0])*[w/(x1-x0),h/(y1-y0)]
            pixels = np.rint(pixels).astype(np.int32)
            if len(pixels)>1:cv2.polylines(view,[pixels],False,color,3,cv2.LINE_AA)
            cv2.circle(view,tuple(pixels[-1]),6,color,-1,cv2.LINE_AA)
    return view


def header(canvas, case, frame, current, slow):
    fps = case['runs'][0]['fps']; sample=frame+case['offset']
    time = case['traces'][0][sample]['time_s']
    text(canvas,'Interaction-Anchored Trajectory Generation',(24,38),1.0,thickness=2)
    text(canvas,f"{case['label']}  |  {len(case['runs'])} successful actual PhysX rollouts  |  no segment resets",(24,75),.7)
    mode = f'cuRobo bridge {current+1}  |  {1/slow:g}x playback' if current is not None else 'Interaction Anchor  |  1x playback'
    text(canvas,mode,(1110,36),.75,(180,233,191),2)
    text(canvas,f"Source time {time:.2f} / {case['reports'][0]['actual_duration_s']:.2f} s",(1330,74),.7)
    for i,color in enumerate(COLORS[:len(case['runs'])]):
        text(canvas,f'Solution {i+1}',(24+i*230,110),.65,color,2)
    text(canvas,'Measured wrist paths; playback speed only',(1130,110),.65)
    for start,end,mode in case['schedule']:
        a=int(24+1872*start/case['runs'][0]['frames']);b=int(24+1872*end/case['runs'][0]['frames'])
        cv2.rectangle(canvas,(a,1038),(b-3,1056),(106,126,56) if mode=='RELAXED' else (75,81,92),-1)
        if b-a>65:text(canvas,'MP' if mode=='RELAXED' else 'A',(a+6,1052),.37)
    cursor=int(24+1872*(frame+.5)/case['runs'][0]['frames'])
    cv2.line(canvas,(cursor,1032),(cursor,1060),(255,255,255),2)


def writer(path, fps):
    output = cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'avc1'),fps,SIZE)
    if not output.isOpened():raise RuntimeError('Cannot open H.264 video writer')
    return output


def verify_video(path, expected):
    cap=cv2.VideoCapture(str(path));fps=cap.get(cv2.CAP_PROP_FPS);count=0
    while cap.read()[0]:count+=1
    cap.release()
    if count!=expected:raise RuntimeError(f'{path}: decoded {count}, expected {expected}')
    return dict(path=str(path.resolve()),frames=count,fps=fps,duration_s=count/fps)


def clean_recorded_videos(spec, out):
    from concurrent.futures import ThreadPoolExecutor
    jobs=[]
    for case in spec['cases']:
        if not case.get('clean_render_runs'):continue
        case['video_paths']=[]
        for index,(name,raw) in enumerate(zip(case['run_dirs'],case['clean_render_runs'])):
            run=(ROOT/name).resolve();raw=(ROOT/raw).resolve()
            if hashlib.sha256((run/'actual_trace.json').read_bytes()).digest()!=hashlib.sha256((raw/'actual_trace.json').read_bytes()).digest():
                raise ValueError('Clean renderer must use the identical successful actual trace')
            video=out/'input_actual'/f"{case['label']}_solution_{index+1}.mp4"
            case['video_paths'].append(str(video));jobs.append((raw,video))
        case['recorded_video_boundary_offset']=0
    def render(job):
        raw,video=job
        env=dict(os.environ,MUJOCO_GL='cgl' if sys.platform=='darwin' else 'egl')
        subprocess.run([sys.executable,str(ROOT/'scripts/render_mounted_actual.py'),
                        '--run-dir',str(raw),'--hide-planning-overlays','--output',str(video)],
                       env=env,check=True,stdout=subprocess.DEVNULL)
        print('CLEAN_ACTUAL_READY',video.name,flush=True)
    with ThreadPoolExecutor(max_workers=2) as pool:
        list(pool.map(render,jobs))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--cases',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--slow-factor',type=int,default=4)
    args=parser.parse_args()
    if args.slow_factor<1:parser.error('slow-factor must be positive')
    spec=json.loads(args.cases.read_text());out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    clean_recorded_videos(spec,out)
    cases=[load_case(case) for case in spec['cases']]
    fps=cases[0]['runs'][0]['fps']
    if any(c['runs'][0]['fps']!=fps for c in cases):raise ValueError('Showcase cases require a shared video fps')
    combined_paths={layout:out/f'showcase_{layout}.mp4' for layout in ('overlay','stacked')}
    combined={k:writer(v,fps) for k,v in combined_paths.items()}
    total=0;reports=[];posters=[]
    for case in cases:
        case_dir=out/case['label'];case_dir.mkdir(exist_ok=True)
        paths={layout:case_dir/f'{layout}_slow_paths.mp4' for layout in combined}
        outputs={k:writer(v,fps) for k,v in paths.items()}
        project=camera_projection(spec['render_camera'],case['runs'][0]['size'])
        number=len(case['windows']);card_height=min(210,int(868/number)-8)
        card_width=int((1872-12*(number-1))/number)
        overlay_boxes=[(1434,140+i*(card_height+8),462,card_height) for i in range(number)]
        stacked_boxes=[(24+i*(card_width+12),650,card_width,360) for i in range(number)]
        overlay_cards=[path_card(case['xyz'],w,(462,card_height)) for w in case['windows']]
        stacked_cards=[path_card(case['xyz'],w,(card_width,360)) for w in case['windows']]
        separation=np.max([np.linalg.norm(a-b,axis=1) for i,a in enumerate(case['xyz'])
                           for b in case['xyz'][i+1:]],axis=0)
        start,end=case['windows'][0]
        preview=int(start+np.argmax(separation[start:end]))-case['offset']
        frame_map=[];written=0
        for frame in range(case['runs'][0]['frames']):
            images=[]
            for cap in case['captures']:
                ok,image=cap.read()
                if not ok:raise RuntimeError('Missing actual input frame')
                images.append(image)
            current=next((i for i,(a,b) in enumerate(case['windows']) if a<=frame<b),None)
            repeats=args.slow_factor if current is not None else 1
            canvases={layout:np.full((SIZE[1],SIZE[0],3),BG,np.uint8) for layout in combined}
            for canvas in canvases.values():header(canvas,case,frame,current,args.slow_factor)
            box=(24,140,1380,868)
            canvases['overlay'][140:1008,24:1404]=scene_view(images,case,project,frame,current,box,True)
            text(canvases['overlay'],f"{len(images)} separate actual rollouts overlaid",(40,990),.65)
            column=(SIZE[0]-48-24*(len(images)-1))//len(images)
            for i,image in enumerate(images):
                box=(24+i*(column+24),165,column,450)
                view=scene_view([image],case,project,frame,None,box,False)
                x,y,w,h=box;canvases['stacked'][y:y+h,x:x+w]=view
                text(canvases['stacked'],f'Solution {i+1} | task passed',(x,150),.65,COLORS[i],2)
            sample=frame+case['offset']
            draw_cards(canvases['overlay'],overlay_cards,case,sample,current,overlay_boxes)
            draw_cards(canvases['stacked'],stacked_cards,case,sample,current,stacked_boxes)
            for layout,canvas in canvases.items():
                for _ in range(repeats):outputs[layout].write(canvas);combined[layout].write(canvas)
                if frame==preview:
                    cv2.imwrite(str(case_dir/f'{layout}_preview.png'),canvas)
                    if layout=='overlay':posters.append(canvas.copy())
                if frame==end//2 and layout=='overlay':cv2.imwrite(str(case_dir/'approach.png'),canvas)
                if current is not None and current>0 and frame==case['windows'][current][0] and layout=='overlay':
                    cv2.imwrite(str(case_dir/f'bridge_{current+1}.png'),canvas)
                if frame==case['runs'][0]['frames']-1:cv2.imwrite(str(case_dir/f'{layout}_terminal.png'),canvas)
            frame_map.append(dict(output_frame=written,repeat=repeats,source_sample=sample,
                                  source_time_s=case['traces'][0][sample]['time_s'],mode='RELAXED' if current is not None else 'ANCHOR'))
            written+=repeats
        for cap in case['captures']:cap.release()
        for output in outputs.values():output.release()
        report=dict(source=case['reports'][0]['source_provenance'],actual_source_duration_s=case['reports'][0]['actual_duration_s'],
                    input_runs=case['runs'],recorded_video_boundary_offset=case['offset'],physics_retimed=False,
                    relaxed_playback_speed=1/args.slow_factor,anchor_playback_speed=1.,
                    windows=case['windows'],frame_map=frame_map,
                    videos={layout:verify_video(path,written) for layout,path in paths.items()})
        (case_dir/'demo_report.json').write_text(json.dumps(report,indent=2)+'\n')
        reports.append(report);total+=written
        print(json.dumps(dict(case=case['label'],frames=written,seconds=written/fps)),flush=True)
    for output in combined.values():output.release()
    cv2.imwrite(str(out/'successful_cases.png'),cv2.vconcat(posters))
    report=dict(cases=reports,videos={layout:verify_video(path,total) for layout,path in combined_paths.items()},
                successful_case_figure=str(out/'successful_cases.png'),source='actual recorded PhysX states')
    (out/'demo_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report['videos']),flush=True)


if __name__=='__main__':main()
