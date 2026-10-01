"""Source-time execution of a held-bowl tilt, return, and release sequence.

No phase inserts simulation time. New spatial plans must fit their source
interval; reference replay and physical measurements share the same clock.
"""
import numpy as np
import json
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from .path_exclusion import exclusion_segments, exclusions_for_window, protect_live_wrist_boundaries, compare_path, transport_points


def run_source_clock(*, args, report, ref, schedule, robot, arm, fingers, wrist,
                     objects, translation, mount, ik, tensor, planner_robot,
                     table, meshes, step, records, out):
    from curobo.types.math import Pose
    from curobo.types.robot import JointState
    from curobo.geom.sdf.world import CollisionCheckerType
    from curobo.wrap.reacher.motion_gen import MotionGen,MotionGenConfig,MotionGenPlanConfig
    source=ref[args.source_wrist]
    semantics=json.loads(args.schedule.read_text())
    release_frame=int(semantics['release_frame'])
    contact_frame=int(semantics['grasp_contact_frame'])
    closure_frame=int(semantics['grasp_closed_frame'])
    approach_start=schedule.bc_windows[0].start
    descent_end=int(semantics['precontact_end_frame'])
    ticks_per_frame=120/schedule.fps
    if abs(ticks_per_frame-round(ticks_per_frame))>1e-8:
        raise ValueError('Source fps must divide the 120 Hz physics clock')
    ticks_per_frame=int(round(ticks_per_frame))
    measured=ref['qpos_right_actual']
    target_f=measured[:,7:]+args.finger_command_blend*(ref['qpos_right'][:,7:]-measured[:,7:])
    weight=np.clip((np.arange(len(source))-contact_frame)/(closure_frame-contact_frame),0,1)
    for finger in (0,1,4,5,6,7,8,9,10,11):target_f[:,finger]+=weight*args.grasp_closure
    if args.grasp_hold_frame is not None:
        target_f[args.grasp_hold_frame:release_frame]=target_f[args.grasp_hold_frame].copy()
    if semantics.get('prepare_open_hand_before_descent',False):
        target_f[:descent_end]=measured[descent_end,7:].copy()
    xyz=source[:,:3]+translation
    wr=Rotation.from_quat(source[:,[4,5,6,3]])
    gt=ref['object_pose_gt_tool'].copy();gt[:,:3,3]+=translation
    # Match the settled physical support height once, never reset a handoff.
    gt[:,:3,3]+=np.asarray(objects['tool'].get_world_pose()[0])-gt[0,:3,3]
    gtr=Rotation.from_matrix(gt[:,:3,:3])
    mount_r=Rotation.from_quat(mount)
    limits=np.array([2.175]*4+[2.61]*3)
    velocity_fraction=float(semantics.get('physical_velocity_fraction',.98))
    robot._articulation_view.set_max_joint_velocities(values=(limits*velocity_fraction)[None],joint_indices=arm)
    exclusions=report.get('path_exclusions')
    (out/'source_clock_config.json').write_text(json.dumps(semantics,indent=2))
    module_dir=__import__('pathlib').Path(__file__).parent
    for filename in ('source_clock.py','path_exclusion.py','diverse_planning.py','grasp_hold.py'):
        (out/('implementation_'+filename)).write_text((module_dir/filename).read_text())
    report['source_clock_config']=semantics
    report['physical_velocity_fraction']=velocity_fraction
    lower=np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-.0175,-2.8973])
    upper=np.array([2.8973,1.7628,2.8973,-.0698,2.8973,3.7525,2.8973])
    report.update({'source_clock':True,'source_duration_s':len(source)/schedule.fps,
                   'source_wrist':args.source_wrist,
                   'semantic_contract':'hold bowl through tilt and return; release at source return location',
                   'old_place_on_target_predicate_disabled':True,'windows':[]})
    qlast=np.asarray(robot.get_joint_positions())[arm].copy()
    held=None;relative_p=None;relative_r=None;grasp_offset=np.zeros(3)
    fidelity=[];joint_speeds=[];retention=[];rotation_retention=[]
    applied_q=qlast.copy();applied_f=np.asarray(robot.get_joint_positions())[fingers].copy();applied_speeds=[];actual_speeds=[]
    grip_config=semantics.get('grasp_feedback',{})
    grip_preload=0.;grip_samples=[]
    finger_limits=np.asarray(robot._articulation_view.get_dof_limits())[0,fingers]
    def controlled_step(q,f,mode,frame):
        nonlocal applied_q,applied_f,grip_preload
        if grip_config.get('enabled',False) and relative_p is not None and frame<release_frame:
            from .grasp_hold import update_preload
            wp,wq=wrist.get_world_pose();op,oq=objects['tool'].get_world_pose()
            rotation=Rotation.from_quat(np.asarray(wq)[[1,2,3,0]])
            object_rotation=Rotation.from_quat(np.asarray(oq)[[1,2,3,0]])
            pd=float(np.linalg.norm(rotation.inv().apply(np.asarray(op)-wp)-relative_p))
            rd=float((relative_r.inv()*rotation.inv()*object_rotation).magnitude())
            options={k:v for k,v in grip_config.items() if k not in ('enabled','closing_joints')}
            grip_preload=update_preload(grip_preload,pd,rd,1/120,**options)
            f=np.asarray(f).copy()
            f[np.asarray(grip_config['closing_joints'],dtype=int)]+=grip_preload
            f=np.clip(f,finger_limits[:,0],finger_limits[:,1])
            grip_samples.append({'source_frame':frame,'preload_rad':grip_preload,'position_drift_m':pd,'rotation_drift_rad':rd})
        dq=np.clip(q-applied_q,-.90*limits/120,.90*limits/120)
        df=np.clip(f-applied_f,-6/120,6/120)
        applied_q=applied_q+dq;applied_f=applied_f+df
        applied_speeds.append(float(np.max(np.abs(dq)*120/limits)))
        step(applied_q,applied_f,mode,frame,velocity=np.r_[dq*120,np.zeros(12)])
        actual_velocity=np.asarray(robot.get_joint_velocities())[arm].copy()
        ratio=float(np.max(np.abs(actual_velocity)/limits))
        actual_speeds.append(ratio)
        if ratio>report.get('measured_velocity_peak',{}).get('ratio',0):
            report['measured_velocity_peak']={'ratio':ratio,'source_frame':frame,
                                              'controller':mode,'arm_velocity_rad_s':actual_velocity.tolist()}

    def fk(q):
        s=ik.kinematics.get_state(tensor.to_device(q.copy())[None])
        return s.ee_position.detach().cpu().numpy()[0].copy(), Rotation.from_quat(s.ee_quaternion.detach().cpu().numpy()[0][[1,2,3,0]])

    def solve(position,rotation,seed):
        # Local least-squares IK penalizes null-space jumps between frames.
        # This avoids stretching the clock to accommodate unrelated IK branches.
        def residual(q):
            p,r=fk(q)
            return np.r_[p-position,.15*(rotation*r.inv()).as_rotvec(),.001*(q-seed)]
        fit=least_squares(residual,np.clip(seed,lower+1e-6,upper-1e-6),bounds=(lower,upper),
                          diff_step=2e-3,max_nfev=60,xtol=1e-7,ftol=1e-7,gtol=1e-7)
        p,r=fk(fit.x);err=np.linalg.norm(p-position);angle=(rotation*r.inv()).magnitude()
        if err>.015 or angle>.15:raise RuntimeError(f'Source-time local IK error {err:.4f}m {angle:.3f}rad')
        return fit.x

    def desired(frame):
        if relative_p is not None and frame>=release_frame:
            # Keep the returned bowl pose while opening; then withdraw the
            # hand continuously. Switching back to the old floating wrist
            # here would drag the bowl because the mounted grasp differs.
            object_rotation=gtr[release_frame-1]
            wrist_rotation=object_rotation*relative_r.inv()
            position=gt[release_frame-1,:3,3]+grasp_offset-wrist_rotation.apply(relative_p)
            position[2]+=max(0.,-grasp_offset[2])
            away=-wrist_rotation.apply(relative_p);away[2]=0
            away/=max(np.linalg.norm(away),1e-8)
            progress=np.clip((frame-release_frame-3)/9,0,1)
            position+=progress*(.13*away+[0,0,.04])
            return position,wrist_rotation*mount_r.inv()
        if relative_p is not None and frame<release_frame:
            lead=int(semantics.get('tilt_control_lead_frames',0))
            blend=np.clip((frame-schedule.bc_windows[1].start)/4,0,1)*np.clip((release_frame-frame)/8,0,1)
            object_rotation=gtr[min(len(gtr)-1,frame+int(round(lead*blend)))]
            wrist_rotation=object_rotation*relative_r.inv()
            position=gt[frame,:3,3]+grasp_offset-wrist_rotation.apply(relative_p)
            # Compensate the measured acquisition height deficit smoothly
            # while approaching the interaction, without altering its clock.
            phase=np.clip((frame-schedule.bc_windows[0].end)/10.,0,1)
            position[2]+=phase*max(0.,-grasp_offset[2])
            for window in schedule.bc_windows[1:]:
                if window.start<=frame<window.start+4:
                    position[2]+=.06*(window.start+4-frame)/4
            return position,wrist_rotation*mount_r.inv()
        position=xyz[frame].copy()
        if closure_frame<=frame<schedule.bc_windows[0].end:
            position[2]+=.025*np.clip((frame-closure_frame)/12,0,1)
        # Spend the existing approach frames on the short contact descent;
        # do not append a new approach interval to the source clock.
        if approach_start<=frame<descent_end:
            position[2]+=.08*(descent_end-frame)/(descent_end-approach_start)
        return position,wr[frame]*mount_r.inv()

    def measure(frame):
        wp,wq=wrist.get_world_pose();op,oq=objects['tool'].get_world_pose()
        orot=Rotation.from_quat(np.asarray(oq)[[1,2,3,0]])
        angle=float(np.arccos(np.clip(orot.apply([0,0,1])@gtr[0].apply([0,0,1]),-1,1)))
        entry={'frame':frame,'measured_time_s':(frame+1)/schedule.fps,'bowl_xyz':np.asarray(op).tolist(),'tilt_rad':angle}
        fidelity.append(entry)
        if relative_p is not None and frame<release_frame:
            current=Rotation.from_quat(np.asarray(wq)[[1,2,3,0]])
            drift=np.linalg.norm(current.inv().apply(np.asarray(op)-wp)-relative_p)
            retention.append(float(drift))
            rotation_retention.append(float((relative_r.inv()*current.inv()*orot).magnitude()))

    for wi,w in enumerate(schedule.windows):
        info={'controller':w.controller,'start':w.start,'end':w.end,'budget_s':(w.end-w.start)/schedule.fps}
        speed_start=len(joint_speeds)
        report['windows'].append(info);print('SOURCE_WINDOW',info,flush=True)
        if w.start==schedule.bc_windows[0].end:
            wp,wq=wrist.get_world_pose();op,oq=objects['tool'].get_world_pose()
            current=Rotation.from_quat(np.asarray(wq)[[1,2,3,0]])
            relative_p=current.inv().apply(np.asarray(op)-wp)
            relative_r=current.inv()*Rotation.from_quat(np.asarray(oq)[[1,2,3,0]])
            held=target_f[w.start-1].copy()
            grasp_offset=np.asarray(op)-gt[w.start,:3,3]
            report['grasp_entry_object_offset_xyz']=grasp_offset.tolist()
            report['grasp_relative_position']=relative_p.tolist()
            actual_lift=float(op[2]-gt[0,2,3])
            report['acquisition_lift_m']=actual_lift
            if actual_lift<.025:
                raise RuntimeError(f'Source-clock acquisition did not lift bowl: {actual_lift:.4f}m')
        active_exclusions=exclusions_for_window(exclusions,w.start,w.end)
        info['active_virtual_exclusion_window']=[w.start,w.end] if active_exclusions else None
        if w.controller=='CUROBO':
            endpoint=min(w.end,len(source)-1)
            p,r=desired(endpoint)
            qstart=np.asarray(robot.get_joint_positions())[arm].copy()
            carry=held is not None and w.start<release_frame
            cfg,_=planner_robot(carry)
            live={'cuboid':{'table':table}}
            for role in (['target'] if carry else ['tool','target']):
                op,oq=objects[role].get_world_pose();rot=Rotation.from_quat(np.asarray(oq)[[1,2,3,0]])
                bounds=meshes[role].bounds
                live['cuboid'][role]={'dims':(bounds[1]-bounds[0]).tolist(),'pose':np.r_[op+rot.apply(bounds.mean(0)),oq].tolist()}
            if active_exclusions and exclusions.get('exclusion_scope','robot')=='robot':
                live['cuboid'].update(active_exclusions['cuboid'])
                info['virtual_path_obstacle_count']=len(active_exclusions['cuboid'])
            planner=MotionGen(MotionGenConfig.load_from_robot_config(cfg,live,tensor_args=tensor,interpolation_dt=1/120,
                       self_collision_check=True,collision_checker_type=CollisionCheckerType.PRIMITIVE,use_cuda_graph=False))
            start=JointState.from_position(tensor.to_device(qstart)[None],joint_names=planner.joint_names)
            spheres=planner.compute_kinematics(start).robot_spheres.detach().cpu().numpy().reshape(-1,4)
            hits=[]
            for name,box in live['cuboid'].items():
                pose=np.asarray(box['pose']);rot=Rotation.from_quat(pose[[4,5,6,3]])
                local=rot.inv().apply(spheres[:,:3]-pose[:3])
                distances=np.linalg.norm(np.maximum(np.abs(local)-np.asarray(box['dims'])/2,0),axis=1)-spheres[:,3]
                hits.extend([name,int(i),float(distances[i]),spheres[i].tolist()] for i in np.flatnonzero((distances<0)&(spheres[:,3]>0)))
            info['start_world_overlap']=sorted(hits,key=lambda x:x[2])[:12]
            np.savez_compressed(out/f'planner_start_spheres_{wi}.npz',spheres=spheres)
            (out/f'source_world_{wi}.json').write_text(json.dumps(live,indent=2))
            if active_exclusions and exclusions.get('exclusion_scope')=='wrist':
                from .diverse_planning import plan_wrist_detour
                anchor=np.asarray(active_exclusions['boundary_wrist_xyz_wxyz'])
                p=anchor[:3].copy();r=Rotation.from_quat(anchor[[4,5,6,3]])*mount_r.inv()
                live_wrist=np.asarray(wrist.get_world_pose()[0])
                adjusted=protect_live_wrist_boundaries(active_exclusions,live_wrist,p)
                active_exclusions.update(adjusted)
                (out/'resolved_path_exclusions.json').write_text(json.dumps(exclusions,indent=2))
                raw,search=plan_wrist_detour(planner,start,p,r,active_exclusions,tensor,info['budget_s'],solve,
                                              goal_joints=active_exclusions.get('boundary_arm_q'))
                info['boundary_target']='accepted_baseline_measured_arm_and_wrist' if 'boundary_arm_q' in active_exclusions else 'accepted_baseline_measured_wrist'
                info['detour_search']=search
                info['virtual_path_obstacle_count']=len(active_exclusions['cuboid'])
                info['plan_status']='wrist_detour_found'
            else:
                qgoal=solve(p,r,qstart)
                goal=JointState.from_position(tensor.to_device(qgoal)[None],joint_names=planner.joint_names)
                goal_ok,goal_status=planner.check_start_state(goal)
                info['local_goal_valid']=bool(goal_ok);info['local_goal_status']=str(goal_status)
                if goal_ok:
                    result=planner.plan_single_js(start,goal,MotionGenPlanConfig(max_attempts=12,enable_graph_attempt=3))
                else:
                    # A smooth local IK solution can still collide. Search other
                    # arm configurations for the same semantic wrist pose.
                    pose=Pose(position=tensor.to_device(p.copy())[None],quaternion=tensor.to_device(r.as_quat()[[3,0,1,2]])[None])
                    result=planner.plan_single(start,pose,MotionGenPlanConfig(max_attempts=16,enable_graph_attempt=3))
                info['plan_status']=str(result.status)
                if not bool(result.success.item()):raise RuntimeError(f'Source-clock cuRobo {w.start}:{w.end} failed: {result.status}')
                raw=result.get_interpolated_plan().position.detach().cpu().numpy().reshape(-1,7)
            ticks=(w.end-w.start)*ticks_per_frame
            if active_exclusions and exclusions.get('exclusion_scope')=='wrist':
                from .diverse_planning import resample_with_velocity_budget
                path=resample_with_velocity_budget(raw,ticks,.90*limits,dt=1/120)
            else:
                path=np.stack([np.interp(np.linspace(0,len(raw)-1,ticks),np.arange(len(raw)),raw[:,j]) for j in range(7)],axis=1)
            qprevious=qstart.copy();fstart=np.asarray(robot.get_joint_positions())[fingers].copy()
            for j,q in enumerate(path):
                frame=w.start+j/ticks_per_frame
                fq=held if carry else (1-j/max(1,ticks-1))*fstart+j/max(1,ticks-1)*target_f[min(endpoint,len(source)-1)]
                joint_speeds.append(float(np.max(np.abs(q-qprevious)*120/limits)));qprevious=q.copy()
                controlled_step(q,fq,'CUROBO',frame)
                if j%ticks_per_frame==ticks_per_frame-1:measure(int(frame))
            qlast=path[-1].copy()
            endstate=JointState.from_position(tensor.to_device(np.asarray(robot.get_joint_positions())[arm].copy())[None],joint_names=planner.joint_names)
            end_spheres=planner.compute_kinematics(endstate).robot_spheres.detach().cpu().numpy().reshape(-1,4)
            np.savez_compressed(out/f'planner_end_spheres_{wi}.npz',spheres=end_spheres)
            if active_exclusions:
                info['path_diversity']=compare_path(transport_points(records,w.start,w.end),active_exclusions)
        else:
            planned=[];seed=qlast.copy()
            for frame in range(w.start,w.end):
                p,r=desired(frame);seed=solve(p,r,seed);planned.append(seed.copy())
            last=qlast.copy()
            for k,q in enumerate(planned):
                frame=w.start+k
                fq=held if held is not None and frame<release_frame else target_f[frame]
                if held is not None and frame>=release_frame:
                    opened=np.zeros(12);opened[2:4]=held[2:4]
                    blend=np.clip((frame-release_frame+1)/3,0,1)
                    fq=held*(1-blend)+opened*blend
                for sub in range(ticks_per_frame):
                    command=last+(q-last)*(sub+1)/ticks_per_frame
                    joint_speeds.append(float(np.max(np.abs(q-last)*schedule.fps/limits)))
                    controlled_step(command,fq,'BC',frame+sub/ticks_per_frame)
                measure(frame);last=q.copy()
            qlast=last
        info['end_time_s']=w.end/schedule.fps
        info['max_joint_speed_limit_ratio']=max(joint_speeds[speed_start:],default=0)
        report['source_clock_measurements']=fidelity
        report['measured_max_joint_speed_limit_ratio']=max(actual_speeds,default=0)
        (out/'source_clock_progress.json').write_text(__import__('json').dumps(report,indent=2))
    trace=np.array([x['bowl_xyz'] for x in fidelity]);tilt=np.array([x['tilt_rad'] for x in fidelity])
    expected_tilt=np.arccos(np.clip(gt[:,:3,2]@gt[0,:3,2],-1,1))
    peak=int(tilt.argmax());expected_peak=int(expected_tilt.argmax())
    final_error=float(np.linalg.norm(trace[-1]-gt[-1,:3,3]))
    wp,_=wrist.get_world_pose();op,_=objects['tool'].get_world_pose()
    release_distance=float(np.linalg.norm(np.asarray(wp)-op))
    final_drift=float(np.linalg.norm(trace[-10:]-trace[-1],axis=1).max())
    released=relative_p is not None and release_distance>np.linalg.norm(relative_p)+.04
    success=bool(tilt.max()>.8 and abs(peak-expected_peak)<=8 and final_error<.05 and retention and max(retention)<.05
                 and max(rotation_retention,default=999)<.35 and released and final_drift<.005)
    timing_ratio=max(actual_speeds,default=0)
    report.update({'status':'source_clock_complete','frames_recorded':len(records),'duration_s':len(source)/schedule.fps,
                   'source_clock_measurements':fidelity,'max_joint_speed_limit_ratio':timing_ratio,
                   'unfiltered_target_speed_limit_ratio':max(joint_speeds,default=0),
                   'peak_tilt_rad':float(tilt.max()),'peak_tilt_frame':peak,
                   'peak_tilt_timestamp_s':(peak+1)/schedule.fps,'peak_tilt_video_frame':peak+1,'reference_peak_tilt_frame':expected_peak,
                   'final_reference_position_error_m':final_error,'max_held_object_relative_drift_m':max(retention,default=999),
                   'max_held_object_rotation_drift_rad':max(rotation_retention,default=999),
                   'grasp_feedback_samples':grip_samples,'grasp_feedback_max_preload_rad':grip_preload,
                   'released_at_end':bool(released),'final_object_stability_m':final_drift,
                   'task_success':success,'timing_limits_pass':timing_ratio<=1.05,
                   'reference_semantics_validated':success,
                   'full_pipeline_success':success and timing_ratio<=1.05})

    if exclusions:
        checks=[w['path_diversity']['actual_wrist_exclusion_pass'] for w in report['windows'] if 'path_diversity' in w]
        report['diverse_solution_success']=bool(report['full_pipeline_success'] and len(checks)==len(exclusion_segments(exclusions)) and all(checks))
