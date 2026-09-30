"""Run VLM BC windows and cuRobo bridges in one mounted-XHand PhysX scene.

Only the initial state is assigned. Every subsequent state is simulated. BC
uses the existing XHand finger commands and measured wrist path with explicit
synchronized time scaling. cuRobo starts each bridge from measured arm state.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
from isaacsim import SimulationApp


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--reference', type=Path, required=True)
    p.add_argument('--schedule', type=Path, required=True)
    p.add_argument('--mount-report', type=Path, required=True)
    p.add_argument('--robot-yaml', type=Path, required=True)
    p.add_argument('--xhand-urdf', type=Path, required=True)
    p.add_argument('--object-cache', type=Path, required=True)
    p.add_argument('--path-exclusions', type=Path)
    p.add_argument('--source-clock', action='store_true', help='Preserve original tilt-return-release semantics and source duration')
    p.add_argument('--source-wrist',choices=['qpos_right','qpos_right_actual','qpos_right_reference'],default='qpos_right')
    p.add_argument('--grasp-closure',type=float,default=0.)
    p.add_argument('--repaired-grasp',type=Path)
    p.add_argument('--object-relative-bc',action='store_true')
    p.add_argument('--grasp-hold-frame',type=int)
    p.add_argument('--arm-kp', type=float, default=0.)
    p.add_argument('--physics-friction', type=float, default=0.)
    p.add_argument('--grasp-only', action='store_true')
    p.add_argument('--place-correction-json', type=Path)
    p.add_argument('--start-at-bc', action='store_true', help='Isolate grasp replay from its approach')
    p.add_argument('--finger-command-blend', type=float, default=1.0)
    p.add_argument('--output-dir', type=Path, required=True)
    args = p.parse_args()
    out=args.output_dir; out.mkdir(parents=True, exist_ok=True)
    (out/'execution_arguments.json').write_text(json.dumps(vars(args),default=str,indent=2))
    app=SimulationApp({'headless':True})
    report={'status':'running','task_success':False,'full_pipeline_success':False,
            'reference_semantics_validated':False,
            'grasp_only_diagnostic':args.grasp_only,'windows':[], 'state_resets_between_windows':0}
    recorder=None; records=[]
    try:
        import torch, yaml, trimesh
        from scipy.spatial.transform import Rotation
        from curobo.geom.sdf.world import CollisionCheckerType
        from curobo.types.base import TensorDeviceType
        from curobo.types.math import Pose
        from curobo.types.robot import JointState
        from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
        from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import FixedCuboid
        from isaacsim.core.prims import SingleRigidPrim
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.utils.types import ArticulationAction
        from pxr import UsdGeom, UsdPhysics, UsdShade
        from multipath.schedule import Schedule,Window
        from multipath.mounted_xhand import assemble,configure_drives,ActualMeshRecorder
        from multipath.bc_trajectory import retime_bc
        from multipath.path_exclusion import transport_points,compare_path,exclusion_segments
        schedule=Schedule.load(args.schedule)
        exclusions=json.loads(args.path_exclusions.read_text()) if args.path_exclusions else None
        if exclusions:
            if exclusions['kind']!='virtual_planner_obstacles':
                raise ValueError('Invalid virtual path exclusions')
            for segment in exclusion_segments(exclusions):
                if not segment['cuboid'] or not any([w.start,w.end]==segment['window'] for w in schedule.curobo_windows):
                    raise ValueError('Exclusion window must match a cuRobo schedule interval')
            report['path_exclusions']=exclusions
        ref=dict(np.load(args.reference,allow_pickle=False))
        config=json.loads(args.mount_report.read_text())
        translation=np.asarray(config['scene_translation_xyz'])
        mount=np.asarray(config['candidate_mount_quaternion_xyzw'])
        source=ref['qpos_right_actual']
        finger_targets=source[:,7:]+args.finger_command_blend*(ref['qpos_right'][:,7:]-source[:,7:])
        if args.grasp_hold_frame is not None:
            held=finger_targets[args.grasp_hold_frame].copy()
            finger_targets[args.grasp_hold_frame:schedule.bc_windows[1].start]=held
            report['grasp_hold_frame']=args.grasp_hold_frame
        goals_xyz=source[:,:3]+translation
        goals_q=(Rotation.from_quat(source[:,[4,5,6,3]])*Rotation.from_quat(mount).inv()).as_quat()[:,[3,0,1,2]]
        initial_objects={r:np.array(ref[f'object_pose_actual_{r}_xyzw'][0],dtype=float) for r in ('tool','target')}
        meshes={r:trimesh.load(args.object_cache/id/(id+'_m.obj'),force='mesh') for r,id in [('tool','180'),('target','049')]}
        for value in initial_objects.values(): value[:3]+=translation
        table_z=min((Rotation.from_quat(initial_objects[r][3:]).apply(meshes[r].vertices)+initial_objects[r][:3])[:,2].min() for r in meshes)-.001
        table={'dims':[.95,1.0,.04],'pose':[.57,0,float(table_z-.02),1,0,0,0]}
        world_config={'cuboid':{'table':table}}
        robot_config=yaml.safe_load(args.robot_yaml.read_text())['robot_cfg']
        world=World(stage_units_in_meters=1.,physics_dt=1/120,rendering_dt=1/30)
        world.scene.add_default_ground_plane()
        world.scene.add(FixedCuboid(prim_path='/World/Table', name='table',position=np.asarray(table['pose'][:3]),scale=np.asarray(table['dims'])))
        robot,wrist,flange=assemble(world,args.xhand_urdf,mount)
        objects={}
        for role,id in [('tool','180'),('target','049')]:
            path='/World/'+role
            add_reference_to_stage(str(args.object_cache/id/(id+'.usd')),path)
            pose=initial_objects[role]
            objects[role]=world.scene.add(SingleRigidPrim(prim_path=path,name=role,position=pose[:3],orientation=pose[[6,3,4,5]]))
        if args.physics_friction:
            mat=UsdShade.Material.Define(world.stage,'/World/XHandPhysicsMaterial')
            props=UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
            props.CreateStaticFrictionAttr(args.physics_friction)
            props.CreateDynamicFrictionAttr(args.physics_friction)
            props.CreateRestitutionAttr(0.)
            for root in ('/World','/xhand_right'):
                UsdShade.MaterialBindingAPI.Apply(world.stage.GetPrimAtPath(root)).Bind(mat,bindingStrength='strongerThanDescendants',materialPurpose='physics')
        world.reset()
        arm,fingers=configure_drives(robot)
        if args.arm_kp:
            robot._articulation_view.set_gains(kps=np.full((1,7),args.arm_kp),kds=np.full((1,7),80.),joint_indices=arm)
        report['arm_gains']=[np.asarray(x).tolist() for x in robot._articulation_view.get_gains()]
        report['physics_friction_override']=args.physics_friction

        tensor=TensorDeviceType(device='cuda:0')
        ik=IKSolver(IKSolverConfig.load_from_robot_config(robot_config,world_config,tensor_args=tensor,num_seeds=40,
                    position_threshold=.005,rotation_threshold=.05,self_collision_check=True,self_collision_opt=True,
                    collision_checker_type=CollisionCheckerType.PRIMITIVE,use_cuda_graph=False))
        def pose_at(frame, offset=None):
            position=goals_xyz[frame].copy()
            if offset is not None:position+=offset
            return Pose(position=tensor.to_device(position)[None],quaternion=tensor.to_device(goals_q[frame].copy())[None])
        repaired=dict(np.load(args.repaired_grasp)) if args.repaired_grasp else None
        if repaired:
            first=schedule.bc_windows[0].start
            goals_xyz[first]=repaired['initial_wrist_xyz_wxyz'][:3]
            qr=Rotation.from_quat(repaired['initial_wrist_xyz_wxyz'][[4,5,6,3]])*Rotation.from_quat(mount).inv()
            goals_q[first]=qr.as_quat()[[3,0,1,2]]
            report['repaired_grasp_source']=str(args.repaired_grasp)
        goal=Pose(position=tensor.to_device(goals_xyz.copy()),quaternion=tensor.to_device(goals_q.copy()))
        solved=ik.solve_batch(goal)
        valid=solved.success.detach().cpu().numpy().reshape(-1)
        batchq=solved.solution.detach().cpu().numpy().reshape(len(source),-1)
        needed=set()
        for w in schedule.windows:
            if w.controller=='BC':needed.update(range(w.start,w.end))
        missing=sorted(i for i in needed if not valid[i])
        if missing and not args.source_clock:raise RuntimeError(f'IK failed at required frames {missing}')
        # Seed each BC frame from its predecessor to limit null-space jumps.
        arm_reference=batchq.copy()
        first_bc=schedule.bc_windows[0].start
        ready_goal=Pose(position=tensor.to_device(goals_xyz[first_bc]+[0,0,.2])[None],
                        quaternion=tensor.to_device(goals_q[first_bc])[None])
        ready_solution=ik.solve_single(ready_goal)
        if not bool(ready_solution.success.item()):raise RuntimeError('Elevated ready pose is unreachable')
        ready_q=ready_solution.solution.detach().cpu().numpy().reshape(-1)
        arm_reference[0]=ready_q
        arm_reference[-1]=ready_q
        report['sequential_ik_fallback_frames']=[]
        for w in ([] if args.source_clock else schedule.bc_windows):
            prev=tensor.to_device(batchq[w.start])
            for i in range(w.start,w.end):
                sol=ik.solve_single(pose_at(i),
                                    retract_config=prev[None],seed_config=prev[None,None],num_seeds=10)
                if bool(sol.success.item()):
                    prev=sol.solution.detach().reshape(-1)
                else:
                    # The independently checked solution remains valid; its
                    # larger joint motion is explicitly handled by retiming.
                    prev=tensor.to_device(batchq[i])
                    report['sequential_ik_fallback_frames'].append(i)
                arm_reference[i]=prev.cpu().numpy()
        np.savez_compressed(out/'arm_bc_ik.npz',arm_q=arm_reference,goals_xyz=goals_xyz,goals_quat_wxyz=goals_q)
        qinit=np.asarray(robot.get_joint_positions()).copy()
        initial_frame=schedule.bc_windows[0].start if args.start_at_bc else 0
        qinit[arm]=arm_reference[initial_frame];qinit[fingers]=source[initial_frame,7:]
        robot.set_joint_positions(qinit)
        robot.set_joint_velocities(np.zeros_like(qinit))
        robot.apply_action(ArticulationAction(joint_positions=qinit))
        for _ in range(120):world.step(render=False)
        recorder=ActualMeshRecorder(world.stage,out)
        sim_steps=0
        last_command=np.r_[qinit[arm],qinit[fingers]].copy()
        report['source_sha256']=__import__('hashlib').sha256(args.reference.read_bytes()).hexdigest()
        if exclusions:
            scene={'source_sha256':report['source_sha256'],'mount_xyzw':mount.tolist(),'scene_translation_xyz':translation.tolist()}
            if scene!=exclusions['scene']:raise ValueError('Exclusions do not match current scene/reference')
        report['pbs_job_id']=__import__('os').environ.get('PBS_JOBID','unknown')
        def capture(mode,frame,command):
            wp,wq=wrist.get_world_pose()
            item={'time_s':sim_steps/120,'mode':mode,'source_frame':float(frame),
                  'arm_q':np.asarray(robot.get_joint_positions())[arm].tolist(),
                  'finger_q':np.asarray(robot.get_joint_positions())[fingers].tolist(),
                  'wrist_xyz_wxyz':np.r_[wp,wq].tolist(), 'command':np.asarray(command).tolist()}
            for r,obj in objects.items():
                op,oq=obj.get_world_pose(); item[r+'_xyz_wxyz']=np.r_[op,oq].tolist()
            records.append(item);recorder.capture()
        def step(aq,fq,mode,frame,velocity=None):
            nonlocal sim_steps,last_command
            last_command=np.r_[aq,fq].copy()
            robot.apply_action(ArticulationAction(joint_positions=np.r_[aq,fq],joint_velocities=velocity,joint_indices=np.r_[arm,fingers]))
            world.step(render=False);sim_steps+=1
            if sim_steps%4==0 and (not args.source_clock or sim_steps<len(source)*4):
                capture(mode,sim_steps/4 if args.source_clock else frame,np.r_[aq,fq])
        # Build conservative XHand spheres from each current link's actual mesh.
        def planner_robot(carry):
            import copy
            from pxr import Usd
            cfg=copy.deepcopy(robot_config); kin=cfg['kinematics']
            spheres=yaml.safe_load((args.robot_yaml.parent/kin['collision_spheres']).read_text())['collision_spheres']
            for name in ('panda_leftfinger','panda_rightfinger'):
                kin['collision_link_names'].remove(name);spheres.pop(name,None)
            kin['lock_joints']={}
            for key in ('joint_names','retract_config','null_space_weight','cspace_distance_weight'):
                kin['cspace'][key]=kin['cspace'][key][:7]
            kin['self_collision_ignore']={k:[n for n in v if n in kin['collision_link_names']] for k,v in kin['self_collision_ignore'].items() if k in kin['collision_link_names']}
            kin['self_collision_buffer']={k:v for k,v in kin['self_collision_buffer'].items() if k in kin['collision_link_names']}
            fp,fq=flange.get_world_pose();fr=Rotation.from_quat(np.asarray(fq)[[1,2,3,0]])
            extras=[];cache=UsdGeom.XformCache()
            for prim in Usd.PrimRange.Stage(world.stage, Usd.TraverseInstanceProxies()):
                if not str(prim.GetPath()).startswith('/xhand_right') or not prim.IsA(UsdGeom.Mesh):continue
                if 'collision' in str(prim.GetPath()).lower():continue
                points=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
                trans=np.asarray(cache.GetLocalToWorldTransform(prim)).T
                points=points@trans[:3,:3].T+trans[:3,3]
                points=fr.inv().apply(points-np.asarray(fp))
                # Oriented boxes split along their longest axis, covered by spheres.
                center=points.mean(0);_,_,axes=np.linalg.svd(points-center,full_matrices=False)
                coords=(points-center)@axes.T;lo=coords.min(0);hi=coords.max(0)
                # Cover small oriented cells instead of a few oversized
                # spheres that protrude far beyond flat palm geometry.
                counts=np.maximum(1,np.ceil((hi-lo)/.025).astype(int))
                width=(hi-lo)/counts
                radius=float(np.linalg.norm(width)*.5+.001)
                for ijk in np.ndindex(*counts):
                    c=lo+(np.asarray(ijk)+.5)*width
                    extras.append({'center':(center+c@axes).tolist(),'radius':radius})
            if carry:
                op,oq=objects['tool'].get_world_pose();points=Rotation.from_quat(np.asarray(oq)[[1,2,3,0]]).apply(meshes['tool'].vertices)+op
                points=fr.inv().apply(points-np.asarray(fp))
                lo,hi=points.min(0),points.max(0);counts=np.maximum(1,np.ceil((hi-lo)/.025).astype(int))
                width=(hi-lo)/counts;radius=float(np.linalg.norm(width)*.5+.001)
                for ijk in np.ndindex(*counts):
                    extras.append({'center':(lo+(np.asarray(ijk)+.5)*width).tolist(),'radius':radius})
            spheres['panda_hand']=extras
            kin['collision_spheres']=spheres
            return cfg,len(extras)
        report.update({'reference':str(args.reference),'schedule':str(args.schedule),'table_z':float(table_z),
                       'mount_xyzw':mount.tolist(),'scene_translation_xyz':translation.tolist(),
                       'bc_finger_source':'measured finger trajectory plus scaled original control residual','finger_command_blend':args.finger_command_blend,'start_at_bc_diagnostic':args.start_at_bc,'bc_wrist_source':'qpos_right_actual',
                       'left_hand_present':False,'relaxed_initial_and_terminal':'first_BC_wrist_elevated_20cm','physics':'PhysX CPU','xhand_force_drive':[5,.5,3]})
        capture('INITIAL',0,np.r_[qinit[arm],qinit[fingers]])
        if args.source_clock:
            from multipath.source_clock import run_source_clock
            if args.start_at_bc or args.grasp_only:
                raise ValueError('Source-clock mode requires a complete rollout')
            run_source_clock(args=args,report=report,ref=ref,schedule=schedule,robot=robot,arm=arm,
                fingers=fingers,wrist=wrist,objects=objects,translation=translation,mount=mount,ik=ik,
                tensor=tensor,planner_robot=planner_robot,table=table,meshes=meshes,step=step,records=records,out=out)
            if not report['full_pipeline_success']:raise RuntimeError('Source-clock semantic or dynamic validation failed')
            return
        windows=list(schedule.windows)
        for wi,w in enumerate(windows):
            if args.start_at_bc and wi==0:continue
            info={'controller':w.controller,'start':w.start,'end':w.end};report['windows'].append(info)
            print('WINDOW',info,flush=True)
            if w.controller=='BC' and wi==1 and repaired is not None:
                start_arm=np.asarray(robot.get_joint_positions())[arm]
                initial_fingers=repaired['initial_finger_q']
                # Reach the measured successful entry using commands, then
                # replay its physically verified control stream at original dt.
                fp,fq0=flange.get_world_pose()
                previous=tensor.to_device(start_arm)
                entry_path=[start_arm.copy()]
                for alpha in np.linspace(0,1,31)[1:]:
                    position=(1-alpha)*np.asarray(fp)+alpha*goals_xyz[w.start]
                    solution=ik.solve_single(Pose(position=tensor.to_device(position)[None],quaternion=tensor.to_device(goals_q[w.start])[None]),
                        retract_config=tensor.to_device(repaired['initial_arm_q'])[None],seed_config=previous[None,None],num_seeds=10)
                    if not bool(solution.success.item()):raise RuntimeError('Cartesian BC entry failed IK')
                    previous=solution.solution.detach().reshape(-1)
                    entry_path.append(previous.cpu().numpy().copy())
                entry_a,entry_f,timing=retime_bc(np.stack(entry_path),np.stack([initial_fingers]*len(entry_path)),fps=15.)
                for a,f in zip(entry_a,entry_f):step(a,f,'BC_ENTRY',w.start)
                for _ in range(120):step(entry_a[-1],initial_fingers,'BC_ENTRY_SETTLE',w.start)
                info['entry_object_displacement_m']=float(np.linalg.norm(np.asarray(objects['tool'].get_world_pose()[0])-initial_objects['tool'][:3]))
                commands=repaired['commands'].copy();n=len(commands)
                if args.grasp_closure:
                    weight=np.clip((repaired['source_frames']-46.)/8.,0,1)
                    for finger in (0,1,4,5,6,7,8,9,10,11):
                        commands[:,7+finger]+=weight*args.grasp_closure
                    info['grasp_closure_rad']=args.grasp_closure
                for i in range(n*4):
                    at=i/4;lo=min(int(at),n-1);hi=min(lo+1,n-1);blend=at-lo
                    command=commands[lo]*(1-blend)+commands[hi]*blend
                    step(command[:7],command[7:],str(repaired['modes'][lo]),float(repaired['source_frames'][lo]))
                measured=[]
                for _ in range(120):
                    step(commands[-1,:7],commands[-1,7:],'BC_HOLD',w.end-1)
                    measured.append(np.asarray(objects['tool'].get_world_pose()[0]))
                measured=np.asarray(measured)
                info['repaired_grasp_min_lift_m']=float(measured[:,2].min()-initial_objects['tool'][2])
                info['repaired_grasp_hold_drift_m']=float(np.max(np.linalg.norm(measured-measured[-1],axis=1)))
                if info['repaired_grasp_min_lift_m']<.04 or info['repaired_grasp_hold_drift_m']>.02:
                    raise RuntimeError('Repaired BC entry failed physical grasp retention')
                finger_targets[w.end-1]=commands[-1,7:]
                if args.grasp_only:
                    report['status']='grasp_lift_hold_passed';break
            elif w.controller=='BC':
                previous=tensor.to_device(np.asarray(robot.get_joint_positions())[arm])
                bc_ik_errors=[]
                for i in range(w.start,w.end):
                    sol=ik.solve_single(pose_at(i),
                        retract_config=previous[None],seed_config=previous[None,None],num_seeds=40)
                    if not bool(sol.success.item()):raise RuntimeError(f'BC handoff IK failed at {i}')
                    previous=sol.solution.detach().reshape(-1);arm_reference[i]=previous.cpu().numpy()
                    fk=ik.kinematics.get_state(previous[None]).ee_position.detach().cpu().numpy().reshape(-1)
                    err=float(np.linalg.norm(fk-goals_xyz[i]));bc_ik_errors.append(err)
                    if err>.005:raise RuntimeError(f'BC frame {i} IK residual {err} m')
                info['bc_ik_max_position_error_m']=max(bc_ik_errors)
                # The free-space approach stops above the contact anchor.
                # Execute the short descent with the BC controller while
                # keeping object contact active in PhysX.
                entry_q=np.asarray(robot.get_joint_positions())[arm]
                entry_f=np.asarray(robot.get_joint_positions())[fingers]
                if np.max(np.abs(entry_q-arm_reference[w.start]))>.02:
                    pa,pf,entry_timing=retime_bc(np.stack([entry_q,arm_reference[w.start]]),
                        np.stack([entry_f,finger_targets[w.start]]),fps=1.)
                    for a,f in zip(pa,pf):step(a,f,'BC_APPROACH',w.start)
                    info['contact_approach_duration_s']=entry_timing['duration_s']
                aq,fq,timing=retime_bc(arm_reference[w.start:w.end],finger_targets[w.start:w.end],schedule.fps)
                info.update(timing)
                errors=[]
                correction_q=np.zeros(7);correction_xy=np.zeros(2)
                correction_max=0.
                for j,(a,f) in enumerate(zip(aq,fq)):
                    frame=w.start+j/(len(aq)-1)*(w.end-w.start-1)
                    if args.object_relative_bc and wi==1:
                        if j%12==0:
                            op=np.asarray(objects['tool'].get_world_pose()[0])
                            expected=np.array([np.interp(frame,np.arange(len(source)),ref['object_pose_actual_tool_xyzw'][:,k]) for k in range(3)])+translation
                            wanted=np.clip(op[:2]-expected[:2],-.06,.06)
                            correction_xy+=np.clip(wanted-correction_xy,-.005,.005)
                            state=ik.kinematics.get_state(tensor.to_device(a)[None])
                            target=state.ee_position.detach().clone();target[0,:2]+=tensor.to_device(correction_xy)
                            corrected=ik.solve_single(Pose(position=target,quaternion=state.ee_quaternion.detach().clone()),
                                retract_config=tensor.to_device(a)[None],seed_config=tensor.to_device(a)[None,None],num_seeds=10)
                            if not bool(corrected.success.item()):raise RuntimeError('Object-relative BC correction IK failed')
                            correction_q=corrected.solution.detach().cpu().numpy().reshape(-1)-a
                            correction_max=max(correction_max,float(np.linalg.norm(correction_xy)))
                        a=a+correction_q
                    step(a,f,'BC',frame)
                    errors.append(float(np.max(np.abs(np.asarray(robot.get_joint_positions())[arm]-a))))
                    if j%240==0:print(f'BC source={frame:.2f} step={j}/{len(aq)} object_z={objects["tool"].get_world_pose()[0][2]:.3f}',flush=True)
                info['arm_tracking_max_rad']=max(errors)
                info['object_relative_correction_max_m']=correction_max
                if wi==1:
                    before=np.asarray(objects['tool'].get_world_pose()[0])
                    wp,wq=wrist.get_world_pose();fp,fq0=flange.get_world_pose()
                    lift=ik.solve_single(Pose(position=tensor.to_device(np.asarray(fp)+[0,0,.10])[None],
                        quaternion=tensor.to_device(fq0)[None]),retract_config=tensor.to_device(np.asarray(robot.get_joint_positions())[arm])[None],num_seeds=40)
                    if not bool(lift.success.item()):raise RuntimeError('BC departure lift IK failed')
                    qnow=np.asarray(robot.get_joint_positions())[arm];qnext=lift.solution.detach().cpu().numpy().reshape(-1)
                    la,lf,lt=retime_bc(np.stack([qnow,qnext]),np.stack([finger_targets[w.end-1]]*2),fps=.5)
                    for a,f in zip(la,lf):step(a,f,'BC_LIFT',w.end-1)
                    hold=[]
                    for _ in range(120):
                        step(la[-1],lf[-1],'BC_HOLD',w.end-1)
                        hold.append(np.asarray(objects['tool'].get_world_pose()[0]))
                    after=np.asarray(objects['tool'].get_world_pose()[0])
                    info['departure_hold_min_lift_m']=float(np.min(np.asarray(hold)[:,2])-before[2])
                    info['departure_hold_object_drift_m']=float(np.max(np.linalg.norm(np.asarray(hold)-after,axis=1)))
                    info['departure_object_lift_m']=float(after[2]-before[2])
                    info['departure_wrist_lift_m']=float(wrist.get_world_pose()[0][2]-wp[2])
                    if info['departure_hold_min_lift_m']<.04 or info['departure_hold_object_drift_m']>.02:
                        raise RuntimeError('Grasp BC failed load-bearing lift; cuRobo transport is not executable yet')
                    if args.grasp_only:
                        report['status']='grasp_lift_hold_passed'
                        break
                else:
                    # Finish contact release before returning to free-space
                    # collision avoidance. No object pose is assigned.
                    fp,fq0=flange.get_world_pose();current=np.asarray(robot.get_joint_positions())[arm]
                    lift=ik.solve_single(Pose(position=tensor.to_device(np.asarray(fp)+[0,0,.12])[None],
                        quaternion=tensor.to_device(fq0)[None]),retract_config=tensor.to_device(current)[None],num_seeds=40)
                    if not bool(lift.success.item()):raise RuntimeError('Release retreat IK failed')
                    qnext=lift.solution.detach().cpu().numpy().reshape(-1)
                    la,lf,lt=retime_bc(np.stack([current,qnext]),np.stack([finger_targets[w.end-1]]*2),fps=.5)
                    for a,f in zip(la,lf):step(a,f,'BC_RELEASE_RETREAT',w.end-1)
                    info['release_retreat_duration_s']=lt['duration_s']
            else:
                active_exclusions=next((s for s in exclusion_segments(exclusions) if s['window']==[w.start,w.end]),None) if exclusions else None
                endpoint=w.end if w.end<schedule.num_frames else w.end-1
                startq=np.asarray(robot.get_joint_positions())[arm]
                if wi==0:
                    # Shape the fingers in free space before computing their
                    # collision envelope for the approach.
                    entry=repaired['initial_finger_q'] if repaired is not None else finger_targets[endpoint]
                    for _ in range(120):step(startq,entry,'PREPARE',0)
                    startq=np.asarray(robot.get_joint_positions())[arm]
                carry=wi>0 and wi<len(schedule.windows)-1
                if carry:
                    # Retarget the reused placing chunk to the measured grasp
                    # and target. Preserve its wrist-relative path and fingers.
                    wp,wq=wrist.get_world_pose();op,oq=objects['tool'].get_world_pose();tp,tq=objects['target'].get_world_pose()
                    wr=Rotation.from_quat(np.asarray(wq)[[1,2,3,0]])
                    objr=Rotation.from_quat(np.asarray(oq)[[1,2,3,0]])
                    tr=Rotation.from_quat(np.asarray(tq)[[1,2,3,0]])
                    relative_position=wr.inv().apply(np.asarray(op)-wp)
                    relative_rotation=wr.inv()*objr
                    next_bc=windows[wi+1];last=next_bc.end-1
                    last_wr=Rotation.from_quat(source[last,[4,5,6,3]])
                    end_obj_r=last_wr*relative_rotation
                    target_points=tr.apply(meshes['target'].vertices)+tp
                    target_center=(target_points.min(0)+target_points.max(0))/2
                    object_center=target_center.copy()
                    object_center[2]=target_points[:,2].max()-end_obj_r.apply(meshes['tool'].vertices)[:,2].min()+.002
                    wrist_destination=object_center-last_wr.apply(relative_position)
                    delta=wrist_destination-goals_xyz[last]
                    if args.place_correction_json:
                        repair=json.loads(args.place_correction_json.read_text())
                        correction=np.asarray(repair['place_translation_correction_xyz'])
                        delta+=correction
                        info['physical_terminal_position_correction_xyz']=correction.tolist()
                    goals_xyz[next_bc.start:next_bc.end]+=delta
                    goal.position[next_bc.start:next_bc.end]=tensor.to_device(goals_xyz[next_bc.start:next_bc.end])
                    info['place_chunk_translation_xyz']=delta.tolist()
                    info['measured_wrist_to_object_xyz']=relative_position.tolist()
                cfg,n_spheres=planner_robot(carry)
                # Rebuild obstacles from measured poses at each handoff.
                live_world={'cuboid':{'table':table}}
                for role in (['target'] if carry else ['tool','target']):
                    op,oq=objects[role].get_world_pose()
                    r=Rotation.from_quat(np.asarray(oq)[[1,2,3,0]])
                    bounds=meshes[role].bounds;center=(bounds[0]+bounds[1])/2
                    live_world['cuboid'][role]={'dims':(bounds[1]-bounds[0]).tolist(),
                        'pose':np.r_[np.asarray(op)+r.apply(center),oq].tolist()}
                if active_exclusions:
                    live_world['cuboid'].update(active_exclusions['cuboid'])
                    info['virtual_path_obstacle_count']=len(active_exclusions['cuboid'])

                info['xhand_and_payload_sphere_count']=n_spheres
                planner=MotionGen(MotionGenConfig.load_from_robot_config(cfg,live_world,tensor_args=tensor,
                    interpolation_dt=1/120,self_collision_check=True,
                    collision_checker_type=CollisionCheckerType.PRIMITIVE,use_cuda_graph=False))
                def js(q):return JointState.from_position(tensor.to_device(q)[None],joint_names=planner.joint_names)
                state=planner.compute_kinematics(js(startq))
                spheres_np=state.robot_spheres.detach().cpu().numpy().reshape(-1,4)
                km=planner.kinematics.kinematics_config
                idxmap=km.link_sphere_idx_map.detach().cpu().numpy().reshape(-1)
                namemap={v:k for k,v in km.link_name_to_idx_map.items()}
                collisions=[];ignored=cfg['kinematics']['self_collision_ignore']
                for si in range(len(spheres_np)):
                    for sj in range(si+1,len(spheres_np)):
                        a,b=namemap[int(idxmap[si])],namemap[int(idxmap[sj])]
                        if a==b or b in ignored.get(a,[]) or a in ignored.get(b,[]):continue
                        if min(spheres_np[si,3],spheres_np[sj,3])<=0:continue
                        overlap=float(spheres_np[si,3]+spheres_np[sj,3]-np.linalg.norm(spheres_np[si,:3]-spheres_np[sj,:3]))
                        if overlap>0:collisions.append([a,b,overlap])
                info['start_self_collision_pairs']=sorted(collisions,key=lambda x:-x[2])[:10]
                goal_ok,goal_status=planner.check_start_state(js(arm_reference[endpoint]))
                info['nominal_bc_joint_goal_valid']=bool(goal_ok);info['nominal_bc_joint_goal_status']=str(goal_status)
                np.savez_compressed(out/f'planner_start_spheres_{wi}.npz',spheres=spheres_np)
                (out/f'planner_world_{wi}.json').write_text(json.dumps(live_world,indent=2))
                goal_spheres=planner.compute_kinematics(js(arm_reference[endpoint])).robot_spheres.detach().cpu().numpy().reshape(-1,4)
                world_hits=[]
                for name,box in live_world['cuboid'].items():
                    pose=np.asarray(box['pose']);rot=Rotation.from_quat(pose[[4,5,6,3]])
                    local=rot.inv().apply(goal_spheres[:,:3]-pose[:3])
                    dist=np.linalg.norm(np.maximum(np.abs(local)-np.asarray(box['dims'])/2,0),axis=1)
                    for si in np.flatnonzero((dist<goal_spheres[:,3]) & (goal_spheres[:,3]>0)):
                        world_hits.append([name,namemap[int(idxmap[si])],float(goal_spheres[si,3]-dist[si])])
                info['nominal_bc_goal_world_collisions']=sorted(world_hits,key=lambda x:-x[2])[:10]
                if w.end<schedule.num_frames:
                    result=None
                    info['precontact_candidates']=[]
                    offsets=([[-.10,0,0],[-.08,-.04,0],[-.06,-.08,0],[0,-.10,0]] if wi==0 and repaired is not None else [])+[[0,0,h] for h in (.06,.08,.10,.12,.15,.18)]
                    for candidate_index,offset in enumerate(offsets):
                        height=offset[2]
                        raised=pose_at(endpoint,offset)
                        candidate=ik.solve_single(raised,retract_config=tensor.to_device(startq)[None],num_seeds=40)
                        if not bool(candidate.success.item()):
                            info['precontact_candidates'].append([height,'ik_failed']);continue
                        cq=candidate.solution.detach().cpu().numpy().reshape(-1)
                        fk=ik.kinematics.get_state(tensor.to_device(cq)[None])
                        actual_goal=fk.ee_position.detach().cpu().numpy().reshape(-1)
                        goal_error=float(np.linalg.norm(actual_goal-goals_xyz[endpoint]-offset))
                        info.setdefault('ik_goal_diagnostics',[]).append({'height':height,'requested_xyz':(goals_xyz[endpoint]+offset).tolist(),
                            'returned_fk_xyz':actual_goal.tolist(),'error_m':goal_error,'reported_error_m':float(candidate.position_error.item())})
                        if goal_error>.005:raise RuntimeError(f'IK reported success with {goal_error} m goal error')
                        ok,status=planner.check_start_state(js(cq))
                        np.savez_compressed(out/f'candidate_spheres_{wi}_{candidate_index}.npz',spheres=planner.compute_kinematics(js(cq)).robot_spheres.detach().cpu().numpy(),arm_q=cq)
                        info['precontact_candidates'].append([height,str(status)])
                        if not ok:continue
                        trial=planner.plan_single_js(js(startq),js(cq),MotionGenPlanConfig(max_attempts=10,enable_graph_attempt=3))
                        result=trial
                        if bool(trial.success.item()):
                            info['precontact_offset_xyz']=offset;break
                    if result is None:raise RuntimeError(f"No collision-free precontact anchor: {info['precontact_candidates']}")
                else:
                    result=planner.plan_single_js(js(startq),js(arm_reference[endpoint]),MotionGenPlanConfig(max_attempts=10,enable_graph_attempt=3))
                if not bool(result.success.item()) and wi==0:
                    # Extend the interaction window to a collision-free entry.
                    # This keeps contact approach under BC instead of asking a
                    # collision-avoiding free-space planner to enter contact.
                    candidates=list(range(endpoint-1,max(5,endpoint-20)-1,-1))
                    solutions=planner.ik_solver.solve_batch(Pose(position=goal.position[candidates],quaternion=goal.quaternion[candidates]))
                    feasible=solutions.success.detach().cpu().numpy().reshape(-1)
                    info['entry_candidates']={str(f):bool(ok) for f,ok in zip(candidates,feasible)}
                    for candidate,ok in zip(candidates,feasible):
                        if not ok:continue
                        trial=planner.plan_single(js(startq),Pose(position=goal.position[candidate:candidate+1],
                            quaternion=goal.quaternion[candidate:candidate+1]),MotionGenPlanConfig(max_attempts=10,enable_graph_attempt=3))
                        if bool(trial.success.item()):
                            result=trial
                            info['original_end']=w.end;info['end']=candidate
                            endpoint=candidate
                            w=Window(w.start,candidate,w.controller)
                            windows[wi+1]=Window(candidate,windows[wi+1].end,'BC')
                            break
                info['plan_status']=str(result.status)
                if not bool(result.success.item()):raise RuntimeError(f'cuRobo bridge {w.start}:{w.end} failed: {result.status}')
                path=result.get_interpolated_plan().position.detach().cpu().numpy().reshape(-1,7)
                count=len(path)
                path=np.stack([np.interp(np.linspace(0,count-1,count*3),np.arange(count),path[:,i]) for i in range(7)],axis=1)
                info['execution_time_scale']=3.
                info['duration_s']=len(path)/120
                fstart=np.asarray(robot.get_joint_positions())[fingers]
                fend=repaired['initial_finger_q'] if wi==0 and repaired is not None else finger_targets[endpoint]
                # Keep the acquired finger command during transport; reach the
                # next BC's entry command smoothly in its final fifth.
                if carry:fstart=finger_targets[w.start-1]
                retention=[]
                for j,a in enumerate(path):
                    alpha=j/max(1,len(path)-1)
                    blend=0. if carry else alpha
                    step(a,fstart+(fend-fstart)*blend,'CUROBO',w.start+alpha*(w.end-w.start))
                    if carry:
                        wp,wq=wrist.get_world_pose();op,_=objects['tool'].get_world_pose()
                        offset=Rotation.from_quat(np.asarray(wq)[[1,2,3,0]]).inv().apply(np.asarray(op)-wp)
                        retention.append(float(np.linalg.norm(offset-relative_position)))
                for _ in range(24):step(path[-1],fstart if carry else fend,'CUROBO',endpoint)
                if carry:
                    info['transport_max_object_relative_drift_m']=max(retention)
                    if max(retention)>.04:raise RuntimeError('Object slipped during cuRobo transport')
                info['final_arm_error_rad']=float(np.max(np.abs(np.asarray(robot.get_joint_positions())[arm]-path[-1])))
                np.savez_compressed(out/f'planner_end_spheres_{wi}.npz',
                    spheres=planner.compute_kinematics(js(np.asarray(robot.get_joint_positions())[arm])).robot_spheres.detach().cpu().numpy())
                if active_exclusions:
                    metrics=compare_path(transport_points(records,w.start,w.end),active_exclusions)
                    info['path_diversity']=metrics
                    if not metrics['actual_wrist_exclusion_pass']:
                        raise RuntimeError('Measured wrist entered a prior-path exclusion region')
            (out/'pipeline_result.json').write_text(json.dumps(report,indent=2))
        # Preserve the applied finger target and hence contact force while
        # settling. Replacing it by measured positions unloads a held grasp.
        settle_command=last_command.copy()
        for _ in range(120):step(settle_command[:7],settle_command[7:],'SETTLE',schedule.num_frames-1)
        op,_=objects['tool'].get_world_pose();tp,_=objects['target'].get_world_pose()
        object_trace=np.asarray([x['tool_xyz_wxyz'][:3] for x in records])
        target_trace=np.asarray([x['target_xyz_wxyz'][:3] for x in records])
        objp,objq=objects['tool'].get_world_pose();tarp,tarq=objects['target'].get_world_pose()
        objrot=Rotation.from_quat(np.asarray(objq)[[1,2,3,0]])
        tarrot=Rotation.from_quat(np.asarray(tarq)[[1,2,3,0]])
        tool_in_target=tarrot.inv().apply(objrot.apply(meshes['tool'].vertices)+objp-tarp)
        bounds=meshes['target'].bounds
        center=(bounds[0,:2]+bounds[1,:2])/2;half=(bounds[1,:2]-bounds[0,:2])/2
        footprint_score=float(np.max(np.linalg.norm((tool_in_target[:,:2]-center)/half,axis=1)))
        support_gap=float(tool_in_target[:,2].min()-bounds[1,2])
        stable_drift=float(np.max(np.linalg.norm(object_trace[-30:]-object_trace[-1],axis=1)))
        task_success=bool(footprint_score<1.0 and -.02<support_gap<.025 and stable_drift<.005)
        report['physical_outcome']={'max_object_lift_m':float(object_trace[:,2].max()-initial_objects['tool'][2]),
            'final_target_footprint_score':footprint_score,'final_support_gap_m':support_gap,
            'final_one_second_object_drift_m':stable_drift,'placement_geometry_and_stability_pass':task_success}
        final_distance=float(np.linalg.norm(np.asarray(op)[:2]-np.asarray(tp)[:2]))
        report.update({'status':'grasp_lift_hold_passed' if args.grasp_only else 'continuous_execution_complete','frames_recorded':len(records),'duration_s':sim_steps/120,
                       'final_object_target_xy_distance_m':final_distance,
                       'task_success':task_success,'task_success_note':'Tool mesh inside target footprint, near target support height, and stable for final second; inspect actual video as well.'})
        report['full_pipeline_success']=bool(task_success and not args.start_at_bc and not args.grasp_only
                                            and len(report['windows'])==len(schedule.windows))
        if exclusions:
            checks=[x['path_diversity']['actual_wrist_exclusion_pass'] for x in report['windows'] if 'path_diversity' in x]
            report['diverse_solution_success']=bool(report['full_pipeline_success'] and len(checks)==len(exclusion_segments(exclusions)) and all(checks))
    except Exception as exc:
        report['status']='failed'
        report['error']=f'{type(exc).__name__}: {exc}'
        import traceback;traceback.print_exc()
    finally:
        if recorder:recorder.save()
        (out/'actual_trace.json').write_text(json.dumps(records))
        (out/'pipeline_result.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2),flush=True)
        app.close()
    if report['status']=='failed':raise SystemExit(1)

if __name__=='__main__':main()
