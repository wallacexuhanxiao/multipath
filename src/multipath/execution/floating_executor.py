"""Execute validated floating Anchors and cuRobo bridges in one PhysX world."""
import json
from pathlib import Path
import numpy as np
from multipath.anchors.artifact import load_validated_anchor
from multipath.anchors.floating import validate_commands
from multipath.anchors.mpc import WindowEvaluation
from multipath.planning.floating import FloatingPlanner,pose_coordinates
from multipath.planning.timing import physics_ticks_per_frame
from multipath.validation.physics import check_retention
from multipath.validation.task import evaluate_task


def execute(scene,schedule,manifest,proposal_dir,report):
    tpf=physics_ticks_per_frame(schedule.fps,scene.dt)
    before=getattr(scene.args,'stop_before_anchor',None)
    through=getattr(scene.args,'through_anchor',None)
    required=[a for a in schedule.anchors.values()
              if (not before or a.core_start<schedule.anchors[before].core_start)
              and (not through or a.core_start<=schedule.anchors[through].core_start)]
    canonical={a.id:load_validated_anchor(scene.args.validated_anchors,a,provenance=manifest['provenance'],
                                         fps=schedule.fps,dt=scene.dt) for a in required}
    if any(v['entry'].get('embodiment')!=scene.embodiment for v in canonical.values()):
        raise ValueError('Canonical Anchor embodiment mismatch')
    initial=scene.state();held=None
    report.update(source='oracle',embodiment=scene.embodiment,source_fps=schedule.fps,
                  source_duration_s=schedule.num_frames/schedule.fps,vlm_used=False,
                  complete_robot_reference_required=False,relaxed_reference_tracking=False,
                  state_resets_between_segments=0,segments=[],anchors=[],task_success=False,full_pipeline_success=False)
    for wi,window in enumerate(schedule.windows):
        if before and window.get('anchor')==before:
            report.update(status='predecessor_prefix_complete',prefix_success=True)
            return
        info=dict(window,budget_s=(window['end']-window['start'])/schedule.fps,success=False)
        report['segments'].append(info);start=scene.state()
        if not np.isclose(start['time_s'],window['start']/schedule.fps,rtol=0,atol=1e-8):
            raise RuntimeError('Continuous source clock mismatch at segment entry')
        print('ORACLE_FLOATING_SEGMENT',info,flush=True)
        if window['mode']=='RELAXED':
            carry=window['contact']=='grasped'
            if start['contact']['mode']!=window['contact']:raise RuntimeError('RELAXED cannot change contact mode')
            if carry:
                if held is None:raise RuntimeError('No physically acquired grasp transform')
                check_retention(start,held)
            later=next((w for w in schedule.windows[wi+1:] if w['mode']=='ANCHOR'),None)
            # A prefix before an unvalidated Anchor stops at the preceding exit;
            # it cannot invent the next canonical entry from a human reference.
            prefix=bool(later and later['anchor'] not in canonical)
            terminal_frame=window['end']
            if prefix:
                if later['anchor']!=before:raise RuntimeError('Missing validated canonical Anchor')
                anchor=schedule.anchors[later['anchor']];terminal_frame=anchor.synth_start
                if terminal_frame==window['start']:
                    report['segments'].pop();report.update(status='predecessor_prefix_complete',prefix_success=True);return
                data=dict(np.load(Path(proposal_dir)/manifest['anchors'][anchor.id],allow_pickle=False))
                goal_pose=data['wrist_hand_q'][0,:7].copy()
                if carry:
                    from scipy.spatial.transform import Rotation
                    matrix=data['tool_pose'][0]@np.linalg.inv(held)
                    goal_pose=np.r_[matrix[:3,3],Rotation.from_matrix(matrix[:3,:3]).as_quat()[[3,0,1,2]]]
                goal=dict(wrist_xyz_wxyz=goal_pose,wrist_twist=np.zeros(6),hand_q=start['hand_q'])
                info.update(end=terminal_frame,budget_s=(terminal_frame-window['start'])/schedule.fps,
                            development_prefix=True,canonical_goal=False)
            else:goal=canonical[later['anchor']]['entry'] if later else initial
            if later and not prefix:
                from multipath.planning.bridge import bridge_precheck
                check=bridge_precheck(start,goal,(terminal_frame-window['start'])/schedule.fps,scene.cfg['planning'])
                info['boundary_precheck']=check
                if not check['feasible']:
                    raise RuntimeError(f'Canonical bridge boundaries require local resynthesis: {check["reasons"]}')
            planner=FloatingPlanner(scene,carry,scene.out/f'planner_{window["start"]}')
            if prefix and not carry:
                reference=np.asarray(goal['wrist_xyz_wxyz']).copy()
                for height in np.linspace(0,float(np.linalg.norm(scene.meshes['tool'].extents)),9):
                    target=reference.copy();target[2]+=height
                    if planner.valid(pose_coordinates(target)):
                        goal['wrist_xyz_wxyz']=target;break
                else:raise RuntimeError('No collision-free development entry')
            commands,timing=planner.plan(np.asarray(goal['wrist_xyz_wxyz']),
                                         (terminal_frame-window['start'])*tpf,np.asarray(goal['wrist_twist']),
                                         goal_state=goal if later and not prefix else None)
            info.update(timing);info['collision_model']={'current_hand':True,'carried_object':carry,'assembly_self_collision':False}
            np.savez_compressed(scene.out/f'relaxed_{window["start"]}_{window["end"]}.npz',commands=commands)
            for j,command in enumerate(commands):
                scene.step_control(command,'RELAXED',window['start']+(j+1)/tpf)
                if timing['hand_boundary_motion'] and not planner.measured_valid():
                    raise RuntimeError('Measured RELAXED hand boundary collision')
                if (j+1)%tpf==0:
                    if carry:check_retention(scene.state(),held)
                    if not planner.measured_valid():raise RuntimeError('Measured RELAXED collision')
            transition=scene.check_transition(np.asarray(goal['wrist_xyz_wxyz']),window['contact'],canonical=goal if later and not prefix else None)
        else:
            anchor=schedule.anchors[window['anchor']];artifact=canonical[anchor.id]
            transition=scene.check_transition(np.asarray(artifact['entry']['wrist_xyz_wxyz']),anchor.entry_contact,canonical=artifact['entry'])
            info['entry_transition']=transition
            if not transition['success']:raise RuntimeError('Physically reached canonical Anchor entry mismatch')
            data=dict(np.load(Path(proposal_dir)/manifest['anchors'][anchor.id],allow_pickle=False))
            commands=artifact['commands'];check=validate_commands(scene,commands,start)
            if not check['feasible']:raise RuntimeError(f'Canonical commands infeasible from actual entry: {check}')
            objective=WindowEvaluation(scene,anchor,data,start)
            for j,command in enumerate(commands):
                frame=anchor.core_start+(j+1)/tpf;scene.step_control(command,'ANCHOR',frame)
                if (j+1)%tpf==0 and not objective.observe(frame,command):
                    info['failure']=objective.failure;raise RuntimeError(f'Continuous Anchor failed: {objective.failure}')
            transition=scene.check_transition(np.asarray(artifact['exit']['wrist_xyz_wxyz']),anchor.exit_contact,canonical=artifact['exit'])
            held=np.asarray(scene.state()['hand_object_transform']) if anchor.exit_contact=='grasped' else None
            report['anchors'].append(dict(artifact['report'],handoff_verified=transition['success'],
                                          continuous_execution_success=transition['success']))
        info['transition']=transition
        if not transition['success']:raise RuntimeError('Actual segment handoff failed')
        if scene.restore_count:raise RuntimeError('State reset during continuous execution')
        if not np.isclose(scene.ticks*scene.dt,info['end']/schedule.fps,rtol=0,atol=1e-8):
            raise RuntimeError('Source time budget changed')
        info.update(success=True,actual_end_s=scene.ticks*scene.dt)
        (scene.out/'progress.json').write_text(json.dumps(report,indent=2))
        if window['mode']=='RELAXED' and prefix:
            report.update(status='predecessor_prefix_complete',prefix_success=True);return
        if through and window.get('anchor')==through:
            report.update(status='prefix_complete',prefix_success=True);return
    task=evaluate_task(scene.state(),manifest,schedule.metadata['task_contract'],report['anchors'])
    success=bool(task['success'] and len(report['anchors'])==len(schedule.anchors) and scene.restore_count==0)
    report.update(task=task,task_success=task['success'],full_pipeline_success=success,
                  actual_duration_s=scene.ticks*scene.dt,status='complete' if success else 'task_failed')
