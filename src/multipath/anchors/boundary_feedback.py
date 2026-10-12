"""Resynthesize a grasp exit from a rejected fixed-time collision fit.

Only commands change. The preceding measured grasp and the collision field
size the proposal; unchanged source/core/physics gates must validate it.
"""
import json
from pathlib import Path
import numpy as np


def same_source_seed(previous,current):
    """Metadata-only oracle edits may reuse commands, never old acceptance.

    Callers separately check the relevant intervals and actual scene/state,
    then perform fresh physics under the current oracle.
    """
    keys=('triplet','sequence','input','transform_world_to_robot','hand_assignments')
    return all(previous.get(key)==current.get(key) for key in keys)


def single_grasp_exit_proposal(scene,anchor,commands,feedback_run,data):
    """Propose a reached grasp tail compatible with the next source-time bridge.

    Fresh padded/core physics must validate these drives. The measured states
    condition the proposal; they are never assigned during final execution.
    """
    from multipath.oracle_schedule import OracleSchedule
    from multipath.planning.bridge import lookahead_grasp_boundary,coordinate_velocity
    from multipath.planning.floating import pose_coordinates,coordinates_pose
    from multipath.planning.timing import boundary_fitted_path,physics_ticks_per_frame
    from multipath.anchors.floating import controls_from_poses
    folder=Path(feedback_run);report=json.loads((folder/'pipeline_result.json').read_text())
    if report['source_provenance']!=scene.manifest['provenance']:
        raise ValueError('Grasp exit feedback belongs to a different source')
    if report.get('state_resets_between_segments')!=0 or not any(
            s.get('anchor')==anchor.id and s.get('success') for s in report['segments']):
        raise ValueError('Exit shaping requires a continuously executed valid core')
    core=[s for s in json.loads((folder/'actual_trace.json').read_text())
          if anchor.core_start<=s['source_frame']<=anchor.core_end]
    acquired=next((s for i,s in enumerate(core)
        if all(v['contact']['mode']=='grasped' for v in core[i:])),None)
    if acquired is None or acquired['source_frame']>=anchor.core_end:
        raise RuntimeError('No physically acquired grasp tail for outgoing compatibility')
    schedule=OracleSchedule.load(scene.args.schedule)
    bridge=next(w for w in schedule.windows if w['start']==anchor.core_end)
    following=next(w for w in schedule.windows if w['start']==bridge['end'])
    if bridge['mode']!='RELAXED' or bridge.get('contact')!='grasped' or following['mode']!='ANCHOR':
        raise ValueError('Exit shaping requires a grasped bridge to the next Anchor')
    next_anchor=schedule.anchors[following['anchor']]
    next_data=np.load(scene.args.proposals/scene.manifest['anchors'][next_anchor.id],allow_pickle=False)
    source_entry=next_data['tool_pose'][next_anchor.core_start-next_anchor.synth_start]
    source_exit=data['tool_pose'][min(anchor.core_end-anchor.synth_start,len(data['tool_pose'])-1)]
    tpf=physics_ticks_per_frame(schedule.fps,scene.dt)
    cut=round((acquired['source_frame']-anchor.core_start)*tpf);ticks=len(commands)-cut
    from multipath.execution.mounted_scene import pose_matrix
    from multipath.planning.bridge import carried_object_clearance
    from scipy.spatial.transform import Rotation
    previous=core[-1];held=np.asarray(previous['hand_object_transform'])
    tracking=pose_matrix(previous['wrist_xyz_wxyz'])@np.linalg.inv(pose_matrix(commands[-1,:7]))
    def exit_clearance(q):
        predicted=tracking@pose_matrix(coordinates_pose(q))@held
        pose=np.r_[predicted[:3,3],Rotation.from_matrix(predicted[:3,:3]).as_quat()[[3,0,1,2]]]
        proposal=dict(previous,physically_acquired_state=False,
            objects=dict(previous['objects'],tool=dict(previous['objects']['tool'],pose=pose.tolist())))
        return min(carried_object_clearance(scene,proposal).values())-.002
    pose,velocity,condition=lookahead_grasp_boundary(acquired,core[-1],source_exit,source_entry,
        ticks*scene.dt,(bridge['end']-bridge['start'])/schedule.fps,scene.dt,scene.cfg['planning'],
        exit_condition=exit_clearance,boundary_hand_object_transform=held)
    first=pose_coordinates(acquired['wrist_xyz_wxyz']);raw=[first.copy()];near=first
    for c in commands[cut:]:
        near=pose_coordinates(c[:7],near);raw.append(near.copy())
    raw=np.asarray(raw);raw[-1]=pose_coordinates(pose,raw[-1]);limits=scene.cfg['planning']
    path,fit=boundary_fitted_path(raw,ticks,scene.dt,np.asarray(limits['velocity_limits']),
        np.asarray(limits['acceleration_limits']),coordinate_velocity(acquired['wrist_xyz_wxyz'],
        acquired['wrist_twist'],first),velocity)
    proposed=np.asarray(commands).copy()
    hands=np.vstack((acquired['command_hand_q'],commands[cut:,7:19]))
    proposed[cut:]=controls_from_poses(np.asarray([coordinates_pose(q) for q in path]),hands,scene.dt)
    return proposed,dict(method='physically_acquired_grasp_joint_exit_entry_fit',
        feedback_run=str(folder),physical_grasp_frame=acquired['source_frame'],
        boundary_proposal=condition,tail_fit=fit,source_time_stretch=False,
        original_finger_commands_retained=True,physical_acceptance=False)


def validated_joint_core_seed(scene,anchor,seed,run,*,side='right'):
    """Reuse passed local commands as a proposal, with fresh joint acceptance."""
    folder=Path(run);report=json.loads((folder/'bimanual_result.json').read_text())
    if (not report.get('bimanual_anchor_success')
            or not same_source_seed(report['source_provenance'],scene.manifest['provenance'])
            or report['core_interval']!=[anchor.core_start,anchor.core_end]
            or report['core_exit']['scene_id']!=scene.scene_id):
        raise ValueError('Compiled joint core does not match this physically validated source/scene/interval')
    command=np.load(folder/'joint_commands.npz',allow_pickle=False)[side].copy()
    if command.shape!=seed.shape:raise ValueError('Compiled joint core has a different source clock')
    return command,dict(method='physically_passed_joint_core_command_seed',run=str(folder),
        acceptance_reused=False,fresh_joint_physics_required=True)


def recorded_core_commands(run,anchor,tpf,side='right'):
    """Read actual commands as an optimizer/controller reference, not a proof."""
    folder=Path(run);report=json.loads((folder/'bimanual_result.json').read_text())
    if report.get('bimanual_anchor_success'):
        return np.load(folder/'joint_commands.npz',allow_pickle=False)[side].copy()
    commands=np.load(folder/'padded'/'executed_controls.npz',allow_pickle=False)[side]
    return commands[anchor.core_start*tpf:anchor.core_end*tpf].copy()


def source_conditioned_translation(delta,normal,actual,source):
    """Keep a native clearance correction inside the original C.4 boundary.

    Preserve its required outward displacement and move tangentially toward
    the source only as needed. The .98 interior is the existing canonical
    boundary fitting margin, not a different physical acceptance threshold.
    """
    from scipy.spatial.transform import Rotation
    from multipath.validation.physics import OBJECT_POSITION_TOLERANCE_M,OBJECT_ROTATION_TOLERANCE_RAD
    normal=np.asarray(normal,dtype=float);normal/=np.linalg.norm(normal)
    delta=np.asarray(delta,dtype=float);error=actual[:3,3]-source[:3,3]
    rotation=float(Rotation.from_matrix(source[:3,:3].T@actual[:3,:3]).magnitude())
    available=.98**2-(rotation/OBJECT_ROTATION_TOLERANCE_RAD)**2
    if available<=0:raise RuntimeError('Translation cannot repair the source orientation boundary')
    radius=OBJECT_POSITION_TOLERANCE_M*np.sqrt(available)
    proposed=error+delta;parallel=float(proposed@normal)
    if abs(parallel)>radius:
        raise RuntimeError('Native clearance correction has no intersection with the source boundary')
    tangent=proposed-parallel*normal
    bound=np.sqrt(max(0.,radius**2-parallel**2));length=np.linalg.norm(tangent)
    if length>bound:tangent*=bound/length
    result=parallel*normal+tangent-error
    score=np.sqrt(np.sum((error+result)**2)/OBJECT_POSITION_TOLERANCE_M**2+
        (rotation/OBJECT_ROTATION_TOLERANCE_RAD)**2)
    return result,dict(source_condition_unchanged=True,source_conditioned_translation=True,
        required_outward_displacement_m=float(delta@normal),
        predicted_object_boundary_score=float(score),
        tangential_correction_m=(result-delta).tolist(),physical_acceptance=False)


def collision_exit_proposal(scene,anchor,seed,feedback_run,*,side='right',source_boundary=None):
    folder=Path(feedback_run)
    report=json.loads((folder/'bimanual_result.json').read_text())
    if not same_source_seed(report['source_provenance'],scene.manifest['provenance']):
        raise ValueError('Exit feedback belongs to a different source')
    if report['core_interval']!=[anchor.core_start,anchor.core_end]:
        raise ValueError('Exit feedback belongs to a different Anchor')
    trace_name='exported_core' if report.get('bimanual_anchor_success') else 'padded'
    trace=json.loads((folder/trace_name/'actual_trace.json').read_text())
    core=[s for s in trace if anchor.core_start<=s['source_frame']<=anchor.core_end]
    if not core or core[-1]['source_frame']!=anchor.core_end:
        raise ValueError('Exit feedback did not physically execute the complete core')
    entry=core[-1]
    if entry['scene_id']!=scene.scene_id or not entry.get('physically_acquired_state'):
        raise ValueError('Exit feedback belongs to a different physical scene')
    failure=report.get('padded_rollout',{}).get('failure') or {}
    source_repair=bool(not report.get('bimanual_anchor_success') and source_boundary is not None
        and failure.get('hand')==side and failure.get('reason') in ('core_exit_handoff','core_object_fidelity')
        and failure.get('source_frame')==anchor.core_end
        and entry['hands'][side]['contact']['mode']=='grasped'
        and report.get('right_exit_resynthesis',{}).get('source_conditioned_translation'))
    if not report.get('bimanual_anchor_success') and not source_repair:
        raise ValueError('Rejected local core is not eligible for physical boundary repair')
    continuous_path=folder/'continuous_result.json'
    continuous=json.loads(continuous_path.read_text()) if continuous_path.exists() else {}
    native=continuous.get('native_contact_failure')
    coordinated=continuous.get('coordinated_collision_failure')
    posthoc=folder/'predicted_start_collision_feedback.json'
    if coordinated is None and posthoc.exists():
        measured=json.loads(posthoc.read_text())
        if not same_source_seed(measured['source_provenance'],report['source_provenance']):
            raise ValueError('Saved geometry feedback belongs to a different source')
        coordinated=measured['collision']
    segments=continuous.get('segments',[])
    last=segments[-1] if segments else {}
    transition=last.get('hands',{}).get(side,{}).get('transition',{})
    if (report.get('bimanual_anchor_success') and continuous.get('boundary_fit_failure')
            and last.get('mode')=='RELAXED' and last.get('start')==anchor.core_end):
        if source_boundary is None:raise ValueError('Quiet exit requires its unchanged oracle object boundary')
        return quiet_grasp_exit_proposal(scene,anchor,seed,folder,core,side,source_boundary,lookahead=True)
    center_for_bridge=bool(report.get('bimanual_anchor_success') and source_boundary is not None and not native
        and last.get('mode')=='RELAXED' and last.get('start')==anchor.core_end
        and transition.get('success') is False and transition.get('object_boundary_error')
        and transition.get('actual_contact',{}).get('mode')=='grasped')
    if center_for_bridge:
        from multipath.execution.mounted_scene import pose_matrix
        from multipath.validation.physics import object_tracking_score,core_pose_errors
        boundary_state=entry if side=='right' else entry['hands'][side]
        if not object_tracking_score(core_pose_errors(boundary_state,np.asarray(source_boundary)))['feasible']:
            raise ValueError('Canonical recentering requires a physically valid previous core')
        world=scene.scene if hasattr(scene,'scene') else scene
        role='tool' if side=='right' else world.supporting_hands[side].object_role
        actual=pose_matrix(entry['objects'][role]['pose'])
        delta=np.asarray(source_boundary)[:3,3]-actual[:3,3]
        feedback=dict(method='measured_source_centered_exit_for_failed_next_boundary',
            source_condition_unchanged=True,previous_core_physically_valid=True,
            next_entry_error=transition['object_boundary_error'],physical_acceptance=False,
            correction_uses_actual_object_pose=True)
    elif source_repair:
        # This rejected program is an optimizer proposal, never a validated
        # Anchor. Preserve its collision-normal displacement and correct the
        # measured boundary tangentially; fresh joint Replay decides success.
        normal=np.asarray(report['right_exit_resynthesis']['normal_world'])
        delta=np.zeros(3)
        feedback=dict(method='measured_local_boundary_translation_refinement',
            normal_world=normal.tolist(),rejected_core_used_as_proposal=True,
            physical_acceptance_reused=False)
    elif coordinated and coordinated['physics_tick']<=1:
        # A collision at the fixed first slope cannot be repaired by moving
        # later path points. Refine the acquired predecessor's command exit,
        # then require fresh padded/core physics and a newly planned bridge.
        rows=coordinated['primary_collisions' if side=='right' else 'support_collisions']
        contacts=[c for row in rows if row['mesh']=='tool' for c in row.get('object_separation_contacts',[])]
        if not contacts:raise RuntimeError('Fixed-start collision has no solid-body separation feedback')
        geometry=json.loads((folder/f'planner_{anchor.core_end}_{side}'/'object_collision_model.json').read_text())
        band=sum(float(np.max(g['native_contact_offsets_m'])) for g in geometry.values())
        normals=np.asarray([c['normal_outward_world'] for c in contacts])
        required=np.asarray([c['penetration_depth_m'] for c in contacts])+band+1e-4
        from scipy.optimize import minimize
        result=minimize(lambda x:(.5*x@x,x),np.zeros(3),jac=True,method='SLSQP',
            constraints=[dict(type='ineq',fun=lambda x:normals@x-required,jac=lambda x:normals)],
            options={'ftol':1e-12,'maxiter':30})
        if np.min(normals@result.x-required)<-1e-8:raise RuntimeError('No translation resolves the fixed-start object collision')
        delta=result.x;normal=delta/np.linalg.norm(delta)
        feedback=dict(method='physically_acquired_exit_plus_predicted_fixed_start_collision',
            predicted_contacts=contacts,native_contact_band_m=band,normal_world=normal.tolist(),
            collision_physically_executed=False,fresh_bridge_validation_required=True)
    elif native:
        from multipath.execution.mounted_scene import pose_matrix
        failed=json.loads((folder/'failed_state.json').read_text())
        force=np.asarray(native['forces_n']);loaded=force>.01
        if not loaded.any():raise ValueError('Native boundary feedback has no loaded contact')
        normal=np.average(np.asarray(native['normals_world'])[loaded],axis=0,weights=force[loaded])
        normal/=np.linalg.norm(normal)
        if side!='right':normal=-normal
        world=scene.scene if hasattr(scene,'scene') else scene
        role='tool' if side=='right' else world.supporting_hands[side].object_role
        peer=next(r for r in failed['objects'] if r!=role)
        held=np.asarray(entry['hands'][side]['hand_object_transform'])
        predicted=pose_matrix(failed['hands'][side]['command_wrist'])@held
        actual=pose_matrix(failed['objects'][role]['pose'])
        def vertices(r,m):return world.meshes[r].vertices@m[:3,:3].T+m[:3,3]
        own_error=vertices(role,actual)-vertices(role,predicted)
        peer_error=vertices(peer,pose_matrix(failed['objects'][peer]['pose']))-vertices(peer,pose_matrix(entry['objects'][peer]['pose']))
        closing_error=max(0.,float(np.max(-own_error@normal)))+max(0.,float(np.max(peer_error@normal)))
        geometry=json.loads((folder/f'planner_{anchor.core_end}_{side}'/'object_collision_model.json').read_text())
        band=sum(float(np.max(g['native_contact_offsets_m'])) for g in geometry.values())
        separation=float(np.min(np.asarray(native['native_separations_m'])[loaded]))
        # Counter the observed contact deficit and closed-loop closing error.
        # Preserve the existing collision fitter's 0.1 mm interior requirement.
        delta=normal*(max(0.,band-separation)+closing_error+1e-4)
        feedback=dict(method='physical_grasp_plus_native_contact_exit_proposal',
            native_contact_band_m=band,native_loaded_separation_m=separation,
            observed_geometry_closing_error_m=closing_error,normal_world=normal.tolist())
    else:
        fit=np.load(folder/f'planner_{anchor.core_end}_{side}'/'collision_time_fit_input.npz',allow_pickle=False)
        clearance=np.asarray(fit['clearance']);gradient=np.asarray(fit['gradient']);worst=int(np.argmin(clearance))
        if clearance[worst]>=0:raise ValueError('No rejected collision fit to correct')
        normal=gradient[worst,:3];length=np.linalg.norm(normal)
        if length<1e-6:raise RuntimeError('Collision feedback has no usable translation direction')
        # The same 0.1 mm interior margin as the existing collision/time fitter.
        delta=(1e-4-float(clearance[worst]))*normal/(length*length)
        feedback=dict(method='physical_grasp_plus_rejected_collision_field_exit_proposal',
            worst_fit_tick=worst,rejected_clearance_m=float(clearance[worst]))
    if source_boundary is not None and not center_for_bridge:
        from multipath.execution.mounted_scene import pose_matrix
        world=scene.scene if hasattr(scene,'scene') else scene
        role='tool' if side=='right' else world.supporting_hands[side].object_role
        actual=pose_matrix(entry['objects'][role]['pose'])
        delta,condition=source_conditioned_translation(delta,normal,actual,np.asarray(source_boundary))
        feedback.update(condition)
    if coordinated and coordinated['physics_tick']<=1:
        commands,refit=quiet_grasp_exit_proposal(scene,anchor,seed,folder,core,side,source_boundary,
            lookahead=True,exit_separation=dict(normal_world=normal.tolist(),outward_displacement_m=float(normal@delta)))
        refit['predicted_start_collision_feedback']=feedback
        return commands,refit
    def grasped(state):return state['hands'][side]['contact']['mode']=='grasped'
    acquired=next((s['source_frame'] for i,s in enumerate(core) if all(grasped(v) for v in core[i:])),None)
    if acquired is None or acquired>=anchor.core_end:
        raise RuntimeError('No physically acquired holding interval before this core exit')
    duration=(anchor.core_end-acquired)/scene.manifest['fps']
    limits=np.asarray(scene.cfg['planning']['velocity_limits'][:3])
    acceleration=np.asarray(scene.cfg['planning']['acceleration_limits'][:3])
    minimum=max(float(np.max(1.875*np.abs(delta)/limits)),
                float(np.sqrt(np.max(5.773503*np.abs(delta)/acceleration))))
    if minimum>duration:
        raise RuntimeError('Collision exit proposal cannot fit the original grasped core time')
    # Use the previously executed seed, including any previous correction.
    old=recorded_core_commands(folder,anchor,scene.tpf,side)
    if old.shape!=seed.shape:raise ValueError('Exit seed has different source timing')
    command=np.asarray(old).copy()
    frame=anchor.core_start+np.arange(1,len(command)+1)/scene.tpf
    phase=np.clip((frame-acquired)/(anchor.core_end-acquired),0,1)
    shift=10*phase**3-15*phase**4+6*phase**5
    offset=shift[:,None]*delta
    command[:,:3]+=offset
    command[:,19:22]+=np.diff(np.vstack((np.zeros(3),offset)),axis=0)/scene.dt
    return command,dict(feedback,
        feedback_run=str(folder),physical_grasp_frame=float(acquired),original_core_duration_s=
        (anchor.core_end-anchor.core_start)/scene.manifest['fps'],available_grasped_duration_s=duration,
        minimum_correction_duration_s=minimum,offset_m=delta.tolist(),
        source_time_stretch=False,state_assignment=False,physical_validation_required=True)


def quiet_grasp_exit_proposal(scene,anchor,seed,folder,core,side,source_boundary,*,lookahead=False,exit_separation=None):
    """Refit only the physically acquired grasp tail inside the original core.

    A local grasp can pass while its outgoing momentum blocks a short bridge.
    Choose a zero-speed command boundary, not a wait or assigned physical state.
    Source conditions and fresh padded/core acceptance remain unchanged.
    """
    from multipath.planning.bridge import feedback_boundary_target,coordinate_velocity
    from multipath.planning.floating import pose_coordinates,coordinates_pose
    from multipath.planning.timing import boundary_fitted_path
    from scipy.spatial.transform import Rotation
    acquired=next((s for i,s in enumerate(core)
        if all(v['hands'][side]['contact']['mode']=='grasped' for v in core[i:])),None)
    if acquired is None or acquired['source_frame']>=anchor.core_end:
        raise RuntimeError('No physically acquired grasp tail available for exit resynthesis')
    start=dict(acquired,**acquired['hands'][side]);world=getattr(scene,'scene',scene)
    if side!='right':
        role=world.supporting_hands[side].object_role
        start['objects']=dict(tool=acquired['objects'][role])
    old=recorded_core_commands(folder,anchor,scene.tpf,side)
    if old.shape!=seed.shape:raise ValueError('Quiet exit seed has different source timing')
    cut=round((acquired['source_frame']-anchor.core_start)*scene.tpf)
    ticks=len(old)-cut;budget=ticks*scene.dt
    wrist=np.asarray(source_boundary)@np.linalg.inv(start['hand_object_transform'])
    pose=np.r_[wrist[:3,3],Rotation.from_matrix(wrist[:3,:3]).as_quat()[[3,0,1,2]]]
    goal=dict(wrist_xyz_wxyz=pose.tolist(),wrist_twist=np.zeros(6).tolist())
    terminal=np.zeros(6)
    if lookahead:
        from multipath.oracle_schedule import OracleSchedule
        from multipath.planning.bridge import lookahead_grasp_boundary
        schedule=OracleSchedule.load(world.args.schedule)
        next_window=next((w for w in schedule.windows if w['start']==anchor.core_end and w['mode']=='RELAXED'),None)
        next_core=next((w for w in schedule.windows if next_window and w['start']==next_window['end'] and w['mode']=='ANCHOR'),None)
        if next_core is None:raise ValueError('Outgoing compatibility requires a following oracle Anchor')
        next_anchor=schedule.anchors[next_core['anchor']]
        local=np.load(world.args.proposals/world.manifest['anchors'][next_anchor.id],allow_pickle=False)
        role='tool' if side=='right' else world.supporting_hands[side].object_role
        source_entry=local[role+'_pose'][next_anchor.core_start-next_anchor.synth_start]
        previous=dict(core[-1],**core[-1]['hands'][side])
        exit_condition=None
        if exit_separation is not None:
            from multipath.execution.mounted_scene import pose_matrix
            tracking=pose_matrix(previous['wrist_xyz_wxyz'])@np.linalg.inv(pose_matrix(old[-1,:7]))
            held=np.asarray(previous['hand_object_transform']);origin=pose_matrix(previous['objects']['tool']['pose'])[:3,3]
            normal=np.asarray(exit_separation['normal_world']);required=exit_separation['outward_displacement_m']
            def exit_condition(q):
                predicted=tracking@pose_matrix(coordinates_pose(q))@held
                return float(normal@(predicted[:3,3]-origin)-required)
        pose,terminal,condition=lookahead_grasp_boundary(start,previous,source_boundary,source_entry,
            budget,(next_window['end']-next_window['start'])/schedule.fps,scene.dt,scene.cfg['planning'],exit_condition=exit_condition)
        if exit_separation is not None:condition['predicted_exit_separation']=exit_separation
        goal['wrist_xyz_wxyz']=pose.tolist()
    else:
        goal,condition=feedback_boundary_target(start,goal,start['wrist_xyz_wxyz'],start,
            source_boundary,budget,scene.dt,scene.cfg['planning'],final_coordinate_velocity=terminal)
    near=pose_coordinates(start['wrist_xyz_wxyz']);raw=[near.copy()]
    for command in old[cut:]:
        near=pose_coordinates(command[:7],near);raw.append(near.copy())
    raw=np.asarray(raw);raw[-1]=pose_coordinates(goal['wrist_xyz_wxyz'],raw[-1])
    limits=scene.cfg['planning']
    path,fit=boundary_fitted_path(raw,ticks,scene.dt,np.asarray(limits['velocity_limits']),
        np.asarray(limits['acceleration_limits']),coordinate_velocity(start['wrist_xyz_wxyz'],
        start['wrist_twist'],raw[0]),terminal)
    from multipath.anchors.floating import controls_from_poses
    hands=np.vstack((start['command_hand_q'],old[cut:,7:19]))
    commands=old.copy()
    commands[cut:]=controls_from_poses(np.asarray([coordinates_pose(q) for q in path]),hands,scene.dt)
    return commands,dict(method='physically_acquired_grasp_lookahead_exit_refit' if lookahead else
        'physically_acquired_grasp_quiet_exit_refit',feedback_run=str(folder),
        physical_grasp_frame=float(acquired['source_frame']),available_grasped_duration_s=budget,
        original_core_duration_s=(anchor.core_end-anchor.core_start)/world.manifest['fps'],
        canonical_terminal_coordinate_velocity=terminal.tolist(),source_boundary_fit=condition,
        nominal_tail_fit=fit,original_finger_commands_retained=True,source_time_stretch=False,
        state_assignment=False,physical_validation_required=True)
