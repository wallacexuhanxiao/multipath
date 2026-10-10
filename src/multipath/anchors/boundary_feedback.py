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
    segments=continuous.get('segments',[])
    last=segments[-1] if segments else {}
    transition=last.get('hands',{}).get(side,{}).get('transition',{})
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
