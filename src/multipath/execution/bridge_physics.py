"""Short physical feasibility trials for an existing fixed-clock cuRobo bridge.

Only its external-wrench feedforward is refined. Finger targets, geometric
path, source clock, contact and collision acceptance remain unchanged. Every
trial reacquires its entry by executing the compiled predecessor from initial.
"""
import json
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def replay_predecessor(scene, anchor, data, left_data, prefix, left_prefix,
                       compiled, output, *, mesh=False):
    from multipath.execution.wrench_handoff import segment_wrenches
    from multipath.anchors.mpc import WindowEvaluation
    left = scene.supporting_hands['left']; tpf = scene.tpf
    scene.begin_initial_trial(output, mesh=mesh)
    right_drive = segment_wrenches(scene, scene.snapshot(), prefix, free_motion=True)
    left_drive = segment_wrenches(left, left.snapshot(), left_prefix, free_motion=True)
    padded = compiled['left_padded']; resets = scene.restore_count
    for tick in range(anchor.core_start*tpf):
        scene.wrench_bias = right_drive[tick]
        if tick < anchor.synth_start*tpf:
            command = left_prefix[tick]; left.wrench_bias = left_drive[tick]
        else:
            if tick == anchor.synth_start*tpf:
                left_drive = segment_wrenches(left, left.snapshot(), padded)
            k = tick-anchor.synth_start*tpf
            command = padded[k]; left.wrench_bias = left_drive[k]
        scene.step_control(prefix[tick], 'RELAXED', (tick+1)/tpf,
                           supporting_commands={'left': command})
    # The left contact mode is supplied with the local source data by caller.
    from dataclasses import replace
    left_anchor = replace(anchor, **left_data['anchor_contacts'])
    checks = [WindowEvaluation(scene, anchor, data, scene.snapshot()),
              WindowEvaluation(left, left_anchor, left_data, left.snapshot())]
    begin = (anchor.core_start-anchor.synth_start)*tpf
    for k, (right_command, left_command) in enumerate(zip(compiled['right'], compiled['left'])):
        scene.wrench_bias = compiled['right_core_wrenches'][k]
        left.wrench_bias = left_drive[begin+k]
        frame = anchor.core_start+(k+1)/tpf
        scene.step_control(right_command, 'BIMANUAL_EXPORTED_CORE', frame,
                           supporting_commands={'left': left_command})
        if (k+1)%tpf == 0:
            for check, command in zip(checks, (right_command, left_command)):
                if not check.observe(frame, command):
                    raise RuntimeError(f'Compiled predecessor core failed: {check.failure}')
    if scene.ticks != anchor.core_end*tpf or scene.restore_count != resets:
        raise RuntimeError('Compiled predecessor changed its source clock or reset within execution')
    return scene.snapshot()


def _desired_motion(folder, commands, starts, dt, fps, window):
    """Old tracking predictions initialize a drive proposal, never acceptance."""
    from multipath.planning.floating import pose_coordinates, coordinates_pose
    from multipath.planning.bimanual import measured_bridge_tracking
    targets = {side: np.vstack((starts[side]['wrist_xyz_wxyz'], c[:, :7]))
               for side, c in commands.items()}
    prediction = folder/f'coordinated_{window["start"]}'/'measured_tracking_prediction.json'
    if prediction.exists():
        prior = json.loads(prediction.read_text())
        profiles, _ = measured_bridge_tracking(prior['physical_rollout'], window, starts,
                                               len(next(iter(commands.values()))), dt, fps)
        for side, poses in targets.items():
            near = pose_coordinates(poses[0]); coordinates = []
            for pose in poses:
                near = pose_coordinates(pose, near); coordinates.append(near)
            profile = profiles[side].copy()
            if side != 'right' and prior.get('support_prediction'):
                # The source planner declared horizontal table support. Its
                # old approximation seeds the controller only; measured tilt,
                # contacts and geometry are never projected during validation.
                profile[:, 2] = 0.; profile[:, 4:] = 0.
            targets[side] = np.asarray([coordinates_pose(q+e) for q, e in zip(coordinates, profile)])
    return targets


def refine_bridge(scene, schedule, manifest, seed_run, replay, output,
                  *, max_seconds=180., max_trials=12, validated_program=None, next_entry_precheck=None):
    from multipath.anchors.boundary_feedback import same_source_seed
    from multipath.anchors.compatibility import boundary_errors, compatible
    from multipath.anchors.floating import controls_from_poses, validate_commands
    from multipath.planning.floating import FloatingPlanner, pose_coordinates
    from multipath.planning.bridge import coordinate_velocity
    from multipath.planning.timing import fixed_budget_path
    from multipath.validation.physics import check_retention, check_relaxed_tool_target_contact
    from multipath.execution.wrench_handoff import feedback_wrench
    from multipath.execution.wrench_handoff import bandwidth_handoff_profile
    from multipath.execution.transport_dynamics import TransportDynamics
    from scipy.ndimage import gaussian_filter1d
    folder = Path(seed_run); output = Path(output); output.mkdir(parents=True, exist_ok=True)
    old = json.loads((folder/'continuous_result.json').read_text())
    if not same_source_seed(old['source_provenance'], manifest['provenance']):
        raise ValueError('Physical bridge seed belongs to a different source')
    window = next(w for w in schedule.windows if w['start'] == round(scene.ticks/scene.tpf))
    if window['mode'] != 'RELAXED': raise ValueError('Physical bridge refinement requires a RELAXED interval')
    ticks = (window['end']-window['start'])*scene.tpf
    hands = {'right': scene, **scene.supporting_hands}
    starts = {side: hand.snapshot() for side, hand in hands.items()}
    old_entry = json.loads((folder/'bimanual_result.json').read_text())['core_exit']
    for side, start in starts.items():
        expected = old_entry if side == 'right' else old_entry['hands'][side]
        if not compatible(boundary_errors(start, expected)):
            raise ValueError('Physical bridge seed has a different reached predecessor')
    seed = dict(np.load(folder/f'segment_{window["start"]}_{window["end"]}.npz', allow_pickle=False))
    commands = {side: seed[side].copy() for side in hands}
    base = {side: seed[side+'_wrenches'].copy() for side in hands}
    nominal = {}
    for side, c in commands.items():
        if c.shape != (ticks, 25) or not validate_commands(hands[side], c, starts[side])['feasible']:
            raise ValueError('Cached bridge violates the source clock or hand limits')
        if not np.allclose(c[:, 7:19], starts[side]['command_hand_q'], atol=1e-8, rtol=0):
            raise ValueError('RELAXED physical refinement must freeze finger commands')
        near = pose_coordinates(starts[side]['wrist_xyz_wxyz']); path = [near]
        for pose in c[:, :7]:
            near = pose_coordinates(pose, near); path.append(near)
        _, nominal[side] = fixed_budget_path(path, scene.dt, ticks, scene.dt,
            scene.cfg['planning']['velocity_limits'], scene.cfg['planning']['acceleration_limits'],
            coordinate_velocity(starts[side]['wrist_xyz_wxyz'], starts[side]['wrist_twist']),
            (path[-1]-path[-2])/scene.dt)
    boundaries = json.loads((folder/f'coordinated_{window["start"]}'/'fitted_boundary.json').read_text())
    goals = {side: boundaries['primary_goal' if side == 'right' else 'support_goal'] for side in hands}
    if validated_program is not None:
        cache = Path(validated_program)
        program = json.loads((cache/'validated_bridge.json').read_text())
        if (not program['physical_report']['success'] or program['window'] != window
                or program['source_provenance'] != manifest['provenance']
                or not compatible(boundary_errors(scene.snapshot(), program['entry']))):
            raise ValueError('Cached physical bridge does not match its verified source/predecessor')
        arrays = dict(np.load(cache/'validated_bridge.npz', allow_pickle=False))
        for side in hands:
            if not np.array_equal(arrays[side], commands[side]):
                raise ValueError('Cached physical bridge changed the checked cuRobo command path')
        program.update(commands=commands, drives={side: arrays[side+'_wrenches'] for side in hands})
        if next_entry_precheck is None:
            return program, dict(success=True, cached_physical_artifact=str(cache),
                                 final_fresh_rollout_required=True, source_time_stretch=False)
        # A bridge can retain grasp and satisfy object pose yet leave a
        # velocity from which the next local Anchor cannot enter its budget.
        # Reuse its controller as the next proposal, not as that missing proof.
        base = program['drives']
    targets = _desired_motion(folder, commands, starts, scene.dt, schedule.fps, window)
    desired = {side: controls_from_poses(poses, np.vstack((starts[side]['command_hand_q'],
                    commands[side][:, 7:19])), scene.dt) for side, poses in targets.items()}
    planners = {side: FloatingPlanner(hand, True, output/f'collision_{side}',
        allowed_object_contacts=schedule.metadata.get('allowed_object_environment_contacts',{}).get(
            'tool' if side == 'right' else hand.object_role, [])) for side, hand in hands.items()}
    fade = {}
    for side, hand in hands.items():
        role = 'tool' if side == 'right' else hand.object_role
        supported = bool(schedule.metadata.get('object_support', {}).get(role))
        model = TransportDynamics(hand, not supported)
        rot = Rotation.from_quat(np.asarray(starts[side]['wrist_xyz_wxyz'])[[4,5,6,3]]).as_matrix()
        fade[side] = 1-bandwidth_handoff_profile(ticks, scene.dt, model.mass,
                       rot@model.inertia@rot.T, scene.cfg['controller'])
    report = dict(success=False, window=window, method='physical feedforward refinement of frozen cuRobo path',
        source_provenance=manifest['provenance'], source_budget_s=ticks*scene.dt,
        nominal_timing=nominal, source_time_stretch=False, finger_commands_frozen=True,
        commands_changed=False, trials=[], final_fresh_rollout_required=True)
    best = None; residual = {side: np.zeros_like(base[side]) for side in hands}
    queue = [residual]; started = time.monotonic()
    for trial in range(max_trials):
        if trial and time.monotonic()-started > max_seconds: break
        residual = queue.pop(0) if queue else {side: best['residual'][side]+best['update'][side] for side in hands}
        entry = replay(output/f'trial_{trial:03d}', mesh=True)
        resets = scene.restore_count; contact_start = scene.tool_target_contact_steps
        if hasattr(scene, 'relaxed_contact_failure'): del scene.relaxed_contact_failure
        held = {side: np.asarray(hand.state()['hand_object_transform']) for side, hand in hands.items()}
        states = {side: [] for side in hands}; failure = None; transitions = {}; done = 0
        print('BRIDGE_PHYSICS_TRIAL_BEGIN', trial, flush=True)
        try:
            for k in range(ticks):
                for side, hand in hands.items(): hand.wrench_bias = base[side][k]+residual[side][k]
                scene.step_control(commands['right'][k], 'RELAXED', window['start']+(k+1)/scene.tpf,
                    supporting_commands={side: c[k] for side, c in commands.items() if side != 'right'})
                done = k+1
                for side, hand in hands.items(): states[side].append(hand.state())
                check_relaxed_tool_target_contact(scene, contact_start)
                for side, hand in hands.items():
                    check_retention(states[side][-1], held[side], require_opposition=done%scene.tpf == 0)
                    if min(planners[side].measured_clearance().values()) < 0:
                        collisions = planners[side].measured_mesh_collisions()
                        if collisions: raise RuntimeError(f'{side}: actual RELAXED mesh collision: {collisions}')
            for side, hand in hands.items():
                goal = goals[side]
                transitions[side] = hand.check_transition(goal['wrist_xyz_wxyz'], goal['contact']['mode'],
                                                          expected_object=np.asarray(goal['source_tool_boundary']))
                if not transitions[side]['success']: raise RuntimeError(f'{side}: actual oracle entry not reached')
            if next_entry_precheck is not None:
                transitions['next_anchor_nominal_entry'] = next_entry_precheck()
        except RuntimeError as error: failure = str(error)
        if scene.restore_count != resets: raise RuntimeError('State reset inside physical bridge trial')
        scene.save()
        errors = {}; update = {}
        for side, samples in states.items():
            if not samples: raise RuntimeError('Bridge trial did not produce finite physical state samples')
            correction = np.asarray([feedback_wrench(desired[side][k, :7], desired[side][k, 19:25],
                state['wrist_xyz_wxyz'], state['wrist_twist'], scene.cfg['controller'])
                for k, state in enumerate(samples)])
            errors[side] = float(np.mean(np.linalg.norm(correction, axis=1)))
            # A bounded, filtered inverse-response proposal. A trial remains
            # rejected until the real collision/grasp/boundary gates all pass.
            correction = np.vstack((correction, np.repeat(correction[-1:], ticks-done, axis=0)))
            correction = gaussian_filter1d(correction, 1., axis=0, mode='nearest')
            cap = np.r_[np.full(3, scene.cfg['controller']['wrist_max_force']),
                        np.full(3, scene.cfg['controller']['wrist_max_torque'])]
            correction = np.clip(correction, -cap, cap)
            correction *= fade[side]
            update[side] = correction
        result = dict(trial=trial, success=failure is None, error=failure, physics_steps=done,
            failure_source_frame=window['start']+done/scene.tpf, transitions=transitions,
            tracking_drive_error=errors, entry_physically_reachable=True, entry_origin='predecessor_rollout',
            state_resets_between_segments=scene.restore_count-resets)
        report['trials'].append(result)
        (output/f'trial_{trial:03d}'/'bridge_result.json').write_text(json.dumps(result, indent=2))
        (output/'bridge_physics_report.json').write_text(json.dumps(report, indent=2))
        print('BRIDGE_PHYSICS_TRIAL_RESULT', trial, result, flush=True)
        if failure is None:
            program = dict(commands=commands, drives={side: base[side]+residual[side] for side in hands},
                           goals=goals, entry=entry, nominal_timing=nominal, source_provenance=manifest['provenance'],
                           physical_report=result, origin=str(folder), window=window)
            report.update(success=True, accepted_trial=trial, compute_seconds=time.monotonic()-started)
            np.savez_compressed(output/'validated_bridge.npz', **commands,
                **{side+'_wrenches': values for side, values in program['drives'].items()})
            (output/'validated_bridge.json').write_text(json.dumps(
                {key:value for key,value in program.items() if key not in ('commands','drives')}, indent=2))
            (output/'bridge_physics_report.json').write_text(json.dumps(report, indent=2))
            return program, report
        score = (done, -sum(errors.values()))
        if best is None or score > best['score']:
            best = dict(score=score, residual=residual, update=update)
            queue = [{side: residual[side]+scale*update[side] for side in hands} for scale in (.5, 1., 2.)]
    report['compute_seconds'] = time.monotonic()-started
    (output/'bridge_physics_report.json').write_text(json.dumps(report, indent=2))
    return None, report
