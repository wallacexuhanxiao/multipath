"""Local Replay -> MPPI on mounted PhysX; never used in RELAXED intervals.

EgoEngine reproduction's temporal Cartesian/finger residuals and softmax
update, adapted through mounted IK and fixed-time command fitting. Candidate
count is a compute budget, not a claim to reproduce unspecified paper values.
"""
from dataclasses import asdict, dataclass
import json
from pathlib import Path

import numpy as np

from multipath.anchors.control_check import control_feasibility
from multipath.anchors.synthesis import (
    fit_arm_startup, fit_hand_startup, fit_timed_controls,
    hand_limits_from_urdf, local_wrist_targets,
)
from multipath.execution.transition import check_transition
from multipath.validation.physics import core_pose_errors, retention_errors


@dataclass(frozen=True)
class MPCConfig:
    chunk_frames: int = 20
    candidates: int = 6
    iterations: int = 2
    wrist_sigma_m: float = .02
    finger_sigma_rad: float = .25

    def __post_init__(self):
        if self.chunk_frames < 1 or self.candidates < 2 or self.iterations < 1:
            raise ValueError('Invalid MPPI compute budget')
        if min(self.wrist_sigma_m, self.finger_sigma_rad) <= 0:
            raise ValueError('MPPI scales must be positive')


class RefinementRequired(RuntimeError):
    """No interaction-valid candidate: escalate; do not advance a bad state."""


def mppi_mean(samples, returns):
    """Original reproduction's softmax update, excluding invalid commands."""
    returns = np.asarray(returns, dtype=float)
    valid = np.isfinite(returns)
    if not valid.any():
        return None
    scores = returns[valid]
    temperature = max(1e-3, .1 * float(np.std(scores, ddof=1) if len(scores) > 1 else 0))
    weights = np.exp((scores - scores.max()) / temperature)
    weights /= weights.sum()
    return np.tensordot(weights, np.asarray(samples)[valid], axes=(0, 0))


class WindowEvaluation:
    """Existing Anchor core/contact gates, plus a dense object/contact return.

    The Cp/CR box is the current project's acceptance rule, not Eq C.4's
    scalar boundary. Context helps synthesis without adding reference gates.
    """
    def __init__(self, scene, anchor, data, start):
        self.scene, self.anchor, self.data = scene, anchor, data
        frame = start['time_s'] * scene.manifest['fps']
        self.held = None
        if ((anchor.entry_contact == 'grasped' and frame < anchor.core_end)
                or (frame >= anchor.core_end and anchor.exit_contact == 'grasped')):
            self.held = np.asarray(start['hand_object_transform'])
        self.total_return = 0.
        self.failure = None

    def observe(self, frame, command):
        a = self.anchor
        measured = self.scene.state()
        reward = 0.
        reason = None
        index = min(max(round(frame) - a.synth_start, 0), len(self.data['tool_pose']) - 1)
        if frame == a.core_start:
            if measured['contact']['mode'] != a.entry_contact:
                reason = 'core_entry_contact'
            self.held = np.asarray(measured['hand_object_transform']) if a.entry_contact == 'grasped' else None
        if a.core_start < frame <= a.core_end:
            error = core_pose_errors(measured, self.data['tool_pose'][index])
            reward = np.sqrt(2.) - np.hypot(error['position_m'] / .05, error['rotation_rad'] / .35)
            if error['position_m'] > .05 or error['rotation_rad'] > .35:
                reason = 'core_object_fidelity'
            # Native opposition, not proximity. No human-mimic or smoothness
            # reward; final release is checked explicitly below.
            reward += 2. * (measured['contact']['mode'] == 'grasped')
            if self.held is not None:
                r = retention_errors(measured, self.held)
                if r['position_m'] > .05 or r['rotation_rad'] > .35 or not r['contact_retained']:
                    reason = 'core_grasp_retention'
        if frame == a.core_end:
            handoff = check_transition(self.scene, command[:7], a.exit_contact, self.data['tool_pose'][index])
            if not handoff['success']:
                reason = 'core_exit_handoff'
            self.held = np.asarray(measured['hand_object_transform']) if a.exit_contact == 'grasped' else None
        if frame > a.core_end and self.held is not None:
            r = retention_errors(measured, self.held)
            if r['position_m'] > .05 or r['rotation_rad'] > .35 or not r['contact_retained']:
                reason = 'post_context_grasp_retention'
        if reason:
            self.failure = {'source_frame': float(frame), 'reason': reason}
            return False
        self.total_return += float(reward)
        return True


def mounted_mpc(scene, anchor, data, baseline, start, out, seed=0, config=None):
    config = config or MPCConfig()
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    fps = scene.manifest['fps']; tpf = round(1 / (fps * scene.dt))
    if len(baseline) != (anchor.synth_end - anchor.synth_start) * tpf:
        raise ValueError('MPPI baseline must cover exactly this padded Anchor')
    limits = np.asarray(scene.robot._articulation_view.get_dof_limits()).reshape(-1, 2)
    limits = limits[np.r_[scene.arm, scene.fingers]]
    hand_velocity = hand_limits_from_urdf(scene)
    offset = np.asarray(start['wrist_xyz_wxyz'][:3]) - data['wrist_hand_q'][0, :3]
    held = np.asarray(start['hand_object_transform']) if start['contact']['mode'] == 'grasped' else None
    prior = local_wrist_targets(anchor, data, held, offset, padded=True)
    rng = np.random.default_rng(seed)
    chosen = baseline.copy(); history = []; chunks = []; trial_id = 0
    method = dict(asdict(config), solver='mounted_Cartesian_MPPI', lookahead_chunks=2,
                  action='per_source_frame_wrist_xyz_and_12_fingers',
                  wrist_orientation='local_reference', source_time_stretch=False,
                  execution='serial_mounted_PhysX', reward='core_object_tracking_plus_native_opposition',
                  feasibility='existing_core_Cp_CR_contact_and_handoff', local_rl_implemented=False)
    (out / 'method.json').write_text(json.dumps(method, indent=2))
    state = start

    def flush():
        (out / 'search.json').write_text(json.dumps(history, indent=2))
        (out / 'chunks.json').write_text(json.dumps(chunks, indent=2))

    def precheck(control, boundary):
        return control_feasibility(control, np.asarray(boundary['arm_q']), np.asarray(boundary['arm_dq']),
            scene.dt, limits, scene.velocity, scene.acceleration,
            start_hand_q=np.asarray(boundary['hand_q']), hand_velocity_limits=hand_velocity)

    def evaluate(control, boundary, begin, label):
        nonlocal trial_id
        trial_id += 1
        checks = precheck(control, boundary)
        trial = dict(trial=trial_id, begin_tick=begin, mode=label, feasible_controls=checks['feasible'],
                     precheck=checks, interaction_feasible=False, score=None)
        history.append(trial)
        if not checks['feasible']:
            return -np.inf, False
        folder = out / f'prediction_{trial_id:04d}'
        scene.begin_trial(folder, boundary, mesh=False)
        objective = WindowEvaluation(scene, anchor, data, boundary)
        try:
            for j, cmd in enumerate(control):
                frame = anchor.synth_start + (begin + j + 1) / tpf
                scene.step(cmd[:7], cmd[7:19], label, frame, cmd[19:26])
                if (j + 1) % tpf == 0 and not objective.observe(frame, cmd):
                    break
            trial['interaction_feasible'] = objective.failure is None
            trial['failure'] = objective.failure
            trial['return'] = objective.total_return
            trial['score'] = -objective.total_return
        except RuntimeError as error:
            trial['failure'] = {'reason': str(error)}
            objective.total_return = -np.inf
        (folder / 'actual_trace.json').write_text(json.dumps(scene.records))
        flush()
        return objective.total_return, trial['interaction_feasible']

    for begin in range(0, len(baseline), config.chunk_frames * tpf):
        stop = min(begin + 2 * config.chunk_frames * tpf, len(baseline))
        commit = min(config.chunk_frames * tpf, len(baseline) - begin)
        chunk = dict(begin_tick=begin, end_tick=begin+commit, prediction_end_tick=stop, selected=None)
        chunks.append(chunk)
        q = np.asarray(state['arm_q']); dq = np.asarray(state['arm_dq'])
        hq = np.asarray(state['hand_q']); hdq = np.asarray(state['hand_dq'])
        # Refit commands from actual q/dq, instead of shifting hand targets
        # beyond their joint limits with a measured-to-target offset.
        replay = baseline[begin:stop].copy()
        try:
            replay[:, :7] = fit_arm_startup(replay[:, :7], q, dq, scene.dt, scene.velocity,
                scene.acceleration, limits[:7], startup_duration=scene.dt)
            replay[:, 7:19] = fit_hand_startup(replay[:, 7:19], hq, hdq, scene.dt, limits[7:], hand_velocity)
            replay[:, 19:26] = np.diff(np.vstack((q, replay[:, :7])), axis=0) / scene.dt
            replay_return, replay_ok = evaluate(replay, state, begin, 'CHUNK_REPLAY')
        except (RuntimeError, ValueError) as error:
            replay_return, replay_ok = -np.inf, False
            chunk['replay_fit_error'] = str(error)
        best = replay if replay_ok else None
        best_return = replay_return if replay_ok else -np.inf
        if replay_ok:
            chunk['selected'] = 'REPLAY'
        else:
            local = prior[begin//tpf:stop//tpf]
            mean = np.zeros((len(local), 15))
            scale = np.r_[np.full(3, config.wrist_sigma_m), np.full(12, config.finger_sigma_rad)]
            for iteration in range(config.iterations):
                samples = rng.normal(size=(config.candidates, len(local), 15)) * scale * (.5 ** iteration) + mean
                samples[0] = mean
                returns = np.full(config.candidates, -np.inf)
                for i, residual in enumerate(samples):
                    try:
                        targets = local.copy(); targets[:, :3] += residual[:, :3]; targets[:, 7:] += residual[:, 3:]
                        arm = []; seed_q = q.copy()
                        for target in targets:
                            seed_q = scene.solve(target[:7], seed_q)
                            arm.append(seed_q)
                        control, _ = fit_timed_controls(np.asarray(arm), targets[:, 7:], q, dq, hq,
                            fps, scene.dt, scene.velocity, scene.acceleration, limits[:7],
                            start_hand_dq=hdq, hand_joint_limits=limits[7:], hand_velocity_limits=hand_velocity,
                            hand_startup_duration=scene.dt)
                    except (RuntimeError, ValueError) as error:
                        history.append(dict(begin_tick=begin, mode='MPPI', iteration=iteration, candidate=i,
                            feasible_controls=False, interaction_feasible=False, score=None, fit_error=str(error)))
                        flush(); continue
                    returns[i], valid = evaluate(control, state, begin, 'MPPI')
                    if valid and returns[i] > best_return:
                        best, best_return = control.copy(), returns[i]
                updated = mppi_mean(samples, returns)
                if updated is not None:
                    mean = updated
                print('MPPI_ITERATION', begin, iteration, 'physical_trials', trial_id, 'valid_plan', best is not None, flush=True)
            if best is not None:
                chunk['selected'] = 'MPPI'
        if best is None:
            chunk['selected'] = 'RL_REQUIRED'; flush()
            np.savez_compressed(out/'accepted_prefix_controls.npz', commands=chosen[:begin])
            (out/'refinement_entry.json').write_text(json.dumps(state, indent=2))
            raise RefinementRequired(f'No interaction-valid Replay/MPPI at source frame {anchor.synth_start+begin/tpf:g}; local RL required')
        # Search restores are development-only. Re-execute selected commands and
        # verify the committed portion; never advance from a failed prediction.
        scene.begin_trial(out/f'committed_{begin:04d}', state, mesh=False)
        objective = WindowEvaluation(scene, anchor, data, state)
        try:
            for j, cmd in enumerate(best[:commit]):
                frame = anchor.synth_start + (begin+j+1)/tpf
                scene.step(cmd[:7], cmd[7:19], 'LOCAL_COMMIT', frame, cmd[19:26])
                if (j+1) % tpf == 0 and not objective.observe(frame, cmd):
                    raise RefinementRequired(f'Selected prediction failed actual commit: {objective.failure}')
        except RuntimeError as error:
            chunk['commit_success'] = False; chunk['commit_error'] = str(error); flush()
            raise
        finally:
            (scene.out/'actual_trace.json').write_text(json.dumps(scene.records))
        chunk['commit_success'] = True
        state = scene.snapshot(); chosen[begin:begin+commit] = best[:commit]
        flush()
    return chosen
