"""Local Replay -> MPPI on mounted PhysX; never used in RELAXED intervals.

EgoEngine reproduction's temporal Cartesian/finger residuals and softmax
update, adapted through mounted IK and fixed-time command fitting. Candidate
count is a compute budget, not a claim to reproduce unspecified paper values.
"""
from dataclasses import asdict, dataclass
import json
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from multipath.anchors.control_check import control_feasibility
from multipath.anchors.synthesis import (
    fit_arm_startup, fit_hand_startup, fit_timed_controls,
    hand_limits_from_urdf,
)
from multipath.execution.transition import check_transition
from multipath.validation.physics import core_pose_errors, retention_errors


@dataclass(frozen=True)
class MPCConfig:
    chunk_frames: int = 20
    candidates: int = 64
    iterations: int = 3
    wrist_sigma_m: float = .02
    finger_sigma_rad: float = .25
    wrist_rotation_sigma_rad: float = .2

    def __post_init__(self):
        if self.chunk_frames < 1 or self.candidates < 2 or self.iterations < 1:
            raise ValueError('Invalid MPPI compute budget')
        if min(self.wrist_sigma_m, self.finger_sigma_rad, self.wrist_rotation_sigma_rad) <= 0:
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


def fitted_cartesian_prior(scene, controls, ticks_per_frame):
    """Lift this local fitted Replay to Cartesian targets; no full reference.

    Optimizing around raw retarget and fitting again changed even the zero
    residual's startup. The actual fitted command is the search center.
    """
    frames = controls[ticks_per_frame-1::ticks_per_frame]
    state = scene.ik.kinematics.get_state(scene.tensor.to_device(frames[:,:7].copy()))
    xyz = state.ee_position.detach().cpu().numpy()
    flange = Rotation.from_quat(state.ee_quaternion.detach().cpu().numpy()[:,[1,2,3,0]])
    wrist = (flange * scene.mount).as_quat()[:,[3,0,1,2]]
    return np.c_[xyz,wrist,frames[:,7:19]]


class WindowEvaluation:
    """Appendix C tracking score with project core/contact boundary checks.

    Padding has no reference-fidelity gate. Auxiliary contact reward is
    optimization-only; evaluation uses object tracking alone. A final release
    may also check its existing task contract at the end of post-context.
    """
    def __init__(self, scene, anchor, data, start, *, continue_tracking_failures=False):
        self.scene, self.anchor, self.data = scene, anchor, data
        frame = start['time_s'] * scene.manifest['fps']
        self.held = None
        if ((anchor.entry_contact == 'grasped' and frame < anchor.core_end)
                or (frame >= anchor.core_end and anchor.exit_contact == 'grasped')):
            self.held = np.asarray(start['hand_object_transform'])
        self.total_return = 0.
        self.evaluation_return = 0.
        self.last_reward = 0.
        self.failure = None
        self.continue_tracking_failures = continue_tracking_failures
        self.tracking_failures = []

    def observe(self, frame, command, measured=None):
        a = self.anchor
        measured = self.scene.state() if measured is None else measured
        reward = 0.
        tracking_reward = 0.
        reason = None
        index = min(max(round(frame) - a.synth_start, 0), len(self.data['tool_pose']) - 1)
        if frame < a.core_start and self.held is not None:
            r=retention_errors(measured,self.held)
            if r['position_m']>.05 or r['rotation_rad']>.35 or not r['contact_retained']:
                reason='pre_context_grasp_retention'
        if frame == a.core_start:
            # Canonical entry is conditioned on this Anchor's source boundary,
            # not inherited from the previous Anchor's exit. Padding states
            # provide physical context only; the boundary must actually arrive.
            from multipath.validation.physics import object_tracking_score
            error=core_pose_errors(measured,self.data['tool_pose'][index])
            score=object_tracking_score(error);reward=tracking_reward=score['reward']
            if not score['feasible']:reason='core_entry_source_boundary'
            if measured['contact']['mode'] != a.entry_contact:
                reason = 'core_entry_contact'
            predecessor=getattr(self.scene,'bridge_entry_start',None)
            if predecessor is not None:
                from multipath.planning.bridge import bridge_precheck
                budget=a.core_start/self.scene.manifest['fps']-predecessor['time_s']
                check=bridge_precheck(predecessor,measured,budget,self.scene.cfg['planning'])
                self.scene.last_bridge_entry_precheck=check
                if not check['feasible']:
                    if reason is None:reason='core_entry_bridge_boundary'
                    limit=np.asarray(self.scene.cfg['planning']['velocity_limits'])
                    violation=max(check['minimum_duration_velocity_only_s']/budget,
                                  check['minimum_duration_velocity_change_only_s']/budget,
                                  max(np.abs(check['initial_coordinate_velocity'])/limit),
                                  max(np.abs(check['terminal_coordinate_velocity'])/limit),
                                  check['hand_q_max_difference_rad']/.10)-1
                    reward-=2*max(0.,violation)
            self.held = np.asarray(measured['hand_object_transform']) if a.entry_contact == 'grasped' else None
        if a.core_start < frame <= a.core_end:
            error = core_pose_errors(measured, self.data['tool_pose'][index])
            from multipath.validation.physics import object_tracking_score
            score = object_tracking_score(error)
            reward = tracking_reward = score['reward']
            if not score['feasible']:
                reason = 'core_object_fidelity'
            # Native opposition, not proximity. No human-mimic or smoothness
            # reward; final release is checked explicitly below.
            # Release Anchors must be allowed to change contact inside CORE.
            # Requiring retention until the exact exit frame makes release
            # interactions fail by construction. Stable grasp Anchors retain it.
            if self.held is not None and a.exit_contact == 'grasped':
                r = retention_errors(measured, self.held)
                if r['position_m'] > .05 or r['rotation_rad'] > .35 or not r['contact_retained']:
                    reason = 'core_grasp_retention'
        if frame == a.core_end:
            transition = getattr(self.scene, 'check_transition', None)
            handoff = (transition(command[:7], a.exit_contact, self.data['tool_pose'][index]) if transition else
                       check_transition(self.scene, command[:7], a.exit_contact, self.data['tool_pose'][index]))
            if not handoff['success']:
                reason = 'core_exit_handoff'
            if getattr(self.scene,'bridge_exit_constraints',False):
                from multipath.planning.bridge import coordinate_velocity,carried_object_clearance
                velocity=coordinate_velocity(measured['wrist_xyz_wxyz'],measured['wrist_twist'])
                ratio=float(np.max(np.abs(velocity)/self.scene.cfg['planning']['velocity_limits']))
                self.scene.last_bridge_exit_velocity_ratio=ratio
                if ratio>1.0001:
                    if reason is None:reason='core_exit_bridge_velocity'
                    reward-=2*(ratio-1)
                if a.exit_contact=='grasped':
                    clearance=carried_object_clearance(self.scene,measured)
                    self.scene.last_bridge_exit_clearance=clearance
                    if min(clearance.values())<0:
                        if reason is None:reason='core_exit_carried_object_collision'
                        reward-=2*max(0.,-min(clearance.values()))/.025
                else:
                    from multipath.planning.bridge import hand_table_clearance
                    clearance=hand_table_clearance(self.scene)
                    self.scene.last_bridge_exit_hand_clearance=clearance
                    if not clearance['collision_free']:
                        if reason is None:reason='core_exit_hand_table_collision'
                        reward-=2*max(0.,-clearance['clearance_m'])/.025
            self.held = np.asarray(measured['hand_object_transform']) if a.exit_contact == 'grasped' else None
        if frame > a.core_end and self.held is not None:
            r = retention_errors(measured, self.held)
            if r['position_m'] > .05 or r['rotation_rad'] > .35 or not r['contact_retained']:
                reason = 'post_context_grasp_retention'
        contract=getattr(self.scene,'terminal_task_contract',None)
        if contract and frame==a.synth_end and a.synth_end==self.scene.manifest['num_frames'] and a.exit_contact=='free':
            # The final release may satisfy CORE's interaction tolerance but
            # miss the task's actual settled-object condition. Use the existing
            # final task contract inside its original post-context budget.
            from multipath.execution.mounted_scene import pose_matrix
            error=core_pose_errors(measured,pose_matrix(self.scene.manifest['final_objects']['tool']))
            speed=float(np.linalg.norm(measured['objects']['tool']['velocity'][:3]))
            task_reward=1.-float(np.hypot(error['position_m']/contract['position_tolerance_m'],
                                        error['rotation_rad']/contract['rotation_tolerance_rad']))
            reward+=task_reward;tracking_reward+=task_reward
            if (error['position_m']>=contract['position_tolerance_m'] or
                error['rotation_rad']>=contract['rotation_tolerance_rad'] or
                speed>=contract['stable_speed_m_s'] or measured['contact']['mode']!='free'):
                reason='terminal_task_state'
        # Padding has no human-trajectory fidelity objective, but preserving a
        # required physical grasp still needs a learning signal. Zero returns
        # for all early retention failures made the local PPO fallback blind.
        if a.core_start<frame<=a.core_end or self.held is not None:
            reward+=2.*measured['contact'].get('opposition_contact',False)
        self.last_reward = float(reward)
        # Include the failed boundary's measured score: otherwise every
        # pre-context candidate missing the boundary receives the same zero.
        self.total_return += float(reward)
        self.evaluation_return += float(tracking_reward)
        if reason:
            if self.continue_tracking_failures and reason == 'core_object_fidelity':
                # Search needs the remaining interaction's dense reward. A
                # temporary tracking miss is recorded, never a successful
                # candidate. Fresh padded/core Replay still uses hard gates.
                self.tracking_failures.append({'source_frame': float(frame), 'reason': reason})
                return True
            self.failure = {'source_frame': float(frame), 'reason': reason}
            return False
        return True


def mounted_mpc(scene, anchor, data, baseline, start, out, seed=0, config=None, rl_solver=None):
    config = config or MPCConfig()
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    fps = scene.manifest['fps']; tpf = round(1 / (fps * scene.dt))
    if len(baseline) != (anchor.synth_end - anchor.synth_start) * tpf:
        raise ValueError('MPPI baseline must cover exactly this padded Anchor')
    limits = np.asarray(scene.robot._articulation_view.get_dof_limits()).reshape(-1, 2)
    limits = limits[np.r_[scene.arm, scene.fingers]]
    hand_velocity = hand_limits_from_urdf(scene)
    rng = np.random.default_rng(seed)
    chosen = baseline.copy(); history = []; chunks = []; trial_id = 0
    method = dict(asdict(config), solver='mounted_Cartesian_MPPI', lookahead_chunks=2,
                  action='per_source_frame_wrist_xyz_rotvec_and_12_fingers',
                  wrist_orientation='world_frame_rotation_residual_about_fitted_replay', source_time_stretch=False,
                  residual_center='fitted_local_replay',zero_residual='exact_replay_controls',
                  execution='serial_mounted_PhysX', reward='core_object_tracking_plus_native_opposition',
                  feasibility='existing_core_Cp_CR_contact_and_handoff', local_rl_enabled=rl_solver is not None)
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
        started=time.perf_counter()
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
        trial['physical_evaluation_s']=time.perf_counter()-started
        (folder / 'actual_trace.json').write_text(json.dumps(scene.records))
        flush()
        return objective.total_return, trial['interaction_feasible']

    search_started=time.perf_counter()
    for begin in range(0, len(baseline), config.chunk_frames * tpf):
        stop = min(begin + 2 * config.chunk_frames * tpf, len(baseline))
        commit = min(config.chunk_frames * tpf, len(baseline) - begin)
        chunk = dict(begin_tick=begin, end_tick=begin+commit, prediction_end_tick=stop, selected=None)
        chunks.append(chunk)
        core_interval=((anchor.core_start-anchor.synth_start-begin/tpf)/fps,
                       (anchor.core_end-anchor.synth_start-begin/tpf)/fps)
        local_clock=np.arange(1,stop-begin+1)*scene.dt
        weights=np.where((local_clock>core_interval[0])&(local_clock<=core_interval[1]),1.,.05)
        q = np.asarray(state['arm_q']); dq = np.asarray(state['arm_dq'])
        hq = np.asarray(state['hand_q']); hdq = np.asarray(state['hand_dq'])
        # Refit commands from actual q/dq, instead of shifting hand targets
        # beyond their joint limits with a measured-to-target offset.
        replay = baseline[begin:stop].copy()
        try:
            # At the original acquired entry, the baseline is already fitted.
            # Later boundaries require a measured-state rebase, shared by both
            # Replay and all sampled residuals.
            if begin:
                replay[:, :7] = fit_arm_startup(replay[:, :7], q, dq, scene.dt, scene.velocity,
                    scene.acceleration, limits[:7], startup_duration=scene.dt,frame_weights=weights)
                replay[:, 7:19] = fit_hand_startup(replay[:, 7:19], hq, hdq, scene.dt, limits[7:], hand_velocity,frame_weights=weights)
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
            local = fitted_cartesian_prior(scene,replay,tpf)
            remaining_context = max(scene.dt,(anchor.core_start-anchor.synth_start-begin/tpf)/fps)
            mean = np.zeros((len(local), 18))
            scale = np.r_[np.full(3, config.wrist_sigma_m), np.full(3, config.wrist_rotation_sigma_rad), np.full(12, config.finger_sigma_rad)]
            for iteration in range(config.iterations):
                samples = rng.normal(size=(config.candidates, len(local), 18)) * scale * (.5 ** iteration) + mean
                samples[0] = mean
                returns = np.full(config.candidates, -np.inf)
                for i, residual in enumerate(samples):
                    if not np.any(residual):
                        # Do not silently replace the known baseline with a
                        # new IK/fitting solution when delta is exactly zero.
                        returns[i], valid = evaluate(replay.copy(), state, begin, 'MPPI_ZERO')
                        if valid and returns[i] > best_return:
                            best, best_return = replay.copy(), returns[i]
                        continue
                    candidate_started=time.perf_counter();ik_seconds=None;fit_seconds=None
                    try:
                        targets = local.copy(); targets[:, :3] += residual[:, :3]
                        # Same world-frame rotation residual as the prior reproduction.
                        rotations=Rotation.from_rotvec(residual[:,3:6])*Rotation.from_quat(local[:,[4,5,6,3]])
                        targets[:,3:7]=rotations.as_quat()[:,[3,0,1,2]]
                        targets[:,7:]+=residual[:,6:]
                        arm = []; seed_q = q.copy()
                        for target in targets:
                            seed_q = scene.solve(target[:7], seed_q)
                            arm.append(seed_q)
                        ik_seconds=time.perf_counter()-candidate_started
                        fit_started=time.perf_counter()
                        control, _ = fit_timed_controls(np.asarray(arm), targets[:, 7:], q, dq, hq,
                            fps, scene.dt, scene.velocity, scene.acceleration, limits[:7],
                            start_hand_dq=hdq, hand_joint_limits=limits[7:], hand_velocity_limits=hand_velocity,
                            hand_startup_duration=remaining_context,core_interval_s=core_interval)
                        fit_seconds=time.perf_counter()-fit_started
                    except (RuntimeError, ValueError) as error:
                        history.append(dict(begin_tick=begin, mode='MPPI', iteration=iteration, candidate=i,
                            feasible_controls=False, interaction_feasible=False, score=None, fit_error=str(error),
                            candidate_generation_s=time.perf_counter()-candidate_started,ik_s=ik_seconds,fit_s=fit_seconds))
                        flush(); continue
                    returns[i], valid = evaluate(control, state, begin, 'MPPI')
                    history[-1].update(ik_s=ik_seconds,fit_s=fit_seconds)
                    print('MPPI_CANDIDATE',iteration,i,'ik_s',round(ik_seconds,3),'fit_s',round(fit_seconds,3),
                          'physics_s',round(history[-1].get('physical_evaluation_s',0),3),'valid',valid,flush=True)
                    if valid and returns[i] > best_return:
                        best, best_return = control.copy(), returns[i]
                updated = mppi_mean(samples, returns)
                if updated is not None:
                    mean = updated
                print('MPPI_ITERATION', begin, iteration, 'physical_trials', trial_id, 'valid_plan', best is not None,'elapsed_s',round(time.perf_counter()-search_started,3), flush=True)
            if best is not None:
                chunk['selected'] = 'MPPI'
        if best is None and rl_solver is not None:
            chunk['selected']='PPO_TRAINING';flush()
            print('LOCAL_PPO_ESCALATE',begin,flush=True)
            proposed=rl_solver(replay,state,begin)
            if proposed is not None:
                value,valid=evaluate(proposed,state,begin,'PPO_VERIFY')
                if valid:
                    best,best_return=proposed.copy(),value
                    chunk['selected']='PPO'
                else:
                    chunk['ppo_repeat_failed']=True
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
