# Local Anchor synthesis: EgoEngine reproduction alignment

Checked 2026-10-03 against the user's `egoengine复现&rl` chat
(`01a074b6-320e-70b0-82be-32e137c6dd42`), the local EgoEngine paper
(`../egoengine.pdf`, section 3.2 and Appendix C), and the existing XHand
reproduction in `../diversityverifier_phase1_scoped/vendor/egoengine`.
That checkout has HEAD `098a28a5c9001eb6600897447132cea550e715fd` **plus
uncommitted refinement/RL changes**. Do not identify its current files with
that commit alone. The reproduction is not the authors' official code.

## What to reuse

| Stage | Existing implementation | Mounted Anchor adaptation |
| --- | --- | --- |
| Retarget | `src/egoengine/retarget/ik.py`: MANO to XHand with Mink, named finger joints and warm-started IK | Already called by `prepare_oracle_anchors.py`, only for padded windows; arm IK and fixed-time fitting follow it. |
| Solver switching | `scripts/run_refine.py`: try Replay, then MPC, then residual PPO, from the current physical boundary | Apply inside each padded Anchor. The paper uses H=20 source control frames, plans/checks 2H and commits H; truncate at the padded end. These internal chunks do not define new oracle Anchors. |
| MPC | `scripts/run_refine.py`: time-varying Cartesian wrist/finger residuals, Gaussian sampling, annealing and softmax MPPI update | Map wrist targets through continuous arm IK, execute arm + XHand in mounted PhysX. Keep actual entry q/dq and the source-time budget. Floating-wrist predictions are not physical evidence for the mounted robot. |
| RL | `src/egoengine/rl/chunk_env.py` and `scripts/train_residual.py`: state-conditioned residual PPO around local reference, with actual-prefix state handling | Adapt observation/action/execution to the mounted articulation. Train only after Replay and MPC fail the interaction; validate a fresh padded rollout and exported core. Existing floating-wrist checkpoints are not directly executable mounted policies. |

Reference modules are reuse sources, not commands to run a full-video
EgoEngine reference. No reference fidelity objective belongs in RELAXED.

## Paper, reproduction choices, and our changes are different layers

- Appendix C.1 specifies 20-step chunks, a two-chunk feasibility horizon,
  greedy Replay → MPC → RL escalation, and executing only the current chunk.
  It is not a full MCTS implementation.
- Appendix C.2 uses weighted object position/rotation error for tracking and
  termination, plus thumb-and-other-finger contact. TACO disables the
  human-mimic and action-smoothness rewards and domain randomization.
- The reproduction's MPPI temperature, candidate count, noise scales and PPO
  hyperparameters are implementation choices; do not attribute every value
  to the paper. Its separate Cp/CR box checks also differ from Eq. C.4's
  weighted scalar error boundary.
- The current XHand retargeter uses SPIDER fingertip **position** targets
  without synthetic fingertip orientation targets. Its self-collision
  constraints are not the complete constraint set described in section 3.2.1.
  Preserve the working XHand implementation and report these differences.
- Later reproduction changes include tool-centric ranking, actual-execution
  acceptance, full velocity/controller-state handling and prefix-conditioned
  PPO. Older `RETARGET.md` / `RL.md` passages still describe Inspire hands or
  zeroed velocities; use current source when those descriptions conflict.
- Our oracle core/padding distinction, fixed source-time arm dynamics, and
  mounted physical verification are project requirements. They must survive
  the port; no GT mid-air initialization, extra settling time, or final resets.

## Current multipath status — not yet an Appendix-equivalent solver

At `47a7b16`, `anchors/development.py::mounted_mpc` is a small diagnostic
shooting optimizer: 6-frame commits, 12-frame lookahead, one 19-dimensional
arm/finger offset shaped by a sine-squared envelope, 6 candidates over two
iterations, and top-two elite updates. This is **not** the reference
time-varying MPPI implementation. The current objective is also different.

The driver first attempts an entire padded Replay and then invokes this
optimizer. It does not yet perform Appendix C.1's per-chunk mode switching.
Mounted residual PPO is **not implemented** (`rl_status=not_attempted`).
Neither MPC failure nor missing RL establishes that the Appendix approach
fails on the mounted task.

The next implementation should port the existing chunk switcher, MPPI and
residual PPO through the mounted execution interface, rather than continue
tuning the diagnostic optimizer. Reuse its measured-entry continuity and
nominal command precheck, but preserve the reference solver's temporal
action sequence and report any necessary mounted adaptation explicitly.

Only three physical decisions remain: local padded/core validity, predecessor
boundary reachability, and final uninterrupted 4.8-second rollout. No new
reward sweeps, sample-specific branches, or cross-sequence tests are needed
before Oracle V0 succeeds. M1 and the RELAXED cuRobo backend remain frozen.

## Evidence boundary

This comparison is a source/paper review, not a new successful physical run.
The last completed padded A1 Replay (r23) lost grasp and failed core fidelity.
On inspection, r24's A1 and snapshot-acquisition processes timed out during
PyTorch import before simulation; it provides no new interaction evidence.

## Implementation follow-up (2026-10-03)

`anchors/mpc.py` now replaces the diagnostic fixed-offset search with:

- 20-frame commits and a 40-frame lookahead, truncated at the padded boundary;
- Replay checked first at each local boundary, then MPPI only if it fails;
- per-source-frame wrist xyz / 12-finger residuals, Gaussian annealing and the
  reproduction's softmax update; wrist orientation remains reference-conditioned;
- continuous mounted IK and fixed-time fitting from measured arm/hand q/dq;
- the existing core fidelity/contact/handoff gates on every prediction and
  fresh execution of the selected chunk. No feasible interaction means no
  committed prefix advance. Failed candidates can guide MPPI but cannot win;
- explicit `local_rl_required` failure with the last accepted state and prefix.

The serial diagnostic budget is six candidates and two iterations, configurable
at the driver. This is not the reproduction's 64-environment batched throughput.
The current core Cp/CR acceptance thresholds are unchanged. The object/contact
reward is adapted to the core; padding has no human trajectory fidelity gate.
Full padded Replay remains the cheap first attempt and final verification; only
its failure invokes local chunk switching. Mounted residual PPO remains to be
ported. Code checks do not establish physical success.
