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

## Current mounted implementation (2026-10-03)

- Replay -> temporal MPPI -> residual PPO operates in 20-frame commits with
  a 40-frame feasibility horizon. All modes start from measured physical state.
- PPO observes robot/hand q and dq, wrist/object state and retargeted commands.
  It adds 18 residuals (xyz, rotation vector, 12 finger joints). The arm is
  controlled through continuous IK and fixed-source-clock mounted execution.
- Separate 64x64 actor/critic networks are the user's requested small model.
  PPO/GAE, learning rate, batch size, action scales and network sizes are
  reproduction choices; Appendix C does not specify these numeric settings.
- C.3/C.4 now use one scalar weighted error for both reward and termination.
  We retain the existing project's axial budgets: C=1, lambda_p=1/.05^2,
  lambda_R=1/.35^2. These are NOT claimed to be the paper's parameter values.
  The resulting ellipse is stricter at corners than the old separate box;
  this is a gate change and must be recorded when comparing runs.
- C.7 contact bonus requires a thumb AND another finger. Link identities come
  from the actual PhysX contact view row order; palm contact and two non-thumb
  links cannot earn this bonus. Contact evidence alone does not prove grasp
  retention or force closure; physical handoffs and retention still apply.
- PPO optimization uses tracking + contact; deterministic checkpoint ranking
  uses C.9 object-tracking return only, excluding invalid frames. The terminal
  tracking step uses the same reward formula, with no r28 bespoke failure
  penalty. Numerical/actuator failures before observation terminate with zero.
- TACO human-mimic, action-smoothness rewards and domain randomization stay off.
  No extra lifting reward is enabled; Appendix C.8 is optional by task.
- Core-only tracking, padded context without fidelity gates, physically acquired
  snapshots, mounted arm limits and final continuous execution are adaptations
  required by this project. No full EgoEngine robot reference is created.
- Discrete braking-distance projection avoids commanding a joint into a state
  with no feasible next command at its position limit. This is an execution
  fix, not a reward change or proof of closed-loop velocity feasibility.
  Actual PhysX velocity checks remain active. IK warm-starts from the preceding
  solution rather than reinitializing each policy action from the baseline.

## Validation boundary

M1/r10 is frozen. No successful A1 or full Oracle V0 has yet been established.
r27/r28 trained real mounted PPO but failed the first local interaction.
The previous any-two-link grasp signal and auxiliary-inclusive ranking were
incorrect relative to Appendix C.7/C.9. Historical reports describe those old
runs; they are not evidence of success under the corrected objective.

Only local padded/core validity, predecessor boundary reachability and final
uninterrupted execution are success gates. RELAXED has no tracking reward.
