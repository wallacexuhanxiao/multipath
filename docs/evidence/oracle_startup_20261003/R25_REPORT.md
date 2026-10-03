# r25: startup recovered, mounted MPPI exercised, zero-residual bug fixed

## Status

A1 has **not passed**. No complete 4.8-second Oracle V0 exists yet.
Job `644408.hopper-m-02`, interactive H200 / hopper-30, ran code `acf67f1`.
The zero-residual correction `afd3f4c` is pushed; its 20-minute regression job
`644419.hopper-m-02` (`oracle_a1_mppi_r26`) completed on hopper-20; its results are appended below.

## What was actually executed

- Node-local Torch Python resources loaded Torch 2.7.0+cu128; the CUDA probe
  identified NVIDIA H200 and completed its tensor operation.
- Torch import through SimulationApp readiness took 54.88 seconds. r24's
  import stalled reading Python files on the shared filesystem. This run
  confirms the staged path works, but changed nodes and concurrency mean it
  is not an isolated filesystem performance comparison.
- Padded A1 Replay completed frames 29–82 in exactly 53/30 = 1.766667 seconds,
  with no internal state reset. The trace contains 53 actual-state samples.
- First core fidelity failure: source frame 47, object error 0.007052 m and
  0.426682 rad. The core exits without grasp; handoff and post-context retention
  fail. Maximum actual arm speed is 0.960266 times its configured limit.
- New local chunk switching used 20-frame commits / 40-frame predictions.
  Thirteen trials including Replay were attempted, twelve entered PhysX,
  and zero were interaction-valid. Six stopped on actual arm velocity,
  five on core object fidelity, one on core entry contact, and one failed IK.
  No failed candidate was committed to the next chunk.

## The new actionable bug and repair

Replay's fitted controls entered the core and reached frame 47. But MPPI's
zero residual rebuilt controls from the raw Cartesian retarget, with a
different fitting horizon and startup context. It reached only frame 33 in
the recorded source-frame trace before an actual speed failure. Zero residual
was therefore not the baseline it purported to refine.

`afd3f4c` fixes the interface:

1. Zero residual executes the exact fitted Replay control array.
2. Nonzero residuals are centered on FK of the locally fitted Replay commands,
   including its finger commands, rather than the pre-fitting raw reference.
3. Candidates retain the remaining original pre-context for startup fitting.
4. Later chunk boundaries share one measured q/dq rebase across Replay/MPPI.

This is a reusable local-synthesis correction: no sequence ID, frame-specific
branch, threshold relaxation, controller change, or added source time.
It changes neither r10 nor the RELAXED planner. Its physical benefit is
reported in the r26 follow-up below, separately from the CPU regression.

## Verification and limits

Six focused command/selection checks passed before the run. After the
zero-residual correction, the two selection/clock cases passed again,
including exact zero-residual failure/return equivalence. These are mocked
selection regressions, not physical success tests.

Mounted residual PPO is still missing. r25 records `local_rl_required`, but
the discovered candidate-interface bug must be physically checked before
treating that result as evidence of MPPI interaction capability being exhausted.
No RL training was launched. Assembly self-collision validation remains at
the previously reported scope; actual acceleration remains diagnostic.

## Artifacts

- Compact machine-readable evidence: [r25/summary.json](r25/summary.json)
- Full local evidence: `outputs/oracle_a1_mppi_r25/`
- Actual-state Replay video: `outputs/oracle_a1_mppi_r25/replay/mounted_xhand_bc_curobo_actual.mp4`
- Detailed exported report: `outputs/oracle_a1_mppi_r25/RESULT.md`

The video is rendered from recorded PhysX mesh transforms at 30 fps; it is a
failed local Replay, not a successful Anchor or final uninterrupted V0.


## r26 physical follow-up — completed

Job `644419.hopper-m-02` ran `afd3f4c` on hopper-20 (H200). Initialization
completed in 40.07 seconds. All 13 predictions passed nominal checks and
entered physical execution; there were **zero** measured-speed failures and
zero IK failures. All 13 ultimately failed core object fidelity.

Replay and MPPI_ZERO now both first fail at frame 47. Their cumulative returns
are 15.150774 and 15.151374, and maximum measured arm-q difference is
0.000044584 rad across 18 recorded source-frame samples. Commands are identical
by construction; measured differences remain from simulator restore/contact
history. This confirms the corrected zero-residual interface at the tested entry.

The best MPPI prediction first fails at frame 50 (versus Replay's 47). This is
0.1 seconds farther inside the **same source schedule**, not added execution
time. It is still a failed candidate and was never committed or exported as a
valid Anchor. No candidate passes the two-chunk horizon.

The budget was six candidates over two iterations in serial mounted PhysX.
This does not prove that MPPI cannot solve the task with a different budget.
It does establish that this run reaches interaction failure without the prior
candidate startup/IK failures. The next unimplemented solver stage is mounted
residual PPO; it was not trained in this run.

Latest machine-readable evidence: [r26/summary.json](r26/summary.json).
Latest local actual-state video and detailed report are under
`outputs/oracle_a1_mppi_r26/replay/mounted_xhand_bc_curobo_actual.mp4` and
`outputs/oracle_a1_mppi_r26/RESULT.md`. The Replay video is 53 frames at 30 fps,
covering 1.766667 seconds; it is still a failed A1 development rollout.
