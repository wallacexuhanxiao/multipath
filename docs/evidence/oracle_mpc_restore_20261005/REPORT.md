# r32: restore previous reproduction's MPC action space and search budget

Job: 647395.hopper-m-02. Code: ead6f23 (rotation restored in 7e41ba4).
Run: oracle_a1_mpc_restore_r32. interactive, H200, 30-minute allocation.

## Question

Can mounted A1 pass with the prior reproduction's wrist rotation search and
64 candidates x 3 iterations, instead of translation-only 6 x 2 search?

## Changes and controls

- MPC residual is xyz + world-frame rotvec + 12 fingers (18 dimensions).
- Scales .02 m, .2 rad, .25 rad match the inspected previous MPC settings.
- 64 candidates x 3 iterations; sequential mounted PhysX, not batched floating hands.
- H=20, two-chunk lookahead, commit current chunk only.
- Preserve exact Replay at zero residual, physically acquired r10 entry,
  current retarget/fitting, finger feedforward, core objective, contact rule,
  source clock and physical limits.
- Existing small PPO remains integrated; this controlled run uses --no-ppo
  to spend the allocation on the enlarged MPC search and its physical replay.
- A successful candidate must still pass fresh padded and exported-core rollout.

This restores two omitted/diminished solver capabilities. It is not an exact
reproduction of the old experiment: embodiment, entry, tracking thresholds and
contact evidence remain different. No threshold was silently loosened.

## Verification

Two focused MPC selection/rotation cases passed; shell syntax and diff checks
passed. r32 was cancelled after 2m51s following the runtime-budget review. It had not
reached physical MPC candidates. This is a cancelled experiment, not an interaction result.


## Efficiency correction and r33 profiling

The local fit is a convex quadratic objective with linear constraints, but was
being solved by generic iterative SLSQP independently for 19 joints per candidate.
It now uses DAQP with the same objective, measured entry q/dq, joint bounds,
velocity, nominal acceleration, startup envelope and core/context weights.
This changes the numerical solver, not physical acceptance or source timing.

Local measurements:

- Actual r31 target window (53 source frames / 212 physics ticks, 19 DOF):
  DAQP 0.256 s; command feasibility passed. Old SLSQP hit its iteration limit
  at arm joint 5, so no valid full-window speedup ratio is reported.
- Short 40-tick arm comparison where both solvers succeed: SLSQP 0.146 s,
  DAQP 0.00442 s, maximum command difference 2.71e-16 rad (~33x faster).
- These are local numerical measurements, not Hopper end-to-end speedups.
- Six existing focused control/MPC checks passed.

Initial IK/fit and per-candidate IK/fit/physical-evaluation times are now recorded.
Next run r33 uses 6 candidates x 2 iterations, rotation enabled, PPO disabled,
interactive H200 with a 20-minute allocation. This is a timing/interaction probe;
full serial 64 x 3 is not restarted without measuring its cost.

A1 remains unvalidated. No additional temporal budget or state patch was added.
