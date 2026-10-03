# Mounted small residual PPO: r27 and bounded r28 follow-up

## Completed r27

Job `644448.hopper-m-02`, H200 hopper-26, code `1aca242`.
PPO actually trained in mounted PhysX; it was not a placeholder:

- Separate 64x64 ELU actor and critic, 24,677 total parameters.
- 18 bounded residual actions: wrist xyz, world-frame rotation vector, 12 fingers.
- 60 PPO updates; 253 episodes including deterministic evaluations; 202.77 s
  inside the solver. Four serial training episodes per update.
- Zero policy first fails core object fidelity at source frame 47.
- Best deterministic evaluation first fails at frame 48, return 20.4569 versus
  zero-policy 15.1515. No successful training episode or accepted window.
- A1 and full Oracle V0 are still **not successful**.

Machine-readable evidence: [r27/summary.json](r27/summary.json).
Full checkpoint, traces and training history: `outputs/oracle_a1_smallppo_r27/`.

The scheduler returned exit 0 even though `development_result.json` correctly
reported `local_refinement_failed`. Isaac/Kit shutdown bypassed the driver's
final failure exit. The single-Anchor launcher now checks the saved physical
result before reporting scheduler success. It does not change physical gates.

## r28 change and scope

Code `f087856`; job `644726.hopper-m-02`, run `oracle_a1_smallppo_r28`.
Same 64x64 model, 60-update / 480-second limit and 20-minute interactive request.

Previously, a terminal failed action received zero incremental reward because
the shared evaluator stopped before adding that frame's tracking reward.
This follow-up assigns a training-only negative terminal penalty: -1 for
other failures, or negative max(1, normalized object pose error) for core
fidelity failure. It records failure categories per update. A successful zero
policy immediately returns for physical validation instead of training.

This is an explicit reward-design change relative to strict alive-mask reward
zeroing in the earlier reproduction, not a claim that the paper requires a
terminal penalty. Whether it improves grasp learning is **unverified** until
r28 completes. No model enlargement, extra waiting, threshold relaxation or
sample-specific reward was added. Final padded/core and uninterrupted rollout
acceptance remain unchanged.
