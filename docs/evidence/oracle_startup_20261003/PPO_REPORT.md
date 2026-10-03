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
terminal penalty. The completed r28 result below shows no successful grasp window;
the small frame improvement does not establish a robust learning benefit. No model enlargement, extra waiting, threshold relaxation or
sample-specific reward was added. Final padded/core and uninterrupted rollout
acceptance remain unchanged.


## Completed r28

Job finished on H200 hopper-25 in 5 min 43 s. Startup took 40.45 s;
PPO consumed 203.95 s for 60 updates and 253 episodes (240 training,
13 deterministic evaluations including the zero policy).

| Evidence | r27 | r28 |
| --- | --- | --- |
| Zero-policy first failed source frame | 47 | 47 |
| Best-return deterministic policy first failed frame | 48 | 49 |
| Successful training episodes | 0 / 240 | 0 / 240 |
| Accepted windows / validated A1 | 0 / no | 0 / no |

r28's best evaluation was update 20. The extra two frames relative to its
zero policy are only 0.067 s of source time; they are not Anchor success.
Returns across r27 and r28 are not directly comparable because the reward changed.
The final update regressed to an actual joint-velocity failure at frame 35.5,
so the last checkpoint should not be confused with the saved best checkpoint.

All 240 training episodes failed: 145 core object-fidelity failures,
79 actual joint-velocity failures, and 16 command-boundary infeasibilities.
This demonstrates that the small residual PPO really executes mounted physics,
but the current bounded training run has not learned an acceptable interaction.
It does not establish whether longer training, different residual parameterization,
or a better grasp initialization would solve it.

The scheduler now reports exit 1, consistent with the saved physical failure;
the job was not terminated by its time limit. No new run was launched after r28.

Machine-readable evidence: [r28/summary.json](r28/summary.json).
Full checkpoints and actual-state training traces:
`outputs/oracle_a1_smallppo_r28/ppo/chunk_0000_attempt_00/`.
No successful exported core, continuous 4.8 s trajectory, or PPO success video
exists from this run. M1 remains frozen and valid; M2 remains incomplete.

## Reusability and next useful investigation

The small actor/critic, failed-window PPO fallback, physical snapshot reset,
source-clock residual execution, terminal failure feedback, and scheduler
result check contain no sequence-name branches. This is reusable implementation,
not evidence of transfer to a second sequence. Terminal feedback was a generic
training change, but its effectiveness is not established by this run.

The next useful investigation is the recorded grasp/contact behavior near the
first core object-rotation failure, together with the high rate of velocity
failures during residual exploration. These are observed limitations; no new
thresholds, additional smoothing layers, or larger network were introduced.
