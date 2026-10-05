# r30: mounted finger velocity feedforward

## Hypothesis and scope

r29 Replay source frame 46: thumb bend target 1.030 rad, measured 0.726 rad.
Its thumb rotation target was 1.322 rad, measured 1.127 rad. The command was
already close to the local retarget proposal, so missing thumb contact cannot
be attributed to retarget geometry alone. The drive used kp=5, kd=0.5 but
received zero finger velocity targets while the position targets were moving.

r30 sends each finger command's discrete derivative to the same drive, as is
already done for the arm. This is generic to local Replay, MPPI, PPO and core
execution. The RELAXED bridge interface, gains, retarget trajectory, source
clock, Appendix objective and physical acceptance are unchanged.

Non-rendering development traces now include arm/hand position and velocity
commands, so future tracking failures can be inspected without another run.
Two focused checks passed: feedforward command consistency and zero-residual
source-clock execution. This is not evidence of physical interaction success.

## Comparison

Use the identical r10 physically acquired frame-29 entry and oracle A1 proposal.
Compare Replay thumb tracking error, first thumb-and-finger contact, core
tracking failure, and final retention/handoff against r29. If Replay fails,
the unchanged Replay -> MPPI -> small PPO escalation runs within the same job.
No sample-specific grasp offset, extra waiting, or manual state patch is added.


## Completed result

Job 644875, code 40632d8, H200 Hopper-22, walltime 6 min 30 s.
PPO completed 60 updates, 253 episodes in 255.24 s; zero accepted windows.
Best evaluation failed tracking at frame 50; zero policy failed at frame 46.
Training failures: 170 tracking, 70 measured actuator velocity.

At frame 46, thumb bend tracking error fell from 0.304 to 0.099 rad with
identical position targets. This supports the drive-interface fix, but does
not establish a valid grasp. Opposition appeared only at recorded frames
64-66; core exit and retention still failed. Replay first failed tracking one
frame earlier than r29. A1 and full V0 remain unsuccessful.

Actual-state Replay video: outputs/oracle_a1_handff_r30/replay/mounted_xhand_bc_curobo_actual.mp4
Verified 53 frames, 30 fps; this is a failed padded local Replay, not PPO or
full 4.8 s V0 success.

## r31 hypothesis: distinguish core from padding during fitting

At source frame 40, measured wrist differs from raw local retarget by 72 mm
and 30.4 degrees, while arm command tracking differs by at most 0.00342 rad.
This indicates substantial deformation upstream of execution, not merely
closed-loop tracking lag. Existing fitting minimized error over padding and
core equally. r31 uses core weight 1 and context weight 0.05, with unchanged
joint/velocity/nominal-acceleration limits and actual source clock. This is a
generic fitting-priority choice to test, not a paper hyperparameter or proven
physical fix. Core validity still requires actual rollout. Save raw local
joint targets alongside fitted commands for attribution.
