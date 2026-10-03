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
