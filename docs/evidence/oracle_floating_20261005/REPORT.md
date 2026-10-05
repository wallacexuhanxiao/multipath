# Floating XHand Oracle V0 integration

## Scope

Following the user's embodiment change, Anchor synthesis and RELAXED transport
now use the same floating XHand. The physical arm and mounted IK are absent from
this backend. The old mounted backend and r10 evidence remain available.

- Local source proposals, oracle core/padding intervals and 30 Hz clock are reused.
- Wrist uses the prior reproduction's external-wrench PD gains, clamps, COM-origin
  velocity correction and total-hand gravity feedforward; fingers use physical drives.
- cuRobo sees six virtual planning joints (XYZ translation, ZYX orientation),
  actual hand geometry and, when carrying, measured carried-object geometry.
  Virtual joints are never used to set the simulator state.
- Fixed-time parameterization is reused. Physical state recording and core
  object/contact acceptance remain active. No full robot reference or VLM is used.
- Floating MPPI retains 20-frame chunks and 40-frame lookahead, Cartesian rotation/
  translation/finger residuals, followed by the existing 64x64 PPO if necessary.
- Artifacts and compatibility distinguish floating wrist pose/twist from arm joints.

The local development entry is `hpc/develop_floating_anchor.pbs`. It plans a
physical predecessor to synthesis-start, then tests padded synthesis and exported
core replay. This is not yet the final oracle executor: the final bridge must
reach the acquired canonical core entry, and all three Anchors must execute in
one uninterrupted 4.8-second rollout. No success is inferred from this development
prefix. New floating snapshots are acquired; mounted r10 snapshots are not reused.

## Integration evidence

- r35 / 647724: simulation and virtual cuRobo FK initialized; raw synthesis entry
  collided with the open-hand geometry. Rejected before motion.
- r36 / 647726: generic local vertical entry search found a collision-free target;
  fixed-time cuRobo path planned for 29/30 s. Measured collision-model rejection at
  frame 26: position error 1.84 mm, orientation error 0.145 rad.
- Planning now inflates hand geometry by the measured ready-state position error
  plus the rotational swept radius. This is a generic tracking allowance, not a
  source-specific offset or controller tuning. Measured collision checks use the
  underlying geometry, without double-counting the planning allowance.
- r37 / 647727 exposed the double-counted allowance in the first implementation;
  r38 / 647733 runs the corrected check and enables floating MPC/PPO fallback.

Four focused new checks cover virtual-coordinate roundtrip, spatial angular
feedforward, fixed source sample count / padding boundary, and measured collision
checks without double-counting tracking margin. Six existing control/MPC checks
also pass. These are interface checks, not physical success evidence.

## Validation boundaries

Floating-hand feasibility does not imply feasibility for a real mounted arm.
The planning hand shape is frozen at the measured bridge posture. Hand self
collision is disabled as in the retained simulation; stronger self-collision
validation is not claimed. Planning budgets are explicitly nominal, not hardware
ratings. Continuous full-V0 execution, later Anchor dependencies, and physical
carried-object transport remain to be validated.


## r38 and r39

r38 (647733) completed the entire 29-frame cuRobo predecessor with nominal
measured-geometry collision checks passing. End wrist error was 8.71 mm /
0.168 rad, and contact remained free. It then rejected an added nominal wrist
tracking threshold, rather than an actual canonical boundary violation.
That added gate has been removed: a development target is not a validated
canonical entry. Tracking error remains reported; actual collision, source clock,
contact/object interaction and canonical state compatibility remain checked.
No reference state is assigned. r39 (647735) runs from this actual state into
padded Replay, followed by floating MPPI/PPO as required.

The snapshot provenance now points to the actual local trace and source frame,
with the predecessor trace retained, rather than only naming the earlier bridge.

r39 has now printed `FLOATING_PREDECESSOR_PASSED` and entered actual floating
MPPI. First two 64-candidate iterations took 19.15 s / 45.38 s cumulative, with
no valid plan yet. A1 acceptance remains pending; the small PPO fallback is enabled.
Job is interactive H200, 20-minute allocation. No full V0 success is claimed.

The r38 predecessor actual-state video is exported locally as
`outputs/oracle_floating_a1_r38/predecessor/floating_xhand_actual.mp4`.
It contains 29 frames at 30 fps (0.9667 s), rendered from measured PhysX mesh
transforms. The final rendered frame was visually inspected. It shows the
approach only, not a successful grasp. Initial plus N measured boundaries are
rendered as N source-time intervals to avoid adding a frame of artificial waiting.
