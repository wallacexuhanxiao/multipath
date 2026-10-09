# Lift failure investigation — 2026-10-09

## Question and acceptance

The observed brush tool moved sideways but did not leave the table. The goal
remains a complete actual PhysX rollout: static oracle schedule, local padded
and exported-core Anchors, cuRobo only for RELAXED intervals, original source
duration, no robot/object resets between segments. Contact alone is not task
success. Existing diversity remains downstream of a successful V0 rollout.

## What the evidence establishes

Lifting reward was already enabled. The shared TACO objective is Appendix
C.3 + C.7 + C.8; C.8 uses measured tool height, with scale 10/m. Training
contact bonus is 2. Evaluation remains object tracking only. These numerical
scales are shared engineering choices, not claimed exact paper parameters.

The r203 diagnostic physically reexecuted the initial cuRobo predecessor and
r201 brush acquisition to frame 26. In two separate physical trials it then
executed either the original continuation or a vertical wrist command with
unchanged finger targets. Both fit the same frame-26-to-32 source budget.
Neither branch assigns an intermediate state.

| Run / branch | Actual wrist rise | Actual tool rise | End dual contact |
|---|---:|---:|---|
| r203 original | 10.44 mm | 1.75 mm | yes |
| r203 vertical, same grip | 20.28 mm | 4.10 mm | yes |
| r204 explicit material, original | 8.53 mm | 0.13 mm | yes |
| r204 explicit material, vertical | 17.46 mm | 0.67 mm | yes |

The tool weighs 1.4715 N in this simulation. The vertical trial's measured
hand-to-tool upward force is intermittent and commonly below that weight;
some samples apply downward force. This explains why touching and lateral
movement did not imply load-bearing grasp. The vertical command experiment
also rules out absence of an upward wrist command as the sole explanation.

Normal contact force is not the full support force. The diagnostic includes
the separately measured friction force via the [Isaac Sim contact API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.prims/docs/index.html).
The six frame-boundary samples are an attribution diagnostic, not a continuous
force-closure proof or a new acceptance gate.

## Repairs and their actual outcomes

1. **Robot grasp reference:** transporting the fitted local XHand grasp by
   source tool motion preserves its grasp-frame geometry after closure. This
   replaces a constant human-wrist correction whose grasp frame changed.
   r202 still failed all three first Anchors, so this is not presented as a
   demonstrated physical solution.
2. **Explicit hand/tool friction material:** imported XHand colliders and the
   tool had no physics material binding. Prior EgoEngine reproduction used
   static/dynamic friction 1.0. The shared configuration now binds that value
   to unbound hand/tool colliders, including URDF geometry under `/colliders`
   outside the articulation root; authored materials and table settings stay
   intact. r204 verified all 20 bindings. The same old commands still slipped,
   so friction alone did not solve the grasp.
3. **Gravity moment:** during vertical movement the wrist developed about
   7–8 degrees of rotation error. The actual hand is 1.122385 kg; measured
   compensation moment is about 0.27–0.31 N m. Translation-only weight
   compensation leaves this moment uncompensated. r206 applied the existing
   measured-moment controller variant, but the old commands lost dual contact.
   Changing the controller requires fresh local synthesis; the old learned
   grasp cannot be used as proof under the new controller.
4. **Accurate PPO artifacts:** `baseline.npz` now records the actual compiled
   zero-residual reference. A different input seed is kept separately as
   `proposal_baseline.npz`. This repairs diagnostic provenance, not physics.
5. **Bridge reuse after configuration changes:** after executing a new valid
   initial bridge, the end-to-end loop replaces a rejected older cache with
   that newly executed bridge. This avoids repeatedly replanning an already
   observed valid prefix while preserving fresh physical execution.
6. **Loaded finger-drive tracking:** the old stiffness/damping was 5/0.5;
   actual loaded finger displacement was 0.03–0.075 rad against a grasp fit
   with 0.5 mm geometric preload. The common configuration now uses 50/1.0,
   preserving the existing 3 N m effort ceiling. Initialization and all fresh
   initial-state trials apply the same configuration. r208 changed these
   gains relative to r207 and freshly synthesized the interactions; brush A0
   passed both padded playback and a separate exported-core playback from a
   physically reexecuted predecessor. No source/action-specific branch is used.

r205 ran the complete pipeline entry point for brush: its initial RELAXED
bridge physically passed; Replay/MPC/PPO and fresh local playback failed A0.
Fresh local playback reached the core exit with about 4.87 mm tool rise, but
carried-tool/table clearance remained -0.30 mm. No accepted Anchor or full
trajectory was exported. Elapsed time was 540.12 s.

## New physical result: brush A0 passes

r207 with explicit material and gravity moment but the old finger gains failed
all three first Anchors. r208 with the loaded finger gains passed brush A0:

| Measured quantity | Result |
|---|---:|
| Padded synthesis interval | [14, 38), 0.8 s |
| Exported core interval | [20, 32), 0.4 s |
| Padded rollout success | true |
| Fresh physically reached core replay success | true |
| Actual tool height gain over the core | 18.869 mm |
| Core-exit tool/table clearance | 15.111 mm |
| Core-exit contact | thumb and middle finger |
| State resets inside either validation rollout | 0 |

The core starts at a freshly executed initial cuRobo predecessor, not an
assigned robot/object pose. The final tool is above the table and remains in
opposition contact. This is evidence of a local load-bearing grasp. It does
not establish full task success or grasp transfer to the other objects.
The pour and hit first Anchors still failed in r208.

The largest recorded hand-speed ratio is 0.307 of the URDF velocity limits.
One compliant joint stop overshot by 0.000719 rad; this report does not claim
mathematically exact simulator joint bounds. No new torque gate was added.

## Remaining full-pipeline failure and repair

The brush end-to-end entry point executed its initial bridge and A0, then
rejected the carry bridge [32, 39), whose original budget is 7/30 = 0.233333 s.
The fixed-budget fit reported acceleration ratio 14.696; path-speed fitting
reported an incompatible moving boundary; the boundary-constrained fit
collided at tick 2. These are planning failures, not evidence that the newly
validated grasp replay failed.

A concrete timing mismatch was found: cuRobo optimized on its default
32-knot clock with 1/30 s between knots (1.033333 s), then the executor fitted
that path to the 0.233333 s source bridge. For moving boundaries, compression
changes the physical start/end derivatives. The planner now sets the carry
optimization step to `source_duration / (32 - 1)` before optimization, and
records that budget. The free-motion backend is unchanged. Final dynamics,
geometry checks and fresh continuous PhysX validation remain required.
The [cuRobo MotionGen configuration](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html)
documents fixed `trajopt_dt` for non-static starts.

This planner repair has passed local syntax checks but has not been executed
on Hopper. During result download, the local NUS VPN route disappeared and
`hopper.nus.edu.sg` stopped resolving. Consequently the final PBS status of
job 656706 and the repaired full rollout are unverified. The last observed
brush candidates 01 and 02 both stopped at this same bridge. A complete
trajectory has not been produced by this repair yet.

The validated A0 commands can warm-start the next end-to-end run; every
boundary and final rollout must still be physically reexecuted. Repeating
unconditioned grasp synthesis for the same planner rejection is not new
interaction evidence.

## Scope and artifacts

All changes depend on local Anchor data, object/hand geometry, actual measured
mass/COM/contact, or cache provenance. There is no action-name/source-ID branch,
no object mass reduction, no source-clock stretch and no relaxed human-path
fidelity gate. The shared default configuration now matches the exact loaded
finger/gravity/material variant physically used by r208. That is a reusable
mechanism with one demonstrated local success; multi-object transfer remains
unproven.

At this report revision, a new complete multi-type rollout has not passed.
The files in `evidence/oracle_action_types_20261008/` contain machine-readable
attribution results, failed full-pipeline reports, the original r208 A0 report
and `r208_lift_summary.json`. The artifact writer also now distinguishes
`core_rollout_complete` from the separately validated padded interval.

Local core video:

`/Users/apple/Desktop/semantic verifier/multipath/outputs/oracle_action_types_20261008/r208_brush/measured_core_video/floating_xhand_actual.mp4`

It contains 12 frames at 30 fps (0.4 s), rendered from recorded actual PhysX
wrist, hand joints and object poses. Exact URDF visual FK reconstructs the
hand meshes; an earlier same-scene recording supplies visual geometry and
static background only. The renderer runs no physics and uses no reference
motion. It is a local A0 core clip, not a complete task video.
