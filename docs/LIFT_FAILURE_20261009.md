# Lift failure investigation — 2026-10-09

## Latest correction — 2026-10-11

Current bimanual evidence: A0 padded and exported-core replay pass, including
actual tool lifting, but a full 3.666667 s rollout does not. r263 executes the
whole first 0.233333 s bridge with both grasps retained, then misses the next
source boundary. r268 exposes a missing collision obstacle: the moving right
thumb intersects the supporting left palm at source 36–39. The two-hand
planner previously included the table and objects, but omitted the other hand.
The shared planner now includes the other hand's measured rigid-body geometry
and refreshes its poses for execution checks. r269 rejects a blind clearance
projection that cannot fit the original time; the replacement fits source pose,
endpoint velocity and collision together. r270/r271 find no valid entry from
the newer A0 exit, including seven optimizer initializations in r271. This is
search failure, not proof that a feasible entry does not exist. No invalid path
is executed or accepted. r272 tests the joint fit from the earlier r263 A0,
whose unmodified bridge had physically completed the seven-frame interval.

The suspected visual-only obstacle explanation was checked against both XHand
URDFs: no link without collision geometry was found, and the EE links explicitly
contain collision geometry. They remain in the physical obstacle model.

**Current result:** the first two-hand Anchor is physically valid; the complete
bimanual pipeline is still unverified. No source-time stretching, new grasp
threshold, intermediate state assignment, or deterministic-policy success
claim was introduced. Latest physical failure video remains r268 (1.3 s).

### Earlier single-hand investigation

**A complete new brush rollout has not passed.** r221's local core fit moves
Replay failure from frame 44 to frame 55, where grasp retention fails.
Local MPC/PPO and fresh Replay still fail. The lift reward is present, and
the first grasp has actual lifting evidence; this does not complete the task.

A more fundamental scene omission was found while the next job was queued.
In the source contact sheet, the left hand supports the plate. The earlier
EgoEngine implementation constructs both left and right hands; this pipeline
constructs only the right hand. r220's plate moves 140.17 mm by frame 39, while
the source plate moves 1.40 mm. Twenty-five native tool/target contact steps
occur during the nominally collision-free carry [32,39). The triangle checks
did not reject the native contact that pushed the free plate. Imported
dynamic collision decomposition and closed-loop pose/grasp deviations are
possible contributors; the present data does not uniquely attribute them.

This qualifies the older r218 result: its grasp/carry prefix passed the then
implemented gates, but it is **not proof of contact-free transport**. Its
actual-state video remains useful failure evidence, not a successful full task.

The shared right-hand scene now explicitly supports uncontrolled environment
roles with fixtures. `fixed_environment_roles` is a generic scene input; the
single-right-hand configuration declares `target`. The target is fixed only
at task initialization, with triangle collision matching the planner's mesh.
There is no target reference replay or intermediate state assignment. The
manipulated tool remains dynamic. This is an explicit substitute for the
omitted supporting hand, not a claim to reproduce bimanual contact dynamics.

Native tool/target contact now rejects RELAXED transport, including explicit
outgoing-bridge context in local Replay/MPC/PPO. Actual Anchor interactions
still allow the required contact. Export also preserves the actual Anchor's
carried-object feedforward flag; previously a string-only handoff field lost
that flag before the fresh final rollout. r222 reacquired a valid A0 under the fixture scene (both padded and
exported-core checks passed), but carry planning failed. r223 removed an
impossible extra collision margin on fixed physical boundaries; r224 solved
the nonlinear fit to a normalized source-boundary residual of -5.01e-7 but
rejected that numerical residual. r225 accepts only that solver-level finite
precision (1e-6 normalized residual); the physical C.4 gate is unchanged.
The resulting 0.233333 s command is nominally feasible, but actual native
tool/target contact rejects execution at frame 33. No complete task passed.

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
changes the physical start/end derivatives. The carry optimizer now uses
`min(32, physics_ticks + 1)` knots and `source_duration / (knots - 1)` per
step, with the physics step as the minimum trajectory step. The free-motion
backend is unchanged. Final dynamics,
geometry checks and fresh continuous PhysX validation remain required.
The [cuRobo MotionGen configuration](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html)
documents fixed `trajopt_dt` for non-static starts.

The VPN recovered and this repair has since been executed on Hopper. r209
exposed cuRobo's requirement that trajectory dt must not be below interpolation
dt; reducing the number of knots fixed that constructor error without adding
source time. r210 then reproduced the collision rejection below.

### Bridge attribution after reconnection

| Run | Observed outcome | Cause / reusable response |
|---|---|---|
| r210 | Nominal boundary fit rejected by collision spheres | At tick 2 the coarse carried-tool sphere overlaps the plate by 1.324 mm, while full triangle meshes have 5.246 mm separation. Refine conservative surface coverage against all mesh-safe fitted poses. Keep every surface and obstacle. |
| r211 | The 0.233333 s bridge executes with frozen fingers and retained dual contact; next source boundary fails | Actual tool error is 92.37 mm / 30.50 deg. The proposed boundary was already 38.04 mm / 15.23 deg away from the source: locking the human terminal wrist velocity consumed the reachable yaw envelope. This was not a successful handoff. |
| r212 | Source-conditioned pose and a robot terminal velocity are nominally reachable; fitted path has a real mesh collision | The oracle fixes pose/contact/time, not human wrist velocity. Jointly select one canonical pose/velocity inside the unchanged source-boundary ellipsoid and discrete dynamics envelope. The proposal error is about 2.66 mm / 1.98 deg, but that does not prove path feasibility or physical success. |
| r213 | Collision-aware source-time fit fails to converge | cuRobo's ESDF backward ignores the reduction's gradient unless `return_loss=True`. The distance gradient was being used incorrectly. |
| r214 | Correct gradient has unit translation norm; translation-only collision LP is infeasible | The carried tool intersects the plate at fitted ticks 4–8. Wrist orientation must be part of the collision/time fit, rather than being fixed while only position is changed. |
| r215–216 | All six coordinates are fitted, but the first hard collision linearization is infeasible; elastic iterations reduce penetration to 1.839 mm | Saved constraints independently show nominal dynamics feasible. Use local trust steps and collision penalties only during optimization, with zero final collision slack. |
| r217 | Jointly fitting the still-unvalidated next entry produces a collision-free, fixed-time command path; actual carry fails at frame 35.75 | Contact remains, but measured hand/tool rotation reaches 20.46 deg. Retain this as physical failure. A local repair misleadingly returns an unchanged core because its post-context modifies the bridge and checks retention only at source frames. |
| r218 | Repaired local grasp and actual carry bridge pass; A1 fails at frame 43 | Correct the local repair flow and then use actual bridge-boundary feedback. No gate is relaxed. A1's stable-grasp controller was dropping the carried-object dynamics feedforward used by the incoming bridge. |

The fitter differentiates the same cuRobo surface-distance field through all
six virtual wrist coordinates. It preserves the actual moving entry and the
velocity/acceleration budgets. A physically validated canonical exit stays
fixed; before the next Anchor has a validated entry, its proposed pose and
velocity can be fitted jointly inside the unchanged oracle boundary condition.
This chooses one canonical entry, not a Goal Set. Final collision checks and
real PhysX execution remain authoritative.

### New physical result: grasp followed by a carry bridge

r218 candidate 02 physically executes initial → cuRobo → A0 → cuRobo → A1
entry without resets. The bridge lasts exactly 0.233333 s and freezes the
finger commands. At frame 39 the measured tool boundary error is 19.577 mm /
16.542 deg, inside the same combined 5 cm / 30 deg oracle condition. Recorded
hand/tool relative rotation stays below 10.36 deg; the runtime checks retention
and actual mesh collision at every physics step. Wrist tracking error remains
21.444 mm / 14.118 deg and is reported separately from task-boundary acceptance.
This is a successful bridge, not a completed brush interaction or full task.

The local repair now preserves the already physically executed retarget core,
rather than rebuilding the failed raw acquisition. Replay, MPC and PPO use
the actual outgoing bridge commands as context; they no longer substitute a
stationary hold or a differently joined wrist path. Local context uses the
same per-physics-step retention check as runtime. The accepted r218 A0 differs
from the old reference by at most 1.016 mm in wrist position and 0.026665 rad in
finger drive. It has freshly validated padded/core artifacts. Source timing,
contact and tracking tolerances were not changed.

The r217 rotation is largely along the nominal two-point pinch axis (axis
alignment 0.889 at frame 35), despite 23–27 N normal contact. This suggests
rotational grasp compliance; nominal contact targets are not measured contact
positions and do not establish force closure. The attached figure plots the
actual recorded rotation, translation and normal forces.

![Actual carry failure and retained contact](evidence/oracle_action_types_20261008/r217_carry_physics.png)

### Next interaction's execution repair

At A1 entry, r218 has a real predecessor and an accepted source boundary. Its
Replay fails object fidelity at frame 43; MPC/PPO and fresh Replay also fail.
The bridge's measured rigid-grasp dynamics feedforward is switched off when
the stable-grasp Anchor starts, leaving only hand gravity compensation and a
short fading handoff bias. The new local synthesis path retains carried-object
dynamics compensation for stable-grasp Anchors. Replay, MPC, PPO, predecessor
reexecution and final artifact execution use the same recorded interface.
Previously validated artifacts retain their recorded control semantics.
This addresses an observed execution discontinuity, but does not solve A1:
r219 Replay reaches frame 45 instead of 43; fresh local refinement still fails
retention. r220 additionally prefers the next interaction's entry velocity
when selecting the unvalidated canonical boundary. Its carry reaches a valid
A1 entry, but A1 fails object fidelity at frame 44.

r220 keeps dual contact throughout frames 39–44. Tool position error grows
from 20.43 mm to 50.37 mm while wrist target tracking error decreases toward
failure. The command retains positive lateral velocity as the source reverses.
The object-relative handoff uses one Hermite duration for both position and
rotation; the slower rotation offset extends the position/velocity correction
to about 0.34 s. This is a specific command-handoff problem, not evidence of a
missing lift reward or PPO model capacity.

r221 replaces that stable-grasp Replay proposal with a fit of the **local core**
source object poses through the measured hand/object transform. Measured wrist
pose and velocity are the initial boundary; the existing wrench takeover
preserves the prior controller response and finger preload. The fit uses the
unchanged 1.5 s core budget and shared nominal velocity/acceleration bounds.
An offline rigid-grasp prediction satisfies the source condition, but is only
an initialization check; it does not establish physical interaction success.
Fresh PhysX Replay, local refinement when necessary, and uninterrupted runtime
remain the acceptance tests.

Two observed redundant loops were removed: a pre-command planning failure
does not rerun unchanged successful grasp synthesis, and carry-context
resynthesis that returns identical core commands and actual exit reports no
progress. The valid local Anchor remains available for the next experiment.
These guards prevent wasted allocations; they do not relax physical gates.

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
The brush initial approach and A0 are physically validated. The carry
has not passed the newly added native contact check; A1 remains unvalidated. Completed single-source jobs reported about 6 GB resident memory.
r221 revealed that the submitted 4 CPU / 75 GB override still resulted in a
12 CPU / 225 GB scheduler request. Correcting the queued resource request
produced a verified 4 CPU / 75 GB / H200 / 20 min request; the PBS defaults
now use these values too. A new submission still expands to 12 CPU / 225 GB
on the server, so the expansion is not solely a script-default issue: verify
the scheduler's actual resource list and correct the queued request. No
running job was cancelled or migrated.
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


## Bimanual extension requested — 2026-10-10

The source left hand supports the plate. The new optional two-hand backend
imports the existing left SPIDER XHand, applies the same wrench/finger
controller as the right hand, and steps both on one PhysX source clock.
The target is dynamic in `floating_xhand_bimanual.json`; no fixture, source
object animation or intermediate object assignment is used. Native measured
left wrist/q/dq/contact/drive state is included in every actual trace record.

`prepare_supporting_anchors.py` adds only the left padded local retargets to
the existing right proposals. The hand/object assignment and contact modes
are static oracle inputs. Finger kinematics, surface grasp fitting and local
contact linearization now accept either real XHand side. The algorithm has
no action-name or sequence-name branch. Numeric controller/reward settings
remain shared. The brush bimanual oracle is a manually reviewed data input,
not a task-specific controller patch.

The first joint experiment uses the passed right A0 commands only as a seed.
It physically executes a left cuRobo approach plus local padding, alongside
the right actual predecessor, then requires two checks: joint padded replay
and separate exported-core replay from a physically reexecuted entry. Each
trial initializes once before time zero and uses no segment-boundary reset.
This experiment does not establish the complete bimanual task.

r226 completed all three local left retargets, then exposed a startup bug:
importing the second robot after the first PhysX reset invalidated existing
articulation handles (`NoneType.link_names`). Both hands now import before
the first reset. r227 then physically executed both hands through frame 32.
Right A0 passed; left had about 1.50 N index contact, but no thumb opposition,
so the joint padded Anchor failed its unchanged exit-contact gate. The target
drift was 0.213 mm at that point. This was not a successful dual grasp.

r228 reused the existing floating MPC/PPO solver through a second-hand view
of the same shared scene. Every candidate executes the right controller too,
while optimization acts on the left wrist/fingers. MPC did not solve it. A
24,293-parameter residual PPO model found accepted sampled controls at update
20. Fresh initial-state execution passed both hands over the complete padded
window [14,38), with no reset within the rollout. Exported-core replay failed
left contact at frame 32; this result is reported separately, not as a passed
bimanual Anchor.

Two export-interface errors were identified: the acquired core entry reused
the old baseline's wrist takeover profile after PPO changed the trajectory;
and CORE playback generated a new takeover instead of slicing the successful
padded controller program. The profile is now compiled from the selected
commands and the measured entry, and CORE preserves the source-indexed
commands plus wrench inputs. `joint_commands.npz` now includes these control
inputs. This is a reusable command/export fix, with no pose assignment or
source-time change. r229 reuses the learned commands for a direct fresh
padded/core physical verification. It again passed padded Replay but failed
left core contact at frame 32. Its log lacks `core_controller_program`: the
job loaded the preceding exporter before the atomic script update completed.
A measured comparison showed exactly identical left entry state/targets at
frame 20; the newly generated core takeover then changed physical motion.
r230 (PBS 657071) verified the already-uploaded program-slicing repair,
without more PPO. Both hard gates passed: fresh joint padded execution
[14,38) and exported-core execution [20,32) from a physically reached entry.
At core exit, both hands had measured thumb AND non-thumb opposition
contact. The padded run retained both contacts through frame 38. The dynamic
plate was not held by a fixture. Each validation trial executed from the task
initial state with zero state resets inside that trial.

The core entry is at 0.666667 s, its exit at 1.066667 s, and the padded run
ends at 1.266667 s. The core still occupies its original 12 / 30 = 0.4 s
source budget. Actual eraser height increased 5.188 mm at core exit and
16.457 mm at padded exit relative to the initial state. Plate translation was
0.885 mm at core exit and 9.165 mm at padded exit; this is measured motion,
not evidence that the plate is perfectly fixed or that later interactions
will succeed.

The PPO search used the same shared local reward/controller parameters,
a 24,293-parameter model and 211.303 s of training. r230 is an independent
fresh physics validation of its compiled controls, not another training run.
The export repair preserves controller inputs across padding removal. It
is a reusable fix to the control-program interface; physical evidence is
currently limited to this joint Anchor, and transfer remains unverified.

Machine-readable evidence: `evidence/oracle_action_types_20261008/r230_bimanual_result.json`.
Local actual-state video (38 frames, 30 fps, 1.266667 s):

`/Users/apple/Desktop/semantic verifier/multipath/outputs/oracle_action_types_20261008/r230_bimanual/padded/floating_xhand_actual.mp4`

Red is the right XHand/eraser; green is the left XHand/plate. Video geometry
comes from recorded actual PhysX mesh transforms. This is the first local
bimanual Anchor, not the complete 110-frame task. Its core entry was acquired
with physical synthesis context; replacing that context with a final RELAXED
bridge and connecting the remaining Anchors still requires validation.
The final report explicitly retains `full_pipeline_success=false` and
`task_success=false`.

The r225 failure diagnostic samples convex-hull tool vertices against the
actual target triangle mesh. At frame 33, planned minimum sampled distance
is 2.422 mm and actual minimum is 0.412 mm. Wrist target error is 2.741 mm;
relative grasp changes 0.877 mm / 2.263 degrees from frame 32. Opposition
contact remains. These unsigned samples do not prove absence of penetration,
but demonstrate why nominal geometric clearance alone is insufficient;
native contact remains a hard transport gate.

## Joint outgoing bridge and load-conditioned synthesis — 2026-10-10

This extension uses the same 110-frame Brush source (3.666667 s). The two
hands share native PhysX and the source clock. Full bimanual task success
remains false. A passed local grasp is not yet a transport-compatible grasp.

| Experiment | What actually passed | Remaining physical failure |
| --- | --- | --- |
| r231 | Same cached local command proposal | Regenerating the left approach changed the actual contact; left core failed |
| r232–233 | Frozen actual predecessor; fresh joint padded/core Replay | Right planning collision field rejected the bridge; later mesh-ownership diagnosis invalidates a right-only attribution |
| r234 | Collision-feedback exit resynthesis; fresh joint padded/core; right bridge planning | Planner wrongly treated table-supported plate contact as an airborne payload collision |
| r235 | Both bridge plans within the unchanged 0.233333 s | Actual left opposition lost at source frame 32.5 |
| r236 | Both bridge plans, with support-aware nominal feedforward | Actual left opposition still lost at 32.5 |
| r237 | Baseline Replay reproduced the grasp and its outgoing-load failure | Failed MPC initialization damaged the previously successful core; PPO failed before the bridge |
| r238 | Zero residual reproduced the successful core and reached the original load failure | Bounded residual PPO still did not produce a valid loaded padded program |

The right collision fit's rejected clearance was -10.978234 mm, 0.05 s
after core exit. The reusable exit proposal derives its direction and size
from that collision field, then changes commands only after the physically
observed grasp. Its correction was approximately +11.071 mm vertically.
Fresh physics increased actual eraser/table clearance at core exit from
1.532 to 10.683 mm. Right planning then passed with velocity ratio 0.982034
and acceleration ratio approximately 1.0. The previously omitted goal
projection was also restored, but r233 proved that projection alone did
not fix this failure. No source time or acceptance threshold changed.

The static oracle now explicitly declares target/table support. Only that
object/environment pair is filtered in planning; all hand/table queries,
the full carried plate against other geometry, and native PhysX collisions
remain. Surface-supported nominal dynamics exclude the plate's full airborne
gravity/COM load. r236 demonstrates that this model correction alone is
insufficient to preserve grasp.

At r236 core exit the left wrist/plate angular-velocity difference was
1.339880 rad/s. Measured thumb force was only 0.248874 N, whereas index
contacts included 1.854727 N and 3.729379 N. The failed bridge changed the
relative pose by just 0.667 mm / 0.008241 rad before thumb contact vanished.
These were acquisition diagnostics, not a proven failure cause. r241 below
demonstrates that the earlier 32.5 rejection also occurs in a previously
accepted stopping program, because the two paths used different contact
sampling frequencies. They do not establish a torque-limit failure.

Outgoing local synthesis now uses the actual planned bridge in post-padding,
with contact checked at each physics step. Export still preserves the selected
commands and their source-indexed controller program. Every selected program
must pass fresh joint padded execution and exported-core execution from a
physically reacquired entry. PPO starts from the physically executed Replay
core after a rejected MPC seed, rather than silently replacing its grasp with
the failed seed. r238's zero policy reproduced the original failure at 32.5;
r237's zero policy had instead failed its core at 32.0.

The next bounded MPC search preserves this wrist program and tries closing
residuals on the measured loaded fingers. The shared MPC finger search scale
sets the proposal radius; joint limits and slew remain unchanged. There is no
source/action-specific numeric override. Acceptance is still actual joint
outgoing-load Replay, followed by exported core and continuous execution.

Reports r231–r238 are saved under `evidence/oracle_action_types_20261008/`.
The joint continuation backend already executes both cuRobo bridges on the
same world and has no inter-segment reset. Its current candidates explicitly
retain `padded_artifacts_complete=false` and
`initial_preparation_uses_synthesis_context=true`. Later padded Anchors and
the final pure RELAXED initial bridge remain unvalidated; this is not a
completed bimanual V0 pipeline. Transfer of these generic fixes to another
source has not yet been physically demonstrated.

### r239–r241: the control/physics sampling mismatch changes the attribution

Positive finger-flexion proposals and proposals from an actual-exit surface
Jacobian both failed fresh core validation. Native finger forces remained
recorded; these proposals are not accepted fixes. The surface Jacobian
explains why blindly increasing flexion is unsafe for a fitted thin rim:
the actual thumb gap derivative for rotation joint 1 was +0.039627 m/rad.
Increasing that joint moved the modeled skin away from the plate. The
geometry model is only a proposal model and cannot substitute for PhysX.

r241 then replayed the already valid stopping context with native contact
recorded at 120 Hz, and compared it with the actual bridge from the identical
core exit. **Both programs lost the thumb/non-thumb opposition signal at
32.5.** In the stopping context it returned at 32.75 (8.333 ms later), and
was present at every subsequent physics step through 38. Native index contact
remained during the gap. The previous 30 Hz padded/core checks accepted this
stopping program, whereas the added 120 Hz bridge opposition check rejected
the same transient. Therefore the earlier bridge failure at 32.5 was not
sufficient evidence that MPC/PPO must repair the acquisition. The weak force
and relative angular velocity were diagnostics, not a proven failure cause.

The reusable correction explicitly aligns the contact gate with the existing
source control clock: hand/object relative pose and any native finger contact
are required every physics step; full thumb AND non-thumb opposition is
required at each source control boundary. Complete loss of finger contact
still fails immediately. Pose tolerances, native force threshold, collision
checks, and source timing are unchanged. Raw instantaneous opposition is never
rewritten; its one-step absences remain in `executed_controls.npz`. Reports
record both validation frequencies. r242 performs the fresh uninterrupted
joint bridge validation with this corrected contract; it is not accepted on
the basis of the sampling diagnosis alone.

PPO can now resume its small model, optimizer and random-generator state for
an identical local residual reference. A fresh zero-policy physical check
still runs first, and checkpoint use does not reuse previous acceptance.
The actual r238 checkpoint restored 27 updates and 24,293 parameters in a
local interface check. This prevents repeated, identical seed-zero training;
it does not establish interaction success. The architecture and shared
Appendix C.3/C.7/C.8 reward are unchanged.

### r242: the next failure is native object/object collision

Fresh execution with the corrected sampling contract passed the earlier
32.5 substep, then failed at 32.75 on native tool/plate contact inside
RELAXED. This remains a hard failure. The measured impulse was 0.119112 N s.
The full task and bridge are still unsuccessful; changing the sampling
contract did not remove the collision gate.

Read-only inspection of the actual USD assets confirmed that both dynamic
objects author `physics:approximation=convexDecomposition`. The planner had
used their original triangle meshes. At the failure, the minimum sampled
eraser convex-hull vertex/plate-triangle distance was 3.200 mm; this unsigned
vertex sample does not prove separation of all collision shapes, but it is
consistent with a visual/cooked-collision-model mismatch.

Planning now reads each actual object's authored approximation. Dynamic
convex objects use a conservative whole convex envelope for cuRobo, carried
geometry and exact collision confirmation. Kinematic triangle surfaces retain
their triangle geometry. The original retarget and rendering meshes remain.
The envelope is not claimed to be the exact native cooked shape, and native
rollout remains acceptance. `object_collision_model.json` records this scope.

The surface-supported hand-only feedforward also omitted the measured contact
reaction needed to preserve wrist preload after its takeover bias faded.
The new support-aware feedforward proposal counters native hand/object forces,
using link origins to approximate their moment about the driven root COM.
It changes control inputs only. This approximation must pass continuous
PhysX; it is not an independent force-closure proof. r243 verifies both
collision-envelope and preload proposals together, so any success is not
attributed to either change separately.

### r243–r244: correct articulation ownership before interpreting geometry

r243's right-hand collision report named a **left-hand** index mesh. The left
URDF importer had placed the second articulation underneath the right USD
root. Prefix-based traversal therefore included that left mesh in the right
wrist's rigid planning assembly. Native contact/body views were already
separate; the error was in the planner's geometry inventory.

Planning now selects meshes belonging to the owning hand's actual rigid-body
links. r244 recorded 19 own meshes per hand and excluded the peer articulation
from the right assembly. Both complete hands remain in PhysX. This is a
generic articulation-ownership correction; it does not depend on an action,
source ID, object name or frame number. Peer-hand dynamic planning geometry is
not yet separately implemented, so this is not a complete bimanual safety
claim.

This finding corrects the earlier r233 attribution: its rejected collision
field was generated by a contaminated right planning model. The +11.071 mm
exit proposal did pass fresh joint padded/core physics and increased measured
eraser/table clearance, but those facts do **not** prove that insufficient
right-hand lift alone caused the earlier planning rejection. The offset is a
physically checked command initialization, not evidence from a clean model.

With ownership corrected, r244 still failed at source 32.75 on native
eraser/plate contact. Both fixed-time planners passed; uninterrupted execution
had no inter-segment reset. At this failure right wrist position tracking
error was approximately 2.5 mm in norm and rotation error 0.01350 rad. Left
plate prediction differed from its actual position by approximately 6.0 mm.
These are measured discrepancies, not evidence of divergent control.

### r245–r246: include the actual native contact band

r245 added native shape-offset and contact-point measurements without changing
the commands or acceptance gate. Each object's actual native contact offset
was 0.001362500247 m, including all 16 plate shapes. Rest offsets were zero.
The pair therefore generates contact inside a 2.725000493 mm band, although
visual/source triangles need not overlap.

At source 32.75 the loaded native contact point had 13.39566 N normal force
and -0.169024 mm solver separation. The other two generated points were
unloaded with positive separation. A read-only FCL check of the planned tool
convex envelope against the entry plate gave only 1.93069 mm separation at
that tick. This proves that the plan omitted the native contact band. Contact
solver separations and post-step visual distances are different observations;
their discrepancy is not a proof of exact native cooked-geometry equivalence.

For dynamic convex objects planning now reads the real native shape offsets
and expands each convex support plane by that object's maximum contact offset.
This retains the complete original convex hull and accounts for both objects'
bands when their envelopes are checked together. cuRobo, cover refinement and
carried-object checks consume the same envelope, so refinement cannot erase
the margin. Retargeting, rendering, native physics and hard native-contact
acceptance are unchanged. r246 performs fresh physical validation of this
repair. Kinematic triangle worlds retain their existing representation; this
change does not claim complete native-contact-band coverage for those worlds.

r246's nominal bridge passed the expanded collision model, but actual execution
still contacted the plate at source 32.75: normal force fell from 13.39566 N
to 8.56421 N, with loaded solver separation +0.647426 mm. Thus the contact-band
correction is necessary model repair but did not establish bridge success.
Right wrist position error remained approximately 1.85 mm. Closed-loop error
and peer-object motion still need physical feasibility checks.

r247 uses the clean r246 native feedback to propose a new Anchor exit. Its
direction is the loaded native contact normal; its size combines the contact
band deficit with the observed geometry closing error of the tool and peer
object. The same previously executed core commands receive a smooth spatial
offset only after physically observed acquisition, inside the original core
duration. No simulator state is assigned, no waiting is introduced, and
source-conditioned padded/core physics must pass again. This replaces feedback
from the earlier contaminated collision inventory with actual native contact
feedback. It is a proposal until fresh execution passes.

Local measured-state video of r244 through the passed A0 core:
`outputs/oracle_action_types_20261008/r244_bimanual/exported_core/floating_xhand_actual.mp4`.
It has 32 frames at 30 fps (1.066667 s); the failed bridge stopped at
1.091667 s, before the next full source frame could be recorded. It is not a
full-task or successful-bridge video.

Contact sampling attribution figure:
`evidence/oracle_action_types_20261008/r241_contact_sampling.png`.

### Round stopped after r247 (2026-10-10)

r247 (Hopper job 657393) passed fresh bimanual A0 padded rollout and exported
core replay. Its contact-feedback exit proposal used a 5.71 mm smooth offset
after physical grasp acquisition, within the unchanged source-time budget.
Continuous execution reached source frame 33.0 (1.1 s) with zero measured
eraser/plate contact steps and zero contact impulse in this executed prefix.
This removes the earlier frame-32.75 collision in the observed prefix; it does
not establish success for the entire [32,39] bridge.

The next actual failure was left-hand thumb contact loss at the source control
boundary 33.0. Non-thumb contact remained (index force 1.56446 N); hand/object
relative error was 1.451643 mm and 0.017777 rad. Right-hand thumb/non-thumb
opposition remained valid. The retention gate rejected execution, with zero
state resets between segments. No acceptance threshold was relaxed.

Status: A0 locally valid; first complete bridge and full bimanual pipeline not
yet valid. A1/A2 padded artifacts are still missing. The local video linked
above is the earlier r244 A0 actual-state recording, not an r247 or full-task
video.

Prepared, not physically validated or submitted: reuse the r247 right-core
commands only as a proposal requiring fresh joint acceptance, then repair the
left outgoing context with contact-band-sized MPC candidates and small
residual PPO if needed. The user requested stopping this round; no next
experiment was submitted. Current changes are saved locally and are unpushed.

### Resumed: r248 shared-world contact interface (2026-10-10)

r247 raw records clarify the failure: between frames 32 and 33 the supporting
wrist rose 1.900 mm while the plate height changed less than 0.002 mm. Thumb
contact-force magnitude fell from 0.24887 N to zero. The measured exit grasp
was marginal under outgoing motion, even though stopping padding had passed.
This is evidence of lost opposition under load, not evidence that adding a
lifting reward alone would solve it.

r248 (658064) froze only the previously executed right-core command seed and
used the planned [32,39] bridge as outgoing synthesis context. Both hands were
executed afresh. Replay reproduced frame-33 failure. MPPI and eight geometric
preload candidates failed fresh padded physics. Native contact-band sizing
produced radii 0.05614 rad (thumb) and 0.03906 rad (thumb plus opposing finger);
these are derived from measured geometry, not source-name configuration.

Small residual PPO started with 24,293 parameters. Its zero-residual rollout
reproduced frame-33 retention failure. A nonzero rollout then encountered
native tool/target collision and exposed an interface bug: the supporting-hand
view has no `tool_target_contacts` sensor. Pair sensors belong to the shared
PhysX scene. The exception aborted training; r248 is not PPO convergence or
interaction-success evidence.

`check_relaxed_tool_target_contact` now resolves the shared physical owner for
both sensor queries and native failure records. Collision remains a hard
failure; it now reaches the optimizer as physical feedback instead of an
AttributeError. A focused check exercised both direct-world and supporting-hand
views, confirmed RuntimeError rejection, and preserved the shared force record.
r249 (658070) reruns the same experiment with this interface repair.

Fresh local r247 measured-state video:
`outputs/oracle_action_types_20261008/r247_bimanual/exported_core/floating_xhand_actual.mp4`.
Verified 33 decoded frames at 30 fps, duration 1.1 s. It includes the failed
bridge prefix, not a complete task.

Hopper submission again expanded the request to 12 CPUs/225 GB despite an
explicit 4 CPU/40 GB select. The queued request was corrected to 4 CPU/40 GB,
one H200, 20 minutes; running jobs were not migrated.

### r249–r252: separate optimizer failures from the support contract

r249 (658070) completed 23 PPO updates in its 240 s budget after the shared
sensor-interface repair. No candidate passed fresh physical acceptance. r250
(658079) started from the original geometric retarget proposal instead of the
cached left core. Acquisition now latches its hand/object transform only when
measured thumb and non-thumb force vectors contain an opposing pair; this is
a controller decision, not a new reward or a force-closure success claim.
The r247 same-sided contact did not satisfy that decision. r250 completed
22 updates and still failed at core exit or outgoing retention.

The changed actual core exit exposed another execution bug: the outgoing
context retained an absolute wrist origin from the old rollout. Its first
position command was approximately 13 mm above the new core's command. r251
(658083) registers this local synthesis context to the current measured exit
pose and velocity. Unchanged exits preserve the old context exactly; changed
exits no longer reuse the previous absolute origin. This context is only a
development proposal. Final bridges are replanned by cuRobo from actual state.
The repaired run still lost left thumb contact at source 33; 24 PPO updates
did not pass. It was not a successful grasp/bridge repair.

r252 (658097) resumed the same 24,293-parameter PPO with model, optimizer and
RNG state restored: its first update was 25. Another 50 updates within 600 s
gave 74 cumulative updates and zero successful physical candidates. The best
training progress reached source 34 but was rejected by native contact or
retention. Longer training alone has not solved this interaction.

Reviewing the original RGB source changes the interpretation of the left-hand
failure. Both hands start free; the left subsequently stabilizes the plate on
the table. The oracle already declares `object_support.target=table` and permits
plate/table contact. Previously that constraint affected payload feedforward
but did not constrain bridge boundary targets. The resulting wrist boundary
demanded approximately 9 mm of upward motion, and small vertical source changes
also enabled the auxiliary lifting reward for this supported plate. At source
39 the original plate center is 0.387128 m versus the actual A0 exit's
0.382350 m. With source rotation retained, measured-clearance projection moves
that command boundary down by 5.567 mm. Original source arrays are unchanged.

The repair projects command-side object proposals onto the declared table
plane while preserving the actual measured support clearance, source-parallel
translation and source rotation. Original source poses remain the core and
boundary acceptance targets. A projection outside the existing tracking
condition is rejected; no simulator state is assigned. The same support
declaration disables the auxiliary lifting objective for a table-supported
object. This does not disable actual contact/retention gates or the lifting
objective for an unsupported carried object.

r253 (658116) uses the previously passed right and left A0 command seeds,
requiring fresh joint padded/core acceptance, then tests the support-constrained
bridge directly. No carry-context PPO is required for this comparison. The
support correction is implemented but physical success is not yet established.

r253 completed fresh joint A0 acceptance but again failed at source 33 with
exactly the r247 measured relative errors. The support projection was consumed
and changed subsequent commands. Its first four bridge commands and wrenches,
however, were byte-identical to r247's; this is a startup physical failure
before the changed boundary proposal can affect execution. The support fix
does not by itself repair bridge handoff.

r254 replaces the supporting contact's link-origin normal-force moment with
the actual PhysX contact-point moment. Any remaining per-link force retains
its link-origin approximation and is reported explicitly. This repairs a
known approximation in the coupled control model, without gain changes or
extra time. It still requires actual fresh bridge acceptance. r254 is a
controller attribution run, not a claim that the failure is already fixed.

r254 (658124) measured a native support-contact y moment of -0.661396 N m,
versus -0.602269 N m from link origins. The remaining per-link force after
native-point reconstruction was below 2e-8 N. Thus the measured-point repair
is consuming the correct forces and changes the missing lever arm by
approximately 0.05913 N m. It still failed at source 33: relative error
1.466243 mm and 0.017795 rad, zero inter-segment resets. It did not establish
successful handoff and is not the main explanation of the observed upshift.

r255 records only the first two source frames of handoff force balance,
including all native contacts, filtered object contacts, applied root wrench
and measured assembly COM velocity. These are diagnostics, with no new
acceptance gate. Their purpose is to distinguish an unmodeled environment
reaction from a controller/trajectory transient before another repair.

r255 (658137) reproduced r254 exactly. All measured contact forces were the
filtered plate forces; no additional environment-contact normal force was
present in these samples. The assembly COM was already moving upward at
0.02590 m/s at A0 exit, although its wrist origin was moving downward.
By source 33 COM velocity was 0.05076 m/s. This rules out the proposed extra
table-contact normal force as the explanation in this observed prefix.

The stronger finding is the incompatible rigid-carry boundary. A0's actual
plate vertical velocity was -0.003026 m/s, while freezing the current hand and
applying the measured wrist twist predicts +0.218941 m/s for the plate. The
first four planned rigid-tool heights rise by 1.828, 3.777, 5.843 and 8.023 mm.
The left hand therefore starts the bridge with a supported press that does
not satisfy the planner's rigid attachment assumption. Small relative pose
error and binary thumb/non-thumb contact at one boundary did not establish a
rigid carry state. Force/torque corrections alone did not resolve that mismatch.

r256 (658145) starts again from the original local geometric left proposal.
The acquisition controller now consumes the same declared table support:
command-side local object poses preserve measured support clearance, and
thumb/non-thumb pressure can latch a table stabilizer. Unsupported airborne
acquisition still requires opposing contact forces before latching. Original
object-tracking, contact, source-time and fresh padded/core gates are unchanged.
The existing local Replay/MPC/PPO escalation operates on this corrected local
proposal; if A0 passes, the next cuRobo bridge is replanned from its new actual
exit. This is a generic support-mode correction, not a source-name exception.

r256's first Replay reached source 30 with native thumb/non-thumb contact,
then an acquisition-target time-fit exception escaped directly to the top
level. This exposed a flow bug: controller fitting failure was not sent to
MPC/PPO. The first Replay now records that physical failure and escalates;
unexecuted command proposals are not presented as successful evidence.

For a table stabilizer the local command reference can be registered once to
the physically acquired contact state, retaining subsequent source-relative
motion and measured table clearance. This prevents a late exact-world-position
catch-up from replacing a valid supported boundary. Original source-conditioned
physical acceptance remains unchanged. The fixed PPO retarget reference also
consumes the declared support; no contact-dependent wrist rewrite was added
inside PPO.

r257 (658151) passed a fresh joint padded replay to source 38 and exported core
replay to source 32. Its small residual PPO accepted sampled controls after
14 updates / 56 episodes, with lifting disabled for the supported object. It
is evidence for reusable local command synthesis, not a claim of a successful
deterministic policy. The subsequent left rigid-carry plan was rejected before
execution: collision/time fitting could not satisfy its frozen-hand geometry.
No complete bridge was accepted.

The initial online Replay also exposed stale stopping padding: acquisition
changed the core commands but the following context still used the original
retarget endpoint. Context is now recompiled from the actually executed core
endpoint. If a core has already passed, PPO uses that real compiled reference
instead of relearning the failed MPC initialization.

r258 (658193) retains r257's physically passed local A0 seed and requires fresh
joint acceptance again. The supported hand enforces a persistent contact
constraint by continuing its actual drive preload when the actual object is
already inside the next source boundary condition; the moving hand gets the
cuRobo bridge. During a stable support Anchor the same hold proposal is allowed
only if it satisfies every original core reference condition, and actual
native contact plus source conditions are checked throughout. This is a
support constraint controller, not a second cuRobo transport path or an
automatically discovered Anchor window. Unsupported or moving payload planning
continues to use the existing cuRobo path. No source clock or object state is
changed, and no success is inferred from the hold proposal alone.

The earlier r255 video is a failed 1.1 s prefix:
`outputs/oracle_action_types_20261008/r255_bimanual/exported_core/floating_xhand_actual.mp4`.
All 33 frames decode at 30 fps. The physical force/rigid-motion comparison is
`evidence/oracle_action_types_20261008/r255_supported_handoff.png`.

Locally decoded r251 measured-state video:
`outputs/oracle_action_types_20261008/r251_bimanual/padded/floating_xhand_actual.mp4`.
33 frames at 30 fps, 1.1 s. It stops at the failed outgoing context and is not
a complete task. r247's verified actual-state video remains available above.

### Joint bridge attribution: r258–r260

r258 failed at source 33. Continuing the exact freshly validated support
commands and wrenches in r259 (658200) also failed there. This rules out a
support-program takeover difference as the explanation for this particular
failure. The r259 padded and exported rollouts have identical left commands,
wrenches and measured states through source 32. Their first four subsequent
left commands and biases remain exactly identical. Only the right controller
changes to its cuRobo bridge.

Both executions produce native eraser/plate contact after source 32; the
continuous bridge has one counted contact by source 33 and 0.105195 N s of
accumulated impulse. The plate then moves and the left non-thumb contact is
lost. There is no other normal contact on the left hand. The bridge's actual
eraser differs from its rigid-grasp command prediction by 3.650 mm and
0.054926 rad at that frame. A geometric collision-free command is therefore
insufficient evidence for this coupled bridge. This is not missing lifting
reward on the supporting plate, nor a reason to train that support PPO longer.

The runtime now records native tool/target collision before secondary grasp
retention failures, preserving contact points/normals for the existing generic
physical boundary resynthesis. r260 (658207) reruns the same physical prefix
to obtain that feedback. The correction changes local Anchor commands only;
the original core acceptance, fixed time and actual native collision gates
remain. No source name selects a controller or reward.

Machine-readable attribution:
`evidence/oracle_action_types_20261008/r259_bridge_attribution.json`.
Latest verified local measured-state video:
`outputs/oracle_action_types_20261008/r259_bimanual/exported_core/floating_xhand_actual.mp4`
(33 decoded frames, 30 fps, failed 1.1 s prefix; not a complete trajectory).
r259 used 6.56 GiB of resident host memory and 1 min 42 s of wall time. Future
short bimanual jobs request 20 GB host memory rather than 40 GB, with the same
20-minute interactive H200 allocation.

r260's native report identifies a 12.623 N loaded eraser/plate contact at
source 33, with a positive 2.177 mm separation inside the 2.725 mm combined
native contact band. Both the collision and secondary left contact loss are
now preserved in the report.

r261 (658211) applies the resulting 5.660 mm outward correction within the
existing grasped core interval. Native grasp remains valid and no eraser/plate
contact occurs through the core exit, but the source C.4 score rises from
0.945404 to 1.025095. The original acceptance correctly rejects it. The prior
error text wrongly blamed a compiled supporting program for this right-hand
failure; the flow now reports the actual failing hand. A genuinely changed
right context can send left Replay failure to the existing MPC/PPO fallback.

The repair now solves the clearance half-space together with the original
source position/rotation boundary: retain the measured outward displacement
and apply the minimum necessary tangential shift toward the source. The .98
interior is the existing canonical fitter's margin. For the observed case,
the proposed offset is [-2.367, -4.684, +5.427] mm, with predicted source score
0.98. This is a command proposal, not physical success. r262 (658217) performs
fresh padded/core replay and replans the bridge from the new measured exit.

### Local optimization and the missing peer-hand obstacle: r262–r272

| Run | Actual outcome |
|---|---|
| r262 / 658217 | Grasp retained, but actual core-exit source score 1.002033: rejected. |
| r263 / 658225 | Fresh padded/core A0 passed; tool rose 22.50 mm. Bridge executes all seven source frames without tool/plate contact and retains both grasps, but the next object boundary misses by 62.62 mm / 0.3218 rad. |
| r264 / 658242 | A balanced drive-memory profile does not improve that handoff. Reverted; the established bandwidth profile remains. |
| r265 / 658251 | A source-centered exit proposal fails right core tracking at frame 31. This reveals that the two-hand script wired supporting-hand fallback only. |
| r266 / 658281 | Right Replay → physical MPC → small residual PPO is now connected. PPO sampled controls pass fresh joint padded/core replay after 3 updates / 12 episodes, in 39.42 s. Tool rises 49.28 mm. The exit coordinate velocity exceeds the next planner's nominal budget; bridge rejected before execution. |
| r267 / 658301 | Adding the actual core-exit velocity compatibility check yields another padded/core artifact after 19 PPO updates / 80 episodes, in 219.88 s. Tool rises 41.26 mm. Actual outgoing transport later exceeds the fixed grasp-relative rotation constraint at 38.25; no successful bridge. |
| r268 / 658320 | Freeze r263 A0 and reuse the existing measured boundary-feedback fit. A0 commands, wrench programs, and measured exit are exactly identical to r263. Right grasp remains; left non-thumb contact is lost at source 39. |
| r269 / 658343 | The peer-hand world loads and A0 passes. Blind goal projection asks for +64.666 mm X displacement; its velocity/acceleration endpoint envelope permits only +8.005 mm. Rejected before bridge execution. |
| r270 / 658360 | Freeze the physically validated r267 command and wrench artifact. Fresh padded/core A0 passes; one coupled source/time/collision fit finds no feasible boundary. |
| r271 / 658377 | Same frozen r267 A0. Seven rotational initializations, all inside the existing oracle tolerance, find no boundary satisfying all constraints. Each proposed goal still has negative peer/world clearance; no bridge executed. Total job wall time 313.281 s. |
| r272 / 658407 | Test the same coupled fit from the earlier r263 A0, preserving its full command/wrench artifact. Pending at this update. |

The r266/r267 result is an executable sampled command artifact, **not proof
that the deterministic PPO policy succeeds**. The optimizer is still the
64×64, 24,293-parameter residual model with Appendix C.3/C.7/C.8 objectives.
The source clock, source condition, and grasp retention thresholds are unchanged.
The new exit-velocity check concerns compatibility with the configured planner;
it does not reclassify nominal acceleration as a simulator hardware limit.

Right-hand optimization reexecutes the measured initial predecessor for each
fresh acceptance, freezes the proven incoming context, and physically executes
the companion hand. Candidate snapshots come from real physical prefixes.
The accepted core exports both commands and wrench inputs; the final trial
reacquires its entry from the task initial state and executes the core slice.
No between-segment state assignment is used.

The two-hand continuation now also accepts measured bridge-end feedback through
the **same** `feedback_boundary_target` used by the established single-hand
pipeline. It requires a matching physical predecessor and source. A new path
still must satisfy unchanged timing, collision, contact, and source-boundary
checks. Feedback from a colliding rollout is not physical acceptance evidence.

Offline FCL checks of r268's actual recorded mesh transforms show intersections
between `right_hand_thumb_rota_link2` and `left_hand_link` at frames 36–39, with
none at 32–35. This confirms an omitted full-scene collision; its precise native
contact force was not recorded in that run. The overlap starts before left
contact loss and is consistent with the coupled tracking/support failure.
The planner fix uses actual rigid-body ownership, which also handles the
importer's nested left-hand path under the right-hand ancestor. Every peer
mesh becomes a convex contact envelope with its measured native contact offset;
per-link poses are refreshed during actual execution. Single-hand scenes have
no peers and keep their existing collision model. This is a geometry mechanism,
not an action-name or source-name branch.

Latest locally decoded measured-state video:
`outputs/oracle_action_types_20261008/r268_bimanual/exported_core/floating_xhand_actual.mp4`
(39 frames at 30 fps; failed 1.3 s prefix, not a complete task).
r267's video has 38 decoded frames at 30 fps (source-boundary recording before
the failure at 1.275 s).

**Remaining acceptance:** reach A1, validate A1/A2 padded and exported-core
artifacts, replace initial left synthesis-context motion with its proper
RELAXED entry plan, and complete the uninterrupted 3.666667 s rollout with task
success. `full_pipeline_success` remains false. Path diversity is downstream.

### Collision-fitting numerics (2026-10-11)

The collision constraint was evaluated through float32 cuRobo geometry, while
SLSQP estimated its full pose/velocity Jacobian by finite differences. r271
shows several internal LSQ failures and iteration limits; this does not
establish physical impossibility. The optional collision fit now consumes the
existing cuRobo distance gradient, including the six-coordinate wrist FK chain.
Source and discrete timing rows retain float64 differences. Distance/gradient
queries are cached by pose because endpoint velocity does not change geometry.
No constraint, threshold, or source clock changes. The established fit without
collision input is retained. r272 started with the preceding numerical backend;
this gradient change awaits a matched fresh physical rerun.
