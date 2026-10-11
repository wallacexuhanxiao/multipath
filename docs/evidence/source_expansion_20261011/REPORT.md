# New source expansion: execution repairs

## Goal

Add new TACO sources to the actual-state three-solution overlay demo. Existing
successful Put-In demos remain the baseline. A local Anchor success is not a
complete trajectory success.

## Observations from r317

- Put-In `20231024_258`: initial local acquisition Replay completed through
  frame 65. Fresh padded Replay failed at core exit frame 59 with 73.54 mm
  object position error and 0.47990 rad rotation error; native grasp persisted.
- Replay and fresh padded runs had identical drive commands and actual entry
  state. Their feedforward wrench programs differed because the exporter
  recomputed feedforward from already modified acquisition drives.
- Pour `20231024_246`: the first local interaction failed. After refinement,
  core-exit native opposition was absent; boundary error was 55.47 mm and
  0.39039 rad. This is a physical interaction failure, not a successful demo.
- Empty bowl `20231027_149` and Empty cup `20230928_031`: local preparation
  completed in 392.47 s and 377.81 s. Simulator object initialization failed
  because `isaaclab.sim` is unavailable in the Isaac Sim execution environment.

## Generic changes

1. A validated floating Anchor can now retain its executed command and native
   feedforward wrench arrays together. Fresh padded/core validation, physical
   predecessor reproduction, export, and continuous execution consume the
   same pair. Exact saved programs skip acquisition feedback/preload rewrites.
   Reshaped proposals use newly computed feedforward and need fresh physics.
2. Source objects missing from the cache are converted generically from TACO
   centimeter meshes to meter meshes. Native USD authors mass, rigid-body and
   convex-decomposition collision properties without an IsaacLab dependency.
   Existing cached object assets are reused.
3. The shared interaction configuration records CORE interior tracking error
   as a diagnostic. Physical source boundary conditions, contact retention,
   time budgets, collision, and final measured task conditions remain gates.
   MPC/PPO and full execution use the same policy.
4. Local acquisition libraries may skip redundant surface IK for the same
   object geometry. Actual contact success still requires fresh execution.
   The raw initialization fallback regenerates its own surface IK if needed.

The changes do not dispatch on task or sequence names. The two Empty schedules
are explicit manually reviewed oracle inputs. These scenes model container
handling; they do not model liquid or granular content transfer.

## Verification status

Changed Python sources parse and `git diff --check` passes. r318 job `659415.hopper-m-02` uses a 20-minute H200 interactive allocation and
reuses completed local proposals. Its four source inputs are in
`configs/oracle/taco_demo_expansion_replay.json`.

The native control repair is now physically verified on Put-In 258 Anchor 0:
Replay, fresh padded Replay and fresh exported CORE Replay all succeeded. The
Replay and padded command arrays are bitwise identical; their native wrench
arrays are also bitwise identical (both maximum differences are zero). The first continuous candidate then executed A0, its following bridge and A1.
Its next bridge rejected the actual entry velocity above the planning budget.
Reached-entry local synthesis corrected A1 (exported exit velocity ratio
0.80526), after which that bridge also passed. The release Anchor was then
locally repaired and validated.

Native USD conversion also succeeded for the new bowl 194, cup 095 and target
158; both Empty sources reached physical interaction trials. No new complete
successful three-solution demo is claimed here.

Local measured-state evidence:
`outputs/new_source_demos_20261011/r318_258_anchor_0/exported_core/anchor_0_actual_h264.mp4`
(34 decoded frames, 30 fps, 1.133 s; a local CORE, not a full task).

## Final V0 result within this turn

Put-In `20231024_258` completed a fresh artifact-consuming, initial-state full
PhysX rollout: all three Anchors and four cuRobo intervals passed, task success
was measured from simulator state, no intermediate reset occurred, and actual
duration equals source duration (128/30 = 4.2666666667 s). No VLM or complete
robot reference was required.

The locally decoded H264 video contains all 128 frames:
`outputs/new_source_demos_20261011/20231024_258/final/complete_actual_h264.mp4`.

Pour 246 and both Empty sources still failed their first physical grasp
interaction after refinement. They are not successful demos. The three-solution diversity stage completed its first bank search but failed
with `No alternative solution available in every interval`. Only the baseline
is counted as a complete successful rollout; no new three-path overlay is
claimed. The V0 success remains valid. Further interval detour search is left
for the next turn because the user requested this turn finish within ten
minutes.


## Continuation: r319–r326

### Two complete trajectories of the new source

Put-In 258 now has the original V0 and one complete spatial alternative.
r323 solution 1 executed all seven intervals from the actual task initial
state. Both rollouts last 128/30 = 4.2666666667 s, have zero segment resets,
and pass measured final object pose, release and stability checks. The
alternative final object error is 30.95 mm / 0.21572 rad; final object speed
is 0.000104 m/s. Three solutions are not yet claimed.

Its measured path differs from the baseline in every RELAXED interval:

| Source frames | Mean nearest-path distance | Maximum nearest-path distance |
| --- | ---: | ---: |
| 0–25 | 6.41 mm | 17.71 mm |
| 59–68 | 8.42 mm | 14.45 mm |
| 88–96 | 16.79 mm | 25.04 mm |
| 117–128 | 66.67 mm | 84.19 mm |

The final free-hand stopping endpoint is unconstrained by the task, so its
larger separation also includes different stopping positions. Distances
above describe the actual paths; no path is magnified for presentation.
The task models container handling, not transfer of liquid or granular content.

Local presentation: `outputs/new_source_demos_20261011/20231024_258/presentation/showcase_overlay.mp4`.
It overlays two successful actual mesh recordings and measured wrist traces.
Only playback is slowed: RELAXED at 0.25×, Anchors at 1×; the clip is
287 frames / 30 fps = 9.5667 s. The renderer now labels the actual count of
successful runs instead of always claiming three. Videos decode locally,
and the overlay preview was visually inspected.

### Reusable failures and repairs

1. Diversity predecessor reproduction and following-CORE validation had
   independently rebuilt feedforward from saved drive targets. The reproduced
   state no longer matched the validated rollout. Both now replay the exact
   drive/wrench pair, sliced by recorded source frames rather than assuming
   every controller record starts at frame zero. r319 reproduced the baseline
   state at frame 59 with maximum difference zero in wrist pose/twist,
   hand q/dq and tool pose/velocity. A focused real-record check also verified
   cropped-source slicing and rejection of incomplete controller coverage.
2. Every r319 detour candidate for [59,68) was rejected by a conservative
   waypoint sphere proxy (24/24). The detour code now reuses the existing
   full-mesh cover refinement at invalid rotated waypoints and fitted paths.
   Physical geometry remains covered and actual collision still rejects the
   path. r320 found the two missing alternative bridges in two attempts;
   all four local interval banks now contain three physically validated paths.
   A complete rollout remains required; local counts are not task successes.
3. Resume discarded repaired Anchor artifacts and selected the first failed
   full attempt instead of the latest actual attempt. It now retains the
   latest validated complete artifact set and latest executed prefix. The
   repaired release Anchor enabled r323 solution 1 to finish. No simulator
   state is patched; each final attempt begins from the task initial state.
4. Zero-residual boundary shooting round-tripped every quaternion and changed
   floating-point bits. This made an exact-equality check discard native force
   records. Zero residual now returns the original commands exactly and a
   nonzero residual leaves its untouched prefix exact. In r325, the identity
   trial kept the successful grasp and reached the genuine outgoing fixed-time
   feasibility rejection; the earlier r323 identity trial lost the grasp.
5. Nonzero boundary proposals still recomputed the whole controller program,
   removing physically acquired contact preload before the residual began.
   The r326 repair preserves the recorded prefix and force program; at the
   actually reached residual onset, it adds only the modeled response
   difference between new and original commands. Fresh physical interaction,
   outgoing bridge and exported CORE checks remain necessary. This last
   change is under physical verification, not yet a successful third rollout.
6. Development restores wrote dynamic velocities to fixed kinematic target
   objects, producing repeated invalid PhysX API errors. Fixed environment
   roles now skip those velocity calls. This preserves their authored fixture
   support; it does not introduce an object attachment or final-runtime reset.

None of these mechanisms branches on a task or source name. The baseline
bridge solver and nominal timing/limit budgets remain fixed.

### New interaction types: measured attribution

The lifting reward is already present (Appendix C.8), alongside object
tracking (C.3) and native thumb/non-thumb contact (C.7). A new lifting loss
was not the missing component. URDF identity between retarget and native
physics was checked. Fingers tracked their joint targets closely while the
wrist and real contact geometry disagreed with nominal surface fits.

For Empty bowl 149, r318's commanded palm went 36.60 mm below the table,
while the actual palm was table-supported (0.44 mm geometric penetration).
The goal had followed rotation produced by one-sided contact before a grasp
existed. Local free-motion mass/inertia feedforward alone (r321) left the
wrist error almost unchanged (37.53 → 37.46 mm) and did not acquire a grasp.
It is retained as a disabled experiment, not promoted as a successful repair.

r324 keeps the stationary-entry grasp orientation relative to the fixed
support during pre-grasp one-sided contact, while following actual tool
translation. Full measured grasp conditioning starts after native opposition.
This shared experiment reduced Empty bowl Replay wrist error to 7.08 mm;
Pour Replay wrist error was 7.35 mm and cup 8.64 mm. Nevertheless, all three
still lacked native thumb opposition at core exit. Replay, local MPPI and
small residual PPO therefore failed the physical Anchor gate. These sources
produce zero complete successful demos in this batch. Improved tracking
is a component result, not proof of successful grasp or cross-task transfer.

Raw attribution: `continuation_summary.json` and the r319/r320 local-bank
reports in this evidence directory. r326 uses H200 interactive job 659580,
20-minute wall time, reusing the two complete successful rollouts. The
remaining physical gate is the third continuous trajectory; broader task
expansion still requires successful acquisition on a different source type.
