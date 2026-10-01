# Source-clock path diversity

This extension preserves the original 144-frame schedule and its BC intervals. New candidates use cuRobo in the existing motion-planning intervals. A candidate is accepted only when the full physical rollout, source-time checks, and actual wrist exclusion checks all pass.

## Reusable small–large–small obstacle profile

`src/multipath/path_exclusion.py:tapered_obstacles_along_path` accepts an arbitrary world-coordinate Nx3 curve. It samples by arc length, uses normalized distance u, and sets the requested half-size to `max_half_size * sin(pi*u)^2`. Entry and exit protection can reduce sizes; a monotone envelope retains the increasing/decreasing shape. Tiny or geometrically infeasible boxes are omitted. Neither obstacle positions nor phase frame numbers are hardcoded in this function.

The CLI exposes the same implementation:

```bash
PYTHONPATH=src python scripts/build_path_exclusions.py \
  --accepted-run outputs/source_clock_r11 \
  --size-profile tapered --exclusion-scope wrist \
  --output outputs/exclusions.json
```

Two scopes are explicit in the saved configuration:

- `wrist`: virtual boxes exclude the wrist curve. cuRobo retains full robot, hand, and carried-object collision checking against real scene geometry. The route search proposes geometrically derived waypoints around the exclusion tube, plans each connection with cuRobo, and rejects curves crossing virtual boxes. No virtual objects are inserted into PhysX.
- `robot`: virtual boxes join the planner's collision world and protect full entry/exit geometry. The short 0.20/0.33-second intervals in this sample had no feasible space for this stronger construction. The wrist setting is not evidence that the full body clears the virtual boxes.

Actual execution is checked separately from the planned curve. `compare_path` densely interpolates the recorded wrist polyline at at most 1 mm spacing and requires more than 2 mm virtual-box clearance. This is an approximation from recorded samples, not a continuous swept-body proof.

## Search and evidence

`hpc/run_source_clock_diverse.pbs` runs sequentially in Hopper interactive. It starts from the successful source-clock r11 rollout, applies exclusions to selected interior cuRobo windows, and stores candidates separately. `MULTIPATH_WINDOW_POLICY=all` selects all interior windows; `longest-budget` chooses the longest interior window automatically, allowing diversity where more motion time is available without changing any BC interval. Failed candidates do not become future exclusion sources. Additional accepted solutions accumulate exclusions from every preceding accepted path.

Each candidate saves the source-clock configuration and the implementation modules used. The accepted baseline uses physical velocity coefficient 0.98, no tilt lead, finger blend 0.25, closure 0.20, and hold frame 54. These are sample-specific settings from the earlier report, not a general grasp solution.

The implementation of geometry generation is reusable; end-to-end cross-sample execution and repeated grasp reliability remain unproven. A baseline repeat (r13) failed at the return boundary due to world collision after the tilt. That failure is retained and is not accepted as a baseline. Source-clock diversity must still pass full physical execution on every new candidate.

## Fixed-time feasibility

Detour waypoints use locally continuous IK when collision-free, with cuRobo pose search as a fallback. The target wrist pose at the next interaction comes from the accepted baseline's actual state. Candidate joint travel is converted into a minimum required time using the commanded velocity limits. Routes exceeding the original window are rejected before execution. This is a necessary speed check, not a sufficient acceleration, contact, or tracking guarantee.

Resampling uses cumulative joint travel rather than subplan sample index, so stops and unequal subplan lengths do not create artificial command jumps. With `--shrink-on-retry`, unsuccessful candidates retry using a smaller peak obstacle size; the normalized small–large–small profile stays unchanged. Every attempt saves its own exclusion configuration.

## Current validation boundary (2026-10-01)

Hopper job `640489.hopper-m-02` completed the r22 search. Attempt 1 was
rejected before execution because its detour needed 0.379 s in a 0.333 s
window. Attempt 2 completed physically but failed the unchanged semantic and
grasp gates. Attempt 3 was accepted as solution 2, following the r11 baseline.
Both accepted runs have 144 recorded PhysX frames at 30 fps (4.8 s), with the
bowl retained through tilt and return, then released. The BC windows are
unchanged; the distinct cuRobo route is in the return interval `[101, 111)`.

For the accepted second run, the actual peak tilt is 53.16 degrees at video
frame 98, six frames after the reference peak (66.53 degrees at frame 92).
The final bowl position is 4.95 cm from the reference, below the 5 cm gate.
Maximum held relative translation is 2.40 cm. Maximum held relative rotation
is 0.3477 rad (19.92 degrees), close to the 0.35 rad gate. The measured peak
joint-speed ratio is 1.0107, inside the 1.05 tolerance. The actual wrist
maintains 3.45 mm minimum clearance from the virtual exclusion boxes,
above the 2 mm gate; its mean nearest-path distance from r11 is 0.95 cm and
symmetric Hausdorff distance is 1.46 cm. This is a measured but modest path
difference, not a large alternate route. Virtual clearance checks the wrist
only. The real scene's full robot, hand, and carried-object collision checks
remain active, but the virtual boxes do not prove swept-body clearance.

The accepted result is one successful second solution on one sample, with
position and rotation margins close to their gates. It does not establish
repeatability or transfer to other samples. Pouring is judged from bowl tilt;
no liquid or contents are simulated. The first attempted larger exclusions
and r17 candidate show that a visually larger route can fail the fixed time
or grasp constraints. No acceptance thresholds were weakened.

The r18 controller added a smoothstep time law blended according to available
velocity slack. The later r22 run additionally uses accepted-baseline joint
handoff, live boundary protection, and bounded slip feedback. Offline tests
cover geometric translation/scaling, unimodal sizes, cumulative source
acceptance, interval isolation, and fixed-budget velocity-bounded easing.
Physical validation is the recorded r22 rollout, not these offline tests.

Local videos: `outputs/source_clock_r11/mounted_xhand_bc_curobo_actual.mp4`,
`outputs/source_clock_diverse_r22/solution_02_attempt_03/mounted_xhand_bc_curobo_actual.mp4`,
and `outputs/source_clock_diverse_r22/baseline_vs_second_solution.mp4`.

Evidence is under `docs/evidence/source_clock_diversity_20261001/`.
The previous Chinese repair report describes its original delivery state;
its statement that default settings still use r12 was historical. The current
source-clock configuration has been restored to r11's 0.98 velocity coefficient
and zero tilt lead; now each rollout snapshots its actual configuration and
implementation. This does not imply deterministic r11 reproduction.

## Per-interval isolation

Virtual path exclusions belong to a single motion-planning interval. Each
interval accumulates only the paths previously accepted for that same interval.
`exclusions_for_window(config, start, end)` activates the exact matching set;
other intervals and BC receive none of those virtual obstacles. Each cuRobo
interval constructs its real-scene collision world afresh. Spatial overlap
between two different intervals does not make their virtual obstacles shared.
Real table/object collision geometry remains shared scene geometry.

A regression test builds two intervals with two accepted trajectories each,
checks that their generated obstacle sets remain separate, and verifies that
a path rejected by one interval's exclusion set is not rejected by the other
interval's set. The current `longest-budget` experiment varies only the return
interval; it does not claim diversity in every motion-planning interval.


## Live anchors and bounded grip feedback

Source-time path extraction includes the BC-to-motion handoff sample. Before
planning a wrist detour, the current interval rebuilds its tapered exclusions
with the actual wrist start and fixed end protected. All previous paths for
that interval must remain excluded; removing the complete diversity constraint
raises an error. The adjusted boxes are saved as `resolved_path_exclusions.json`
and used both for planning and actual-path acceptance. Other intervals are
unchanged.

`configs/put_in_source_clock_feedback.json` is an experimental alternative to
the baseline: object motion relative to the grasp triggers a bounded increase
in finger position targets (at most 0.10 rad, at most 0.15 rad/s, joint limits
respected). It does not attach, reset, or directly command the object. The
closing-joint mask remains XHand-specific; feedback thresholds and margins
still require validation on other objects and hands. The current alternate
physical velocity coefficient is 0.94; task acceptance thresholds are unchanged.
