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

As of the r17 search, there is one accepted source-clock baseline (r11) and
zero accepted diverse solutions. The closest full candidate, r17 attempt 2,
ran 144 frames / 4.8 seconds: peak tilt 47.21 degrees, final position error
3.30 cm, actual return-path mean distance 2.34 cm from baseline, Hausdorff
4.20 cm, and minimum virtual-box clearance 6.10 mm. It was rejected because
relative hand–bowl rotation drift reached 25.62 degrees, exceeding the
unchanged 0.35-radian gate. No thresholds were weakened.

The r18 controller uses a smoothstep time law blended according to available
velocity slack. Its endpoint easing is reusable and locally tested, but its
physical result is pending. Tests cover geometric translation/scaling,
unimodal sizes, cumulative source acceptance, interval selection, and
fixed-budget velocity-bounded easing. These offline tests do not establish
physical success.

Evidence is under `docs/evidence/source_clock_diversity_20261001/`.
The previous Chinese repair report describes its original delivery state;
its statement that default settings still use r12 was historical. The current
source-clock configuration has been restored to r11's 0.98 velocity coefficient
and zero tilt lead; now each rollout snapshots its actual configuration and
implementation. This does not imply deterministic r11 reproduction.
