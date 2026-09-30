# Mounted XHand BC + cuRobo execution

The physical entry point is `scripts/run_mounted_bc_curobo.py`. It uses Isaac
Sim 5.1 and cuRobo v0.7.7 on Hopper. Submit `hpc/run_mounted_pipeline.pbs` to
`interactive`; jobs run sequentially because this queue permits one running
job per user. The held unrelated job is not part of this pipeline.

## Inputs and retained behavior

- `assets/putin_xhand_reference.npz`: existing 144-frame XHand reference.
  Its source and checksum are in the adjacent JSON file.
- `configs/put_in_bc_curobo_contact_checked.json`: candidate BC windows
  `[43,65)` and `[86,122)`; the original VLM schedule is retained separately.
- `configs/franka_xhand_mount.json`: one fixed transform used consistently
  for both the physical mount and wrist IK.
- Original XHand URDF and original bowl 180 / target 049 USD collision assets
  are reused from the previous Hopper checkout.

The right XHand has its original 12 finger joints. A physical fixed joint
mounts it to the Franka arm. The original Franka gripper, including its
housing, is hidden and has its collisions disabled. The old gripper's
collision spheres are replaced with XHand spheres in the planner.

The old finger position commands cannot simply be stretched in time and
expected to reproduce the old measured hand shape. The runner exposes
`--finger-command-blend`: the target is the measured finger position plus
this fraction of the original command-minus-measurement residual. A value
of 1 reproduces the original commands; 0 follows measured finger positions.
The current physical repair uses 0.25 with the original force drive gains
5 / 0.5 and maximum effort 3. This is a derived BC controller, with the raw
source preserved. It is not evidence of unchanged policy equivalence.

## Continuous execution

The arm is initialized once. No robot or object state is assigned between
windows. cuRobo starts from the measured arm state and receives obstacles
from measured object poses. Planning uses conservative sphere covers of the
current XHand and, during transport, the physically held object.

BC follows measured wrist poses through continuous IK and synchronized time
parameterization of arm and finger motion. Local contact adapters descend
from a collision-free precontact anchor, lift and hold after acquisition,
and withdraw after release. A contact state is never passed directly to a
free-space planner as though it were collision-free.

After grasp acquisition, the placing chunk is translated using the measured
wrist-to-object transform and the current target geometry. The chunk's
relative wrist motion and finger sequence are retained. Free-space execution is slowed
by a factor of 3, with the acquired finger target held until the next BC.

`--start-at-bc` is a diagnostic only: it omits the free-space prefix and starts
at the first BC state. Do not describe such a run as the full five-window
pipeline. Full validation must omit this flag.

## Physical evidence

Each output directory contains:

- `pipeline_result.json`: stage status, planning results, timing, grasp
  lift/hold, transport retention, and final placement checks.
- `actual_trace.json`: measured arm, finger, wrist, bowl and target states.
- `visual_meshes.npz` / `visual_actual.npz`: actual mesh geometry/transforms.
- Planner sphere snapshots for diagnosing collisions.

Render the measured state recording locally with:

```bash
MUJOCO_GL=cgl /tmp/ego-render-env/bin/python scripts/render_mounted_actual.py \
  --run-dir outputs/RUN_NAME
```

The renderer replays measured PhysX transforms without running another
physics rollout. It verifies the video opens and has the expected frame
count. `scripts/analyze_mounted_actual.py` independently summarizes the
recorded motion. A completed script, a reachable wrist trajectory, or an
object lifted transiently is not enough to establish task success.

Final acceptance requires full continuous execution, retained grasp through
transport, released bowl inside the target footprint and near its support
height, stable for the final second, and inspection of the actual video.
The final task test currently uses a conservative elliptical target
footprint; it is specific to this demonstration rather than a general VLM
success judge. Left-hand behavior is not included in this single-arm scene.

## Repaired grasp artifact

`assets/putin_grasp_repaired.npz` preserves the command stream and measured
entry of the physically successful grasp in run `mounted_pipeline_r16`.
It is derived from the original XHand BC, with arm mounting, time scaling,
and contact approach/lift adapters. `--repaired-grasp` replays that fixed
command stream after continuously reaching its entry; it never assigns the
robot or object state at the handoff. Passing the earlier isolated run does
not establish that the prefixed run succeeds. The full run checks entry
object displacement and retained lift after replay.

The alternate `--object-relative-bc`, `--grasp-hold-frame`, `--arm-kp`, and
`--physics-friction` switches are diagnostic repair candidates. Their mere
presence does not imply they improved success. See individual result files.

## Current boundary repair diagnostics

Pose targets are constructed from fresh NumPy copies for every IK solve.
Passing slices of a shared Pose batch to the solver had allowed subsequent
queries to alter targets used elsewhere. Every BC handoff and precontact
query now checks the returned FK against its requested XYZ independently.

A Cartesian entry path reduced observed bowl displacement at the first BC
entry from 16.9 mm (joint interpolation) to 1.7 mm. That alone did not make
the fixed grasp robust. `hpc/run_grasp_closure_sweep.pbs` tests four small
finger residuals sequentially in a single interactive allocation. Each
candidate begins at the ready pose and retains the same physical gates.

The completed sweep found retained lifts of 98.9, 101.0, and 107.4 mm for
closure residuals +0.10, +0.20, and +0.30 rad respectively; -0.10 rad failed.
These are grasp-only results, not completed placement trials. Their old
terminal settle replaced commanded finger targets with measured angles,
unloading the grasp after the retention gate. The runner now retains the
last applied command during settle. The +0.10 rad candidate completed all
five windows in `mounted_pipeline_r25` (PBS job `639526.hopper-m-02`).

`full_pipeline_success` requires terminal placement success, every schedule
window, and neither `--start-at-bc` nor `--grasp-only`. Diagnostic outcomes
remain explicitly identified even when their local physical gate passes.

## Full-schedule result, 2026-09-30

Run `mounted_pipeline_r25` starts at the elevated ready pose, executes all
five windows, and records zero intermediate state resets. Both
`task_success` and `full_pipeline_success` are true. The measured retained
lift is 86.3 mm, transport relative drift is 20.6 mm, and final bowl support
gap is 3.0 mm. The final footprint score is 0.903 (must be below 1); final
one-second drift is 0.000514 mm. The 53.075-second actual-state video has
797 frames at 15 fps. Grasp, transport, and final release frames were
inspected: the bowl is released onto the plate and the hand withdraws.

This is a scene-specific repaired controller: it uses the saved r16 grasp
command stream, a +0.10 rad finger residual, and the r15 placement offset.
It is not an unchanged-policy or cross-embodiment generalization result.
Isaac runs CPU PhysX; cuRobo uses CUDA on the allocated H100. An identical
configuration repeat, `mounted_pipeline_r26` (PBS job `639579.hopper-m-02`),
also passed all five windows with zero resets: retained lift 90.2 mm,
transport drift 20.1 mm, footprint score 0.882, support gap 3.0 mm, and
final one-second drift 0.000301 mm. Its simulated duration is 50.0 seconds.
Both runs disabled `start_at_bc` and `grasp_only`. These two consecutive
passes establish repeatability for this fixed scene/configuration, not a
statistical success rate or robustness to changed initial conditions.

Submit from `/scratch/e0985859/multipath_v0_fc2da7b` on Hopper:

```bash
/cm/shared/apps/pbspro/current/bin/qsub hpc/run_mounted_pipeline.pbs
```

Each allocation uses a separate `outputs/mounted_pipeline_<job number>`
directory by default. Set `MULTIPATH_RUN_NAME` through qsub's `-v` option
only when an explicit new run name is needed. The script runs the full
schedule; it does not enable either diagnostic shortcut.

## Automatic sequential path diversity

The obstacle placement API is `multipath.path_exclusion.obstacles_along_path`.
It takes any world-space `N x 3` curve, an obstacle half-size, optional sample
spacing, and optional protected entry/exit collision spheres. It samples the
whole curve uniformly by arc length and removes candidates intersecting the
protected geometry. No Put-In frame numbers, fixed middle fractions, or hand
selected obstacle coordinates are used. With no robot geometry, it protects
endpoint neighborhoods; if no room remains it reports failure rather than
silently returning an empty obstacle set. Coordinate units must match the
obstacle size (the mounted scene uses meters).

`build_path_exclusions.py` adapts accepted mounted rollouts to this API. It
automatically discovers all cuRobo intervals between BC windows, or all
cuRobo intervals if none are bracketed by BC. An explicit interval can also
be requested. It uses the saved robot/hand/payload collision spheres to
protect boundaries and keeps each interval's obstacles separate.

```bash
PYTHONPATH=src python scripts/build_path_exclusions.py \
  --accepted-run outputs/mounted_pipeline_r26 \
  --output outputs/exclusions.json
```

Repeat `--accepted-run` to exclude multiple accepted solutions cumulatively.
These obstacles exist in the planner, not as new physical props: they steer
path search without changing the manipulation task. They apply only to the
selected free-motion intervals. The planner checks its robot/hand/payload
against them; measured wrist paths are independently checked after execution.
Acceptance requires full physical task success and actual exclusion clearance.

`explore_diverse_paths.py` runs this loop automatically: build exclusions,
launch a complete rollout, reject failures, append a successful path, then
rebuild exclusions for the next solution. Its `solutions.json` records every
attempt. The default is one additional solution with at most three attempts;
the script never counts a failed candidate as a solution or an exclusion source.

On Hopper, submit `hpc/run_diverse_paths.pbs` to `interactive`. For example:

```bash
/cm/shared/apps/pbspro/current/bin/qsub \
  -v MULTIPATH_BASELINE_RUN=outputs/mounted_pipeline_r26,MULTIPATH_ADDITIONAL_SOLUTIONS=1 \
  hpc/run_diverse_paths.pbs
```

The geometric generator is reusable across curves and schedules; the scene
loader and task success predicate in the mounted runner still require an
adapter for a different manipulation task. Generic geometry tests are not
evidence of physical success on another task.

### Verified automatic search

PBS job `639659.hopper-m-02` ran `explore_diverse_paths.py` from baseline
`mounted_pipeline_r26`. It automatically identified the transport interval,
sampled 13 candidate boxes along the 0.300 m curve, and filtered 9 against
protected entry/exit geometry. The remaining 4 boxes required a new route.
No coordinates or interval frame numbers were supplied to this search.

The first candidate, `mounted_diverse_auto_01/solution_02_attempt_01`, passed
the complete five-window physical rollout and actual path-exclusion check.
Minimum measured wrist clearance from the exclusions was 66.9 mm. Mean
nearest distance from the new measured curve to the old curve was 73.5 mm;
symmetric Hausdorff distance was 106.8 mm. Final footprint score was 0.892,
support gap 3.0 mm, and no intermediate states were reset. The serial manifest
accepted the candidate and ended with two solutions. This establishes one
automatically generated alternative on the existing scene.

The earlier fixed-interior-fraction prototype `mounted_diverse_02a` also
passed physically, but the reusable implementation uses whole-curve sampling
and geometry filtering. The old prototype is not the default placement rule.

The renderer overlays virtual obstacles in pink and prior measured paths in
cyan during the applicable motion interval. These overlays are explicitly
labelled; robot, hand, and object motion always come from measured PhysX.
`scripts/plot_path_diversity.py --run-dir RUN` exports a static comparison.
