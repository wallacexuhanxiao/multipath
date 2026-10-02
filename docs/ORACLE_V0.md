# Oracle Interaction Anchor V0

## Current iteration gates

1. Each Anchor independently passes its padded physical rollout, then an exact
   replay of its exported core from the physically reached core entry.
2. All Anchors and cuRobo bridges execute continuously from initial state within
   the original 4.8 s. Failures are reported at the segment that actually fails.
3. Only after gate 2: diversity, second-sequence transfer, and later segmentation work.

`hpc/run_oracle_anchor_batch.pbs` runs independent local development processes
inside one Hopper interactive allocation. A1 runs alongside snapshot acquisition
for later Anchors; A2/A3 start independently once measured states are available.
A1 success immediately triggers a continuous connection through A2, without
waiting for A3. `--through-anchor` and `--continue-development` support this
prefix experiment; it is never reported as a completed 4.8 s trajectory.

Historical traces lack velocities/contact and cannot be restored as snapshots.
`scripts/acquire_anchor_snapshots.py` can physically re-execute their controls
in the current scene to measure new states. This is development-only acquisition,
not the Oracle runtime or proof of task success. No historical/GT object pose
is assigned during execution. Missing acquisition remains an explicit failure.
Independent development does not waive final boundary reachability.

### Container startup repair (2026-10-02)

Job 642792 ran on H200 hopper-17 for 2m13s, but all four container launches
failed before Python with `bash -c echo 53394` timing out after one minute.
The exact command comes from the image's
`/.singularity.d/env/92-env-torch-distributed.sh`: MASTER_PORT defaults to
`4242 % 16384 + 49152`. Its unnecessary Bash subprocess loads BASH_ENV
(`/etc/bash.bashrc`), which sources NVIDIA `/etc/shinit_v2` and probes the GPU
with nvidia-smi/cudaCheck. Apptainer 1.3.1 applies a one-minute command timeout
during environment evaluation. The specific blocked GPU-probe subcommand was
not captured; the failing outer command and initialization chain were identified.

The batch binds a copy of that script with just the Bash subprocess replaced
by equivalent shell arithmetic. Other distributed settings are retained.
A CPU-only patched-container check returned the unchanged port 53394 in 0.29s.
Job 642822 reruns on one H200 for at most 40 minutes and first checks a real
CUDA allocation before starting Anchor work. Its GPU result is still pending.
The batch now exits unsuccessfully when the final physical gate fails, rather
than hiding container/experiment failures behind a successful summary write.

## Method and entrypoints

`configs/oracle/put_in.json` retains the existing manually reviewed cores:
35–76, 82–101, 111–128 at 30 fps, 144 source frames. Each has six context
frames on each side. The source's reviewed semantics are hold/tilt/return/release;
the TACO dataset label “put in” does not override that existing review.

1. `scripts/prepare_oracle_anchors.py`: raw human MANO/object data → retarget
   only the padded intervals, with the existing EgoEngine HandRetargeter.
   These outputs are explicitly unvalidated kinematic proposals.
2. `scripts/run_oracle_v0.py`: initialize the mounted Franka/XHand once;
   plan RELAXED intervals from actual state; execute and validate each local
   Anchor, retaining measured entry/exit, velocities, contact, commands and provenance.
3. `--validated-anchors <development/anchors>`: a fresh final rollout from
   the task initial scene, using the physical canonical states and recorded
   commands. It may not restore snapshots or set state at handoffs.
4. `hpc/run_oracle_v0.pbs`: sequences these stages on Hopper interactive.
   A failed development stage prevents final validation from running.

Required job inputs: `MULTIPATH_SCHEDULE`, `MULTIPATH_TRIPLET`,
`MULTIPATH_SEQUENCE`. Optional output name: `MULTIPATH_RUN_NAME`.
No VLM, reference-event compilation, full robot reference, or path diversity
is used by this entrypoint. Existing experiments remain available separately.

## Boundaries and timing

The M2 development entrypoint is `scripts/develop_mounted_anchor.py`, submitted
with `hpc/develop_mounted_anchor.pbs`. It consumes an actual predecessor trace
at `synth_start`, preserving measured joint/object positions AND velocities.
It executes the entire padded window, assesses interaction fidelity only in
the core, and records the physically produced core entry/exit. Only core
commands are exported as the final Anchor. Post-context checks grasp retention.
The first experiment uses the r10 predecessor; this does not certify a bridge
to a changed canonical core entry. That connection still needs fresh validation.

On replay failure, a small shooting MPC searches arm and finger residuals in
the mounted PhysX model. Its candidate restores and traces are explicitly
development-only. The selected controls must pass another uninterrupted padded
replay. No prediction candidate is itself accepted as an Anchor. PhysX contact
caches are recomputed on the first commanded step, not restored or warmed up
using extra source time. This new physical path is awaiting its first Hopper
result; implementation is not evidence of M2 success.

cuRobo uses the actual XHand and, when grasped, actual carried-object geometry.
Joint-space planning consumes only boundary states. Human object fidelity is
checked inside Anchor cores and at prescribed handoffs, never along a relaxed
bridge. Contact retention is an explicit persistent constraint.

A path that cannot physically fit its source budget is rejected. When uniform
resampling fails, squared-path-speed reachability allocates time along the path
subject to velocity and acceleration constraints; all resampled states are
collision-checked before execution. cuRobo’s
proposed duration is not assumed to be the minimum feasible duration. Resampling must satisfy
velocity and acceleration limits including boundary velocities; collision is
checked again on the resampled path. Actual dynamics are also measured. Planning velocity is passed directly as
arm velocity feedforward; finite differences against a stale position target
are not used. Local mounted retargeting follows a continuous IK branch and
fits commands at physics rate under the same dynamic limits before replay.
This kinematic fit is not called physical MPC.
Time is never extended and commands are never silently clipped.

Initial ready configuration is selected with the mounted collision model.
A proposed free-hand Anchor entry is searched within one object extent using
the same full collision geometry. Its offset decays over the existing context
length during local replay; replay must still satisfy core interaction checks.
A held boundary preserves the measured hand/object transform.

## Current implementation limits

- Mounted padded replay and MPC are implemented for local development; local RL
  remains unimplemented and is considered only after measured MPC failure.
- Artifact entries declare `initial_state`, `predecessor_rollout`, or
  `physically_acquired_snapshot`, with trace provenance for non-initial entries.
  `reference_constructed_state` cannot be verified. Canonical artifact loading
  requires a completed padded physical replay with no internal resets.
- The earlier sequential core-replay developer remains diagnostic. Its artifacts
  cannot bypass the new padded-replay requirement for canonical playback.
- Physical canonical artifact playback is implemented but has not yet passed
  a full put-in rollout. Do not claim V0 or diverse-path success.
- Contact forces identify hand/tool contact; grasp retention additionally uses
  measured relative motion. Contact alone is not load-bearing proof.
- The development and final reports distinguish partial failure from success.
  Actual-state traces/video must be interpreted with those reports.
