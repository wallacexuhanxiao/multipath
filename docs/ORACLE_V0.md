# Oracle Interaction Anchor V0

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
