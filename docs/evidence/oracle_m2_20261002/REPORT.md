# M2: mounted local Anchor synthesis

## Status

M1 remains passed by r10. **M2 is not yet physically validated.**
The first padded mounted experiment is `oracle_m2_padded_r12`, Hopper
interactive job `642740.hopper-m-02`. At the last check it was queued:
`Not Running: Insufficient amount of resource: node_pool`.
Queued core-only r11 (`642710`) was cancelled before execution. It has no result.
There is no new physical video yet.

## Failure mechanism and repair

Observed failure: r10 reached the first Anchor but could not execute its local
commands. Its preceding geometric IK targets had branch jumps; padding only
affected retargeting/smoothing, not a physical synthesis rollout.

General invariant: Anchor feasibility must be established on the mounted
arm + hand, from a physically acquired entry, using the whole synthesis context.

Implemented:

- Restore r10's measured source-frame-29 snapshot (0.966667 s) for development.
  It includes arm/hand positions and velocities, object poses and velocities,
  and drive targets. Trace and predecessor report hashes are retained.
- Execute synthesis frames 29–82, with core 35–76. These boundaries come from
  the oracle schedule, not execution-code branches. Core entry/exit are measured
  after real execution. Export only the core's 120 Hz commands.
- Use continuous local IK and dynamics-constrained fitting as initialization.
  This is not itself physical MPC or an interaction-success claim.
- On replay failure, run bounded shooting MPC over arm and hand residuals in
  the mounted PhysX model. Candidate traces and restores are separate. The
  selected controls must pass a fresh uninterrupted padded replay.
- Keep the existing core pose/contact tolerances. Post-context checks retention.
  Reject wrong source duration, measured actuator violations, incomplete replay,
  or invalid core boundaries. No waiting or clock stretching is introduced.
- Reject `reference_constructed_state`; require explicit entry origin and
  provenance. Final canonical loading also requires completed padded replay
  with zero internal state restores.

No bridge-planning, time-parameterization, or bridge-controller code changed.
Local RL is not integrated. No multi-path or VLM work was added.

## Evidence limits and next decision

Four focused invariant checks and Python/shell syntax checks passed locally.
These do not establish a valid grasp, successful MPC, or M2 completion.
PhysX contact solver caches are not serialized; contact is recomputed on the
first commanded step without extra warm-up time. The uninterrupted acceptance
replay is required because prediction restores are only development evidence.

Padding may change the canonical core entry. r10 therefore establishes the
origin of the development snapshot, not a successful bridge to that new entry.
A fresh source-timed bridge and final continuous rollout must later verify it.

Transfer is structural at this stage: all mechanics consume schedule boundaries,
joint limits and measured state; no task/sequence/object-name branches were
added. Transfer to a second sequence and actual M2 success remain untested.
