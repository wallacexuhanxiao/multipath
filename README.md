# multipath — Interaction-Anchored Trajectory Generation

**V0:** one continuous physics-verified Put-In trajectory from a single
egocentric video. Keep locally validated grasp and put-in anchors, replace the
noninteraction transport with cuRobo, and run from the initial state to task
completion without intermediate resets.

First target: `(put in, bowl, plate) / 20231024_253`.

```
Single video + reconstructed digital twin
 -> manual grasp / place segmentation
 -> local EgoEngine replay + MPC if needed
 -> verified, compatible measured anchor boundaries
 -> cuRobo attached-object joint-space transport
 -> measured state check, physical correction if necessary
 -> uninterrupted PhysX Grasp → Transport → Put-In
 -> physical task predicate + actual-state trace + video
```

## Implementation

`src/multipath/schema.py`: anchored boundary schemas and safe artifact I/O.

`src/multipath/transitions.py`: canonical boundary and fixed-grasp
compatibility checks.

`src/multipath/planning/curobo.py`: actual documented
`MotionGen.plan_single_js` and `attach_external_objects_to_robot` calls,
including observed bowl pose, matching named joints, real collision geometry
and the exact canonical end-joint configuration.

`src/multipath/orchestrator.py`: continuous single-path control flow,
measured transition checks, physical-correction hook and fail-closed acceptance.

`tests/` + GitHub Actions: offline, mock-physics protocol tests.

## Prerequisites to physically run

1. Choose an **actual matching** Isaac Sim robot and cuRobo YAML (the
original EgoEngine uses floating hands whereas DexImit uses UR5e+LEAP).
2. Produce two physical measured, mutually compatible Grasp and Put-In
anchor artifacts for the same robot and scene.
3. Install real `PhysicsWorld` adapter for the existing EgoEngine/Isaac
execution backend and a task-specific success predicate. No simulated
success has been claimed from CI.

Detailed [integration contract](docs/INTEGRATION.md) and [V0 milestones](docs/V0_PLAN.md).

```bash
pip install -e '.[dev]'
pytest -q
python -m multipath --help
```

References: [existing EgoEngine fork](https://github.com/wallacexuhanxiao/diversityverifier/tree/feat/egoengine-xhand-faithful)
(`098a28a5c9001eb6600897447132cea550e715fd`) and
[DexImit-Open](https://github.com/mujc2021/DexImit-Open)
(`c5749809fa425525a9bf7aa392dbb3f6268ae28f`).
