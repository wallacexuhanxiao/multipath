# multipath — Interaction-Anchored Trajectory Generation

V0: produce **one** continuous, physics-verified Put-In trajectory from a single ego video, preserving local grasp/placement interactions and replacing only the relaxed transport with cuRobo. Multi-path search comes **after** V0 succeeds.

**First target:** TACO `(put in, bowl, plate) / 20231024_253`.

```
Video / digital twin
  -> manually selected grasp / put-in anchor intervals
  -> EgoEngine-style local anchor synthesis + actual-state verification
  -> compatible measured canonical boundary states
  -> cuRobo arm planning with the object modeled as attached
  -> real PhysX transport from grasp-exit state
  -> observed transition check / real corrective motion if needed
  -> place anchor -> uninterrupted full-task PhysX rollout
  -> actual-state NPZ, metrics JSON, video
```

## Source of truth

- EgoEngine base: [diversityverifier, feat/egoengine-xhand-faithful](https://github.com/wallacexuhanxiao/diversityverifier/tree/feat/egoengine-xhand-faithful), inspected at `098a28a5c9001eb6600897447132cea550e715fd`.
- Planning reference: [DexImit-Open](https://github.com/mujc2021/DexImit-Open), inspected at `c5749809fa425525a9bf7aa392dbb3f6268ae28f`; particularly `pipeline/gen_traj.py` and `third_party/any2dex/any2dex/util/curobo_util.py`.
- Reuse **interfaces and techniques**, not DexImit's SAPIEN simulator or UR5e/LEAP calibration blindly.

## What's implemented

- Strict, named-joint boundary-state and anchor serialization/validation.
- Concrete cuRobo `MotionGen.plan_single` adapter (lazy dependency), controlled collision-world loading, commanded hand hold, and plan acceptance against a canonical goal.
- Single-path orchestrator with real-state transition checks, physically executed correction hook, uninterrupted whole-task replay requirement, task-success and provenance gates.
- Runnable `multipath` CLI, mock end-to-end tests and CI.
- JSON manifest for the first Put-In task. **No frame ranges or successful anchors are fabricated.**

## Runtime requirements

```bash
pip install -e '.[dev]'
pytest -q
python -m multipath --help
```

cuRobo and Isaac Sim must be installed in an existing compatible GPU/Isaac environment. The CLI requires a real `module:factory` physics adapter and actual locally verified anchor files. It does **not** claim that unit tests establish a successful PhysX experiment. Details: [integration instructions](docs/INTEGRATION.md), [V0 milestones](docs/V0_PLAN.md).

**Critical embodiment blocker:** existing EgoEngine execution reports floating hand roots + finger DOFs. DexImit cuRobo plans UR5e **arm** joints. These are not automatically the same robot. Before real execution, either mount the selected hand on a matched arm in the Isaac digital twin and revalidate both anchors, or supply a cuRobo robot model identical to the actual simulated robot. A fabricated arm state is not acceptable.
