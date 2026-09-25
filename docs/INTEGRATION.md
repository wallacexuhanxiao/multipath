# V0 integration contract

This repo now implements `MotionGen.plan_single_js` with canonical joint goals
and `attach_external_objects_to_robot()` using the real observed object pose.
The configured cuRobo robot **must** declare the dedicated `attachment_link`
with sufficient preallocated collision spheres. The observed bowl mesh and
world scene must use matching coordinate frames and scale. See:
https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html

The remaining required step is a *real* Isaac Sim embodiment and runtime adapter.
The upstream EgoEngine run exposes floating hand roots and finger joint
positions, NOT an already mounted UR5e arm. Copying DexImit's UR5e configuration
without mounting the same arm/hand in Isaac gives invalid joint observations.
Do not fabricate arm DOFs from floating wrist poses.

Source implementations:

- EgoEngine: https://github.com/wallacexuhanxiao/diversityverifier/tree/feat/egoengine-xhand-faithful
- `scripts/run_egoengine_auto_pipeline.py`: reference retarget/refine, not automatically standalone compatible Anchors.
- `scripts/export_execution_trace.py`: actual hand/tool/target poses, lacking mounted arm joint telemetry.
- `src/diversityverifier/isaac_execution.py`: existing real `ContinuousExecutor`; extend only after the mounted robot, joint commands and collider mapping are audited.
- DexImit `third_party/any2dex/any2dex/util/curobo_util.py` and `pipeline/gen_traj.py`: cuRobo/SAPIEN planning pattern.

The real `PhysicsWorld` adapter must implement all methods in
`multipath.orchestrator.PhysicsWorld`, with exactly **one** call to
`reset_initial` at full-run start. Afterward, `observe()` must return actual
PhysX states, `execute_joint_commands()` must simulate each timestep and
record controls/states/contacts, `correct_to()` may execute *real* servo
motions but never write states, `check_grasp()` and `check_placement()`
must inspect contacts and task geometry, and `save_actual_trace()` must
persist full actual-state data and video. Missing evidence is failure.

`Anchor.save()` / `Anchor.load()` use `anchor.json` plus `commands.npz`.
Do not label independently synthesized Anchor slices `verified` unless
their respective measured local PhysX rollouts passed. Grasp exit and
Put-In entry must be jointly compatible under `static_anchor_compatibility()`.

Example once the embodiment and adapters exist:

```bash
pip install -e '.[dev,planning]'
multipath \
 --world my_isaac_adapter:create_world \
 --grasp-anchor /path/to/measured/grasp \
 --place-anchor /path/to/measured/put_in \
 --robot-yaml /path/to/matched_robot.yml \
 --world-yaml /path/to/matched_scene.yml \
 --attached-mesh /path/to/bowl.obj \
 --attachment-link attached_object \
 --output-dir outputs/put_in_v0
```

Offline CI mocks test the orchestrator, not Isaac, contact validity,
cuRobo GPU kernels, or any particular physically verified sequence.
