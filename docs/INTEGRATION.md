# Integration contract and current implementation boundary

The code here implements the V0 *control flow, state gates and a cuRobo bridge*. It does not claim a GPU PhysX rollout was run. Physics access is deliberately an injected `PhysicsWorld` adapter; a mock cannot provide the final result.

## Existing reference code

- `diversityverifier/scripts/run_egoengine_auto_pipeline.py` runs the current full reference pipeline (retarget → Replay/MPC/RL as needed); it does not already export independent, compatible local grasp/place anchors.
- `diversityverifier/scripts/export_execution_trace.py` exports actual hand/tool/target poses and velocities. It does **not** contain a fabricated mounted UR5e arm state or the full contact-state snapshot needed here.
- `diversityverifier/src/diversityverifier/isaac_execution.py` implements the existing `ContinuousExecutor`. Wire V0 against this actual environment after the arm/hand embodiment and measured state schema are verified. The exact action API and frame mapping must be checked against the installed vendored EgoEngine SHA.
- DexImit uses `setup_curobo_utils()`, `MotionGen.plan_single()` and `get_interpolated_plan()` in `pipeline/gen_traj.py`, but drives SAPIEN instead of Isaac.

## Required `module:factory` adapter

Implement `PhysicsWorld` from `multipath.orchestrator` against real Isaac:
`reset_initial()` (only full-run reset), `observe()` (return measured Boundary),
`execute_joint_commands()` (synchronous PhysX steps and actual joint targets),
`correct_to()` (real commanded short corrective motion; return False if unsupported),
`check_grasp()`, `check_placement()`, `no_hard_physics_failure()`,
`save_actual_trace()` (persist *all actual frames*, collision/contact channels and video).

The caller passes the real adapter factory to `multipath --world yourpackage.module:factory`. No fake `world` implementation is shipped as a real runtime.

### Boundary requirements

Every `Boundary` is measured and tagged by one scene ID; arm and hand have explicit joint names and q/dq; EE, held bowl, and grasp offsets use `xyz+xyzw`; velocities are measured (not assumed zero); `grasped` derives from independent contacts; anchor command arrays have columns in exactly the named arm/hand order. For the full rollout, all state comes from *actual simulator observations*. Never substitute the desired/canonical boundary for measured `observe()`.

### cuRobo object attachment — mandatory blocker

The current `CuRoboPlanner` intentionally refuses to plan until its attached-object hook has been verified with the exact locally installed cuRobo API. In the selected version, register the held bowl in the cuRobo collision world, invoke the supported attachment API and confirm the attachment before enabling `plan()`. The placeholder does not silently skip carried-object collision. Confirm transforms for the robot link, bowl geometry and planner vs simulator world coordinates. Test collisions in both systems.

Also confirm the real robot YAML and world YAML. DexImit UR5e/LEAP yaml cannot simply be used to drive existing floating Inspire/XHand EgoEngine PhysX hands.

### Example

```bash
pip install -e '.[dev,planning]'
# install matching Isaac Sim and cuRobo separately
multipath \
 --world my_isaac_adapter:create_world \
 --grasp-anchor /path/to/verified/grasp \
 --place-anchor /path/to/verified/put_in \
 --robot-yaml /path/to/verified_robot.yml \
 --world-yaml /path/to/verified_scene.yml \
 --attached-mesh /path/to/bowl.obj \
 --attached-pose-wxyz 0 0 0 1 0 0 0 \
 --output-dir outputs/put_in_v0
```

If a critical field (real arm joints, verified contact, attachment, actual trace, task predicate) is missing, *fail*, do not fill it with a placeholder and claim physical success.
