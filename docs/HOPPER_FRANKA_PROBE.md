# Hopper Franka Put-In probe (2026-09-29)

This is a new Isaac Sim scene with a mounted Franka Panda, table, dynamic cylinder
object, and tray. It is a physical integration probe for multipath V0, not a
verified Put-In trajectory. The upstream repository was pinned at
`fc2da7bf06932ce8cde0984843d40830ceeac209` before these local additions.
The original XHand BC is now retained as the hand controller. The initial
Franka two-finger grasp probes below tested arm integration only; they did not
execute the old XHand BC.

## Location and runtime

- Hopper checkout: `/scratch/e0985859/multipath_v0_fc2da7b`
- Queue: `interactive`; PBS project `CFP04-CF-057`; one H100 requested.
- Isaac Python: `/scratch/e0985859/egoengine_baseline/envs/sim-isaac510/bin/python`
- Container: `/app1/common/singularity-img/hopper/pytorch/pytorch_2.7.0_cuda_12.8.sif`
- cuRobo: v0.7.7, commit `0a50de1ba72db304195d59d9d0b1ed269696047f`, installed to checkout `deps/`.

Submit one job at a time from the Hopper checkout:

```bash
/cm/shared/apps/pbspro/current/bin/qsub hpc/run_franka_scene_smoke.pbs
/cm/shared/apps/pbspro/current/bin/qsub hpc/run_franka_curobo_motion.pbs
/cm/shared/apps/pbspro/current/bin/qsub hpc/run_franka_grasp_probe.pbs
```

The local source has the scripts in `scripts/`, the PBS files in `hpc/`, and
the planner's static obstacle geometry in `configs/franka_putin_proxy_world.yml`.
The cuRobo build and runtime dependencies are staged by the two install PBS
files. The mounted arm and cylinder are simulated by Isaac; the tray is a
physical collision object rather than a success label.

## Measured results

| Run | Result |
| --- | --- |
| Scene idle, 638083 | Franka articulation with 9 joints, dynamic cylinder settles on table through 240 PhysX steps; `outputs/franka_scene_smoke_r1/scene_smoke_result.json`. |
| cuRobo arm path, 638128 | 31-step planned arm path runs in Isaac. Isaac/cuRobo `panda_hand` FK position differs by 0.000000306 m at start; final maximum joint error 0.001475 rad; `outputs/franka_curobo_motion_r2/arm_motion_result.json`. |
| Grasp r2, 638147 | Oblique open-finger pregrasp pushed cylinder ~0.039 m before approach; lift failed. |
| Grasp r3, 638150 | Overhead pregrasp kept cylinder stable (<0.000001 m displacement). Approach left cylinder within 0.0008 m horizontally, but lift failed; cylinder rise ~0 m and hand-object offset drift 0.118 m. |
| Grasp r4, 638154 | Aiming 0.015 m farther in x/y still stopped above the object; approach max joint tracking error ~0.061 rad and lift failed. |
| Grasp r5, 638156 | A 0.04 m diameter proxy was pushed ~0.053 m during approach; no lift; `outputs/franka_grasp_probe_r5/grasp_probe_result.json`. |
| XHand wrist IK, 638199 | Both BC windows, 58/58 measured wrist poses, have Franka arm IK solutions with and without the current static cuboid world; candidate fixed mount and scene translation, no XHand collision geometry yet. `outputs/xhand_arm_ik_r2/xhand_arm_ik_result.json`. |
| Sequential wrist IK, 638235 | Previous-frame-seeded IK also solves 58/58 wrist poses. One selected static-world joint path still has a 0.310 rad adjacent-frame jump (9.29 rad/s at 30 fps); the first Franka joint's URDF velocity limit is 2.175 rad/s. This path cannot be replayed at original timing. `outputs/xhand_arm_ik_r4/xhand_arm_ik_result.json`. |
| XHand physical mount, 638213 | Original right XHand URDF joined to Franka hand via PhysX fixed joint; 21 total DOFs (7 arm + 2 unused Franka fingers + 12 XHand). Idle relative-position drift over 120 steps 2.88e-8 m. `outputs/mount_xhand_smoke_r4/mount_result.json`. |
| Mounted XHand finger BC, 638229 | Old 22-frame grasp-window finger commands mapped by exact joint names. Physically entering the first hand pose reduced first-frame maximum error to 0.096 rad; window maximum 0.550 rad, mean absolute 0.0455 rad. No wrist movement/object contact. `outputs/mount_xhand_smoke_r6/mount_result.json`. |
| Old finger gains on mounted chain, 638232 | Copying EgoEngine's 5.0/0.5/3.0 stiffness/damping/force into the combined articulation caused mean absolute command error 0.959 rad. This controller transplant failed; `outputs/mount_xhand_smoke_r8/mount_result.json`. |

All grasp attempts preserve actual PhysX state through approach, close, lift,
and hold. They do not reset the object after contact. `anchor_verified` and
`task_success` remain false. No physical Grasp or Put-In Anchor can be exported
from these traces.

The old XHand finger commands can be used with the mounted XHand; the old
floating-root commands become arm wrist targets. The 58/58 result is
pose reachability, not a speed-feasible arm trajectory. Sequential seeding
reduced but did not eliminate joint jumps; arm trajectory optimization and
possibly time scaling remain necessary. The idle mount
result proves the fixed joint is stable, not that grasp contact survives
motion. Named-joint XHand finger BC has now executed on the mounted chain, but
the imported controller does not yet reproduce the old actual finger trace.
The required next experiment is time-continuous arm wrist tracking plus real
object contact while executing the same XHand finger BC; then cuRobo can replace every relaxed
interval. The real `PhysicsWorld` adapter, task predicate, and uninterrupted
full rollout remain to be implemented.
