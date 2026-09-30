"""Create a physically simulated Franka arm with a small Put-In proxy scene.

This is an embodiment smoke test. It does not claim a grasp, an Anchor, or task
success. The bowl is a cylinder until the TACO mesh and its collision model are
validated in this mounted-arm scene.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--franka-usd", default=None)
    parser.add_argument("--steps", type=int, default=240)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    app = SimulationApp({"headless": True})
    result = {"status": "failed", "task_success": False, "anchor_verified": False}
    try:
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import DynamicCylinder, FixedCuboid
        from isaacsim.core.api.robots import Robot
        from isaacsim.core.utils.nucleus import get_assets_root_path
        from isaacsim.core.utils.stage import add_reference_to_stage
        from pxr import Usd

        asset_root = get_assets_root_path() if args.franka_usd is None else None
        if args.franka_usd is None and asset_root is None:
            raise RuntimeError("Isaac Sim asset root unavailable; pass --franka-usd")
        robot_usd = args.franka_usd or (
            asset_root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        )
        world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 30)
        world.scene.add_default_ground_plane()
        add_reference_to_stage(robot_usd, "/World/Franka")
        robot = world.scene.add(Robot(prim_path="/World/Franka", name="franka"))
        table = world.scene.add(FixedCuboid(
            prim_path="/World/Table", name="table",
            position=np.array([0.60, 0.0, 0.175]),
            scale=np.array([0.80, 0.75, 0.35]),
            color=np.array([0.55, 0.55, 0.55]),
        ))
        bowl = world.scene.add(DynamicCylinder(
            prim_path="/World/BowlProxy", name="bowl_proxy",
            position=np.array([0.52, -0.18, 0.39]),
            radius=0.03, height=0.07, mass=0.15,
            color=np.array([0.9, 0.2, 0.2]),
        ))
        tray_center = np.array([0.60, 0.20, 0.365])
        world.scene.add(FixedCuboid(
            prim_path="/World/TrayBase", name="tray_base", position=tray_center,
            scale=np.array([0.20, 0.20, 0.03]), color=np.array([0.2, 0.3, 0.8]),
        ))
        for name, pos, size in (
            ("front", [0.60, 0.29, 0.415], [0.20, 0.02, 0.08]),
            ("back", [0.60, 0.11, 0.415], [0.20, 0.02, 0.08]),
            ("left", [0.51, 0.20, 0.415], [0.02, 0.16, 0.08]),
            ("right", [0.69, 0.20, 0.415], [0.02, 0.16, 0.08]),
        ):
            world.scene.add(FixedCuboid(
                prim_path=f"/World/TrayWall_{name}", name=f"tray_wall_{name}",
                position=np.array(pos), scale=np.array(size),
                color=np.array([0.2, 0.3, 0.8]),
            ))
        world.reset()
        joint_names = list(robot.dof_names)
        if len(joint_names) < 9 or not all(f"panda_joint{i}" in joint_names for i in range(1, 8)):
            raise RuntimeError(f"unexpected Franka DOFs: {joint_names}")
        trace = []
        for step in range(args.steps):
            world.step(render=False)
            if step % 10 == 0 or step == args.steps - 1:
                q = np.asarray(robot.get_joint_positions(), dtype=float)
                dq = np.asarray(robot.get_joint_velocities(), dtype=float)
                pos, quat = bowl.get_world_pose()
                trace.append({
                    "step": step, "arm_and_gripper_q": q.tolist(),
                    "arm_and_gripper_dq": dq.tolist(),
                    "bowl_proxy_position": np.asarray(pos, dtype=float).tolist(),
                    "bowl_proxy_quaternion_wxyz": np.asarray(quat, dtype=float).tolist(),
                })
                if not (np.isfinite(q).all() and np.isfinite(dq).all() and np.isfinite(pos).all()):
                    raise RuntimeError("nonfinite measured PhysX state")
        stage = world.stage
        usd_path = args.output_dir / "franka_putin_proxy.usda"
        stage.GetRootLayer().Export(str(usd_path))
        result.update({
            "status": "scene_smoke_passed", "robot_usd": robot_usd,
            "robot_joint_names": joint_names, "steps": args.steps,
            "bowl_proxy_final_position": trace[-1]["bowl_proxy_position"],
            "scene_usd": str(usd_path), "geometry": "proxy_cylinder_and_tray",
            "note": "Measured idle PhysX scene only; no grasp, planning, or Put-In success tested.",
        })
        (args.output_dir / "idle_trace.json").write_text(json.dumps(trace, indent=2) + "\n")
        del table
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        (args.output_dir / "scene_smoke_result.json").write_text(json.dumps(result, indent=2) + "\n")
        app.close()


if __name__ == "__main__":
    main()
