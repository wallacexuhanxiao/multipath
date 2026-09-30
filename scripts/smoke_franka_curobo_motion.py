"""Plan and physically execute one short Franka joint motion in the proxy scene.

This checks embodiment and controller tracking. The bowl stays on the table;
this is not a grasp or Put-In success test.
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
    parser.add_argument("--robot-yaml", type=Path, required=True)
    parser.add_argument("--world-yaml", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    app = SimulationApp({"headless": True})
    result = {"status": "failed", "task_success": False, "anchor_verified": False}
    try:
        import torch
        import yaml
        from curobo.types.base import TensorDeviceType
        from curobo.types.robot import JointState
        from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import DynamicCylinder, FixedCuboid
        from isaacsim.core.api.robots import Robot
        from isaacsim.core.prims import SingleXFormPrim
        from isaacsim.core.utils.nucleus import get_assets_root_path
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.utils.types import ArticulationAction

        if not torch.cuda.is_available():
            raise RuntimeError("cuRobo requires a visible CUDA GPU")
        asset_root = get_assets_root_path()
        if asset_root is None:
            raise RuntimeError("Franka asset root unavailable")
        robot_usd = asset_root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 30)
        world.scene.add_default_ground_plane()
        add_reference_to_stage(robot_usd, "/World/Franka")
        robot = world.scene.add(Robot(prim_path="/World/Franka", name="franka"))
        boxes = yaml.safe_load(args.world_yaml.read_text())["cuboid"]
        for name, item in boxes.items():
            world.scene.add(FixedCuboid(
                prim_path=f"/World/{name}", name=name,
                position=np.asarray(item["pose"][:3], dtype=float),
                scale=np.asarray(item["dims"], dtype=float),
            ))
        bowl = world.scene.add(DynamicCylinder(
            prim_path="/World/BowlProxy", name="bowl_proxy",
            position=np.array([0.52, -0.18, 0.39]),
            radius=0.03, height=0.07, mass=0.15,
        ))
        world.reset()
        names = list(robot.dof_names)
        if names != [*(f"panda_joint{i}" for i in range(1, 8)),
                     "panda_finger_joint1", "panda_finger_joint2"]:
            raise RuntimeError(f"unexpected Isaac joint order: {names}")
        # The bundled cuRobo Franka model locks both gripper DOFs open.
        robot.apply_action(ArticulationAction(
            joint_positions=np.array([0.04, 0.04]), joint_indices=np.array([7, 8])))
        for _ in range(120):
            world.step(render=False)
        q0 = np.asarray(robot.get_joint_positions(), dtype=float)
        if np.max(np.abs(q0[7:] - 0.04)) > 0.005:
            raise RuntimeError(f"gripper failed to open: {q0[7:].tolist()}")

        tensor = TensorDeviceType(device="cuda:0")
        cfg = MotionGenConfig.load_from_robot_config(
            str(args.robot_yaml), str(args.world_yaml), tensor_args=tensor,
            interpolation_dt=1 / 30, self_collision_check=True,
        )
        planner = MotionGen(cfg)
        planner.warmup()
        plan_names = list(planner.joint_names)
        if plan_names != names[:7]:
            raise RuntimeError(f"Isaac/cuRobo joint mismatch: {names} vs {plan_names}")
        hand_paths = [str(prim.GetPath()) for prim in world.stage.Traverse()
                      if prim.GetName() == "panda_hand"]
        if len(hand_paths) != 1:
            raise RuntimeError(f"expected one panda_hand link, got {hand_paths}")
        hand_prim = SingleXFormPrim(prim_path=hand_paths[0], name="hand_pose_probe")
        actual_hand_pos, actual_hand_quat = hand_prim.get_world_pose()
        kin = planner.compute_kinematics(JointState.from_position(
            torch.as_tensor(q0[:7], dtype=torch.float32, device="cuda:0")[None],
            joint_names=plan_names,
        ))
        predicted_hand_pos = kin.ee_pose.position.detach().cpu().numpy().reshape(-1, 3)[0]
        predicted_hand_quat = kin.ee_pose.quaternion.detach().cpu().numpy().reshape(-1, 4)[0]
        fk_position_error = float(np.linalg.norm(actual_hand_pos - predicted_hand_pos))
        fk_rotation_error = float(2 * np.arccos(np.clip(
            abs(np.dot(actual_hand_quat, predicted_hand_quat)), 0, 1)))
        result.update({
            "isaac_hand_path": hand_paths[0],
            "isaac_hand_pose_wxyz": [*np.asarray(actual_hand_pos).tolist(),
                                      *np.asarray(actual_hand_quat).tolist()],
            "curobo_hand_pose_wxyz": [*predicted_hand_pos.tolist(),
                                       *predicted_hand_quat.tolist()],
            "fk_position_error_m": fk_position_error,
            "fk_rotation_error_rad": fk_rotation_error,
        })
        if fk_position_error > 0.01 or fk_rotation_error > 0.10:
            raise RuntimeError("Isaac/cuRobo hand FK mismatch")
        target = q0[:7].copy()
        target[0] += 0.15
        def js(q: np.ndarray) -> JointState:
            return JointState.from_position(
                torch.as_tensor(q, dtype=torch.float32, device="cuda:0")[None],
                joint_names=plan_names,
            )
        plan = planner.plan_single_js(
            js(q0[:7]), js(target), MotionGenPlanConfig(
                max_attempts=30, enable_graph_attempt=5,
                check_start_validity=True,
            ))
        if not bool(plan.success.item() if hasattr(plan.success, "item") else plan.success):
            raise RuntimeError(f"cuRobo joint-space plan failed: {plan.status}")
        trajectory = plan.get_interpolated_plan()
        path = trajectory.position.detach().cpu().numpy()
        if path.ndim == 3:
            path = path[0]
        if path.ndim != 2 or path.shape[1] != 7 or len(path) < 2:
            raise RuntimeError(f"unexpected planned path shape {path.shape}")
        dt = float(plan.interpolation_dt.item() if hasattr(plan.interpolation_dt, "item")
                   else plan.interpolation_dt)
        substeps = max(1, round(dt * 120))
        trace = []
        for index, arm_q in enumerate(path):
            command = np.r_[arm_q, 0.04, 0.04]
            robot.apply_action(ArticulationAction(joint_positions=command))
            for _ in range(substeps):
                world.step(render=False)
            actual = np.asarray(robot.get_joint_positions(), dtype=float)
            bowl_pos, _ = bowl.get_world_pose()
            trace.append({
                "index": index, "command_q": command.tolist(),
                "actual_q": actual.tolist(),
                "bowl_proxy_position": np.asarray(bowl_pos, dtype=float).tolist(),
            })
        # Let the controller settle at the final command without writing a state.
        for _ in range(60):
            world.step(render=False)
        final_q = np.asarray(robot.get_joint_positions(), dtype=float)
        final_error = float(np.max(np.abs(final_q[:7] - target)))
        bowl_final, _ = bowl.get_world_pose()
        bowl_drift = float(np.linalg.norm(np.asarray(bowl_final) - [0.52, -0.18, 0.385]))
        if final_error > 0.08 or bowl_drift > 0.02:
            raise RuntimeError(f"physical tracking/scene failed: arm={final_error}, bowl={bowl_drift}")
        result.update({
            "status": "arm_motion_passed", "robot_joint_names": names,
            "planner_joint_names": plan_names, "plan_steps": len(path),
            "plan_dt": dt, "final_arm_error_rad": final_error,
            "bowl_proxy_drift_m": bowl_drift,
            "note": "Actual PhysX arm motion only; grasp and Put-In not established.",
        })
        (args.output_dir / "arm_motion_trace.json").write_text(json.dumps(trace) + "\n")
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        (args.output_dir / "arm_motion_result.json").write_text(json.dumps(result, indent=2) + "\n")
        app.close()


if __name__ == "__main__":
    main()
