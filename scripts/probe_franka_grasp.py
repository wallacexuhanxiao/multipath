"""Probe a Franka grasp/lift on the dynamic bowl proxy with measured PhysX state.

This run uses one reset at the start. It does not export a verified Anchor until
contact evidence and repeated full-task rollouts are available.
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
    trace: list[dict] = []
    try:
        import torch
        import yaml
        from curobo.types.base import TensorDeviceType
        from curobo.types.math import Pose
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
            raise RuntimeError("cuRobo requires CUDA")
        assets = get_assets_root_path()
        if assets is None:
            raise RuntimeError("Franka USD asset unavailable")
        world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 30)
        world.scene.add_default_ground_plane()
        add_reference_to_stage(
            assets + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd", "/World/Franka")
        robot = world.scene.add(Robot(prim_path="/World/Franka", name="franka"))
        for name, item in yaml.safe_load(args.world_yaml.read_text())["cuboid"].items():
            world.scene.add(FixedCuboid(
                prim_path=f"/World/{name}", name=name,
                position=np.asarray(item["pose"][:3], dtype=float),
                scale=np.asarray(item["dims"], dtype=float),
            ))
        bowl = world.scene.add(DynamicCylinder(
            prim_path="/World/BowlProxy", name="bowl_proxy",
            position=np.array([0.52, -0.18, 0.39]),
            radius=0.02, height=0.07, mass=0.08,
        ))
        world.reset()
        hand = SingleXFormPrim(prim_path="/World/Franka/panda_hand", name="hand_probe")
        names = list(robot.dof_names)
        if names[:7] != [f"panda_joint{i}" for i in range(1, 8)]:
            raise RuntimeError(f"unexpected Franka arm joints: {names}")

        def sample(phase: str) -> None:
            hand_pos, hand_quat = hand.get_world_pose()
            bowl_pos, bowl_quat = bowl.get_world_pose()
            trace.append({
                "phase": phase, "frame": len(trace),
                "joint_q": np.asarray(robot.get_joint_positions(), dtype=float).tolist(),
                "joint_dq": np.asarray(robot.get_joint_velocities(), dtype=float).tolist(),
                "hand_position": np.asarray(hand_pos, dtype=float).tolist(),
                "hand_quaternion_wxyz": np.asarray(hand_quat, dtype=float).tolist(),
                "bowl_position": np.asarray(bowl_pos, dtype=float).tolist(),
                "bowl_quaternion_wxyz": np.asarray(bowl_quat, dtype=float).tolist(),
            })

        def step_n(count: int, phase: str) -> None:
            for _ in range(count):
                world.step(render=False)
                sample(phase)

        robot.apply_action(ArticulationAction(
            joint_positions=np.array([0.04, 0.04]), joint_indices=np.array([7, 8])))
        step_n(120, "settle_open")
        q0 = np.asarray(robot.get_joint_positions(), dtype=float)
        # Gripper local +Z points down, so the two fingers approach from above.
        orientation = np.array([0.0, 1.0, 0.0, 0.0], dtype=float)
        settled_bowl = np.asarray(bowl.get_world_pose()[0], dtype=float)
        planner = MotionGen(MotionGenConfig.load_from_robot_config(
            str(args.robot_yaml), str(args.world_yaml),
            tensor_args=TensorDeviceType(device="cuda:0"),
            interpolation_dt=1 / 30, self_collision_check=True,
        ))
        planner.warmup()
        if list(planner.joint_names) != names[:7]:
            raise RuntimeError("cuRobo and Isaac arm joint order differ")

        def plan_and_execute(position: np.ndarray, phase: str, finger_target: float) -> None:
            actual_q = np.asarray(robot.get_joint_positions(), dtype=float)
            js = JointState.from_position(
                torch.as_tensor(actual_q[:7], dtype=torch.float32, device="cuda:0")[None],
                joint_names=list(planner.joint_names),
            )
            pose = Pose(
                position=torch.as_tensor(position, dtype=torch.float32, device="cuda:0")[None],
                quaternion=torch.as_tensor(orientation, dtype=torch.float32, device="cuda:0")[None],
            )
            planned = planner.plan_single(js, pose, MotionGenPlanConfig(
                max_attempts=30, enable_graph_attempt=5, check_start_validity=True,
            ))
            if not bool(planned.success.item() if hasattr(planned.success, "item") else planned.success):
                raise RuntimeError(f"{phase} cuRobo planning failed: {getattr(planned, 'status', None)}")
            path = planned.get_interpolated_plan().position.detach().cpu().numpy()
            if path.ndim == 3:
                path = path[0]
            result[phase + "_planned_final_q"] = path[-1].tolist()
            dt = float(planned.interpolation_dt.item() if hasattr(planned.interpolation_dt, "item")
                       else planned.interpolation_dt)
            for arm_q in path:
                robot.apply_action(ArticulationAction(
                    joint_positions=np.r_[arm_q, finger_target, finger_target]))
                step_n(max(1, round(dt * 120)), phase)
            step_n(30, phase + "_settle")
            result[phase + "_plan_steps"] = len(path)
            result[phase + "_hand_position"] = list(map(float, hand.get_world_pose()[0]))
            result[phase + "_actual_final_q"] = np.asarray(robot.get_joint_positions(), dtype=float)[:7].tolist()
            result[phase + "_max_q_error_rad"] = float(np.max(np.abs(
                np.asarray(robot.get_joint_positions(), dtype=float)[:7] - path[-1])))
            result[phase + "_bowl_displacement_m"] = float(np.linalg.norm(
                np.asarray(bowl.get_world_pose()[0], dtype=float)[:2] - settled_bowl[:2]))

        plan_and_execute(np.array([0.39, 0.00, 0.65]), "clearance", 0.04)
        plan_and_execute(np.array([0.52, -0.18, 0.62]), "pregrasp", 0.04)
        if result["pregrasp_bowl_displacement_m"] > 0.01:
            raise RuntimeError("pregrasp moved the bowl proxy before contact was intended")
        approach_attempts = []
        for goal in (
            [0.535, -0.155, 0.49],
            [0.535, -0.165, 0.49],
            [0.52, -0.18, 0.49],
            [0.52, -0.18, 0.50],
            [0.52, -0.18, 0.51],
        ):
            try:
                plan_and_execute(np.array(goal), "approach", 0.04)
                result["chosen_approach_goal"] = goal
                break
            except RuntimeError as exc:
                if "approach cuRobo planning failed" not in str(exc):
                    raise
                approach_attempts.append({"goal": goal, "error": str(exc)})
        else:
            raise RuntimeError("all nearby approach poses failed cuRobo planning")
        result["approach_attempts"] = approach_attempts
        before_close, _ = bowl.get_world_pose()
        robot.apply_action(ArticulationAction(
            joint_positions=np.array([0.012, 0.012]), joint_indices=np.array([7, 8])))
        step_n(120, "close_hold")
        pre_lift_bowl, _ = bowl.get_world_pose()
        pre_lift_hand, _ = hand.get_world_pose()
        actual_fingers = np.asarray(robot.get_joint_positions(), dtype=float)[7:]
        result.update({
            "bowl_before_close": np.asarray(before_close, dtype=float).tolist(),
            "bowl_before_lift": np.asarray(pre_lift_bowl, dtype=float).tolist(),
            "finger_q_after_close": actual_fingers.tolist(),
        })
        lift_goal = np.asarray(pre_lift_hand, dtype=float) + [0.0, 0.0, 0.12]
        plan_and_execute(lift_goal, "lift", float(np.mean(actual_fingers)))
        step_n(120, "post_lift_hold")
        final_bowl, _ = bowl.get_world_pose()
        final_hand, _ = hand.get_world_pose()
        rise = float(final_bowl[2] - pre_lift_bowl[2])
        held_offset_drift = float(np.linalg.norm(
            (final_bowl - final_hand) - (pre_lift_bowl - pre_lift_hand)))
        result.update({
            "status": "grasp_probe_complete", "bowl_rise_m": rise,
            "held_offset_drift_m": held_offset_drift,
            "bowl_final_position": np.asarray(final_bowl, dtype=float).tolist(),
            "hand_final_position": np.asarray(final_hand, dtype=float).tolist(),
            "load_bearing_candidate": bool(rise > 0.05 and held_offset_drift < 0.03),
            "note": "No object state reset; contact sensor validation is still required.",
        })
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        (args.output_dir / "grasp_probe_result.json").write_text(json.dumps(result, indent=2) + "\n")
        (args.output_dir / "grasp_probe_trace.json").write_text(json.dumps(trace) + "\n")
        app.close()


if __name__ == "__main__":
    main()
