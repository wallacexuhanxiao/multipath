"""Kinematic feasibility of retaining XHand BC with a mounted arm.

This checks a candidate fixed mount and scene translation. It does not assert
that XHand is already welded to the arm or that the old grasp survives it.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bc-dir", type=Path, required=True)
    parser.add_argument("--robot-yaml", type=Path, required=True)
    parser.add_argument("--world-yaml", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    import torch
    import yaml
    from scipy.spatial.transform import Rotation
    from curobo.geom.sdf.world import CollisionCheckerType
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

    records = []
    for path in sorted(args.bc_dir.glob("bc_*.npz")):
        with np.load(path, allow_pickle=False) as data:
            for frame, root, obj in zip(
                data["source_frame_indices"], data["qpos_right_actual"],
                data["object_pose_actual_tool_xyzw"], strict=True,
            ):
                records.append((int(frame), root[:7].astype(float), obj[:3].astype(float)))
    records.sort(key=lambda item: item[0])
    if not records:
        raise ValueError("no BC wrist frames found")

    # Provisional transform: align the first BC's observed bowl with the new
    # scene's bowl. Orient the fixed mount so that the first BC wrist matches
    # the Franka home panda_hand orientation measured in the arm smoke test.
    first_xhand = records[0][1]
    scene_bowl = np.array([0.52, -0.18, 0.385], dtype=float)
    translation = scene_bowl - records[0][2]
    panda_home_wxyz = np.array([0.0062855, 0.9212930, 0.0259865, 0.3879491])

    def rotation_from_wxyz(q):
        return Rotation.from_quat([q[1], q[2], q[3], q[0]])

    mount = rotation_from_wxyz(panda_home_wxyz).inv() * rotation_from_wxyz(first_xhand[3:7])
    goals_xyz = np.stack([root[:3] + translation for _, root, _ in records])
    goals_wxyz = []
    for _, root, _ in records:
        xyzw = (rotation_from_wxyz(root[3:7]) * mount.inv()).as_quat()
        goals_wxyz.append(xyzw[[3, 0, 1, 2]])
    goals_wxyz = np.asarray(goals_wxyz)
    goal = Pose(
        position=torch.as_tensor(goals_xyz, dtype=torch.float32, device="cuda:0"),
        quaternion=torch.as_tensor(goals_wxyz, dtype=torch.float32, device="cuda:0"),
    )

    report = {
        "status": "ik_probe_only",
        "frames": [frame for frame, _, _ in records],
        "scene_translation_xyz": translation.tolist(),
        "candidate_mount_quaternion_xyzw": mount.as_quat().tolist(),
        "xhand_finger_bc_changed": False,
        "arm_is_physically_mounted": False,
        "xhand_collision_geometry_in_ik": False,
    }
    tensor_args = TensorDeviceType(device="cuda:0")
    for label, world_model in (
        ("kinematics", None),
        ("static_world", yaml.safe_load(args.world_yaml.read_text())),
    ):
        config = IKSolverConfig.load_from_robot_config(
            str(args.robot_yaml), world_model,
            tensor_args=tensor_args, num_seeds=40,
            position_threshold=0.005, rotation_threshold=0.05,
            self_collision_check=True, self_collision_opt=True,
            collision_checker_type=CollisionCheckerType.PRIMITIVE,
            use_cuda_graph=False,
        )
        solver = IKSolver(config)
        result = solver.solve_batch(goal)
        success = result.success.detach().cpu().numpy().reshape(-1).astype(bool)
        position_error = result.position_error.detach().cpu().numpy().reshape(-1)
        rotation_error = result.rotation_error.detach().cpu().numpy().reshape(-1)
        q_solution = result.solution.detach().cpu().numpy().reshape(len(success), -1)
        contiguous = np.diff(np.asarray(report["frames"])) == 1
        joint_delta = np.abs(np.diff(q_solution, axis=0))[contiguous]
        report[label] = {
            "success_count": int(success.sum()),
            "frame_count": len(success),
            "failed_frames": [records[i][0] for i in np.flatnonzero(~success)],
            "max_position_error_m": float(position_error.max()),
            "max_rotation_error_rad": float(rotation_error.max()),
            "max_contiguous_joint_step_rad": float(joint_delta.max()),
            "max_contiguous_joint_speed_rad_s_at_30fps": float(joint_delta.max() * 30),
        }
        np.savez_compressed(args.output_dir / f"{label}_ik_solutions.npz",
                            frames=np.asarray(report["frames"]), arm_q=q_solution,
                            success=success, goals_xyz=goals_xyz,
                            goals_quat_wxyz=goals_wxyz)
        sequential_q = []
        sequential_success = []
        previous = None
        for i, frame in enumerate(report["frames"]):
            if i == 0 or frame != report["frames"][i - 1] + 1:
                previous = torch.as_tensor(q_solution[i], dtype=torch.float32, device="cuda:0")
            one_goal = Pose(position=goal.position[i:i + 1],
                            quaternion=goal.quaternion[i:i + 1])
            one = solver.solve_single(
                one_goal, retract_config=previous[None],
                seed_config=previous[None, None], num_seeds=10,
            )
            ok = bool(one.success.item())
            sequential_success.append(ok)
            if ok:
                previous = one.solution.detach().reshape(-1)
            sequential_q.append(previous.detach().cpu().numpy().copy())
        sequential_q = np.stack(sequential_q)
        seq_delta = np.abs(np.diff(sequential_q, axis=0))[contiguous]
        report[label]["sequential"] = {
            "success_count": int(sum(sequential_success)),
            "frame_count": len(sequential_success),
            "failed_frames": [report["frames"][i] for i, ok in enumerate(sequential_success) if not ok],
            "max_contiguous_joint_step_rad": float(seq_delta.max()),
            "max_contiguous_joint_speed_rad_s_at_30fps": float(seq_delta.max() * 30),
        }
        np.savez_compressed(args.output_dir / f"{label}_sequential_ik.npz",
                            frames=np.asarray(report["frames"]), arm_q=sequential_q,
                            success=np.asarray(sequential_success))
    (args.output_dir / "xhand_arm_ik_result.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
