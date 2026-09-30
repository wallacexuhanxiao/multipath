"""Extract video-aligned BC references for EgoEngine-style local synthesis.

The XHand finger arrays can seed the same XHand after arm mounting. The floating
root poses must become arm wrist targets and be revalidated in coupled physics.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from multipath.schedule import Schedule
from multipath.xhand import RIGHT_XHAND_FINGER_JOINT_NAMES


REFERENCE_KEYS = (
    "qpos_left_reference", "qpos_right_reference",
    "object_pose_gt_tool", "object_pose_gt_target",
)
ACTION_KEYS = ("qpos_left", "qpos_right")
OBSERVED_KEYS = (
    "qpos_left_actual", "qpos_right_actual",
    "eef_pose_left_xyz_quat_wxyz", "eef_pose_right_xyz_quat_wxyz",
    "object_pose_actual_tool_xyzw", "object_pose_actual_target_xyzw",
    "opposition_contact_proxy", "thumb_contact_proxy",
    "other_finger_contact_proxy",
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--schedule", type=Path, required=True)
    parser.add_argument("--egoengine-trajectory", type=Path, required=True)
    parser.add_argument("--arm-mount-result", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    schedule = Schedule.load(args.schedule)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    mount_verified = bool(args.arm_mount_result and json.loads(
        args.arm_mount_result.read_text()).get("physical_xhand_mount_verified"))
    manifest = {
        "schedule": str(args.schedule.resolve()),
        "source_trajectory": str(args.egoengine_trajectory.resolve()),
        "source_embodiment": "floating XHand roots, two 19-DOF hands",
        "target_embodiment": "same XHand fingers on a mounted arm; arm model under test",
        "status": ("xhand_finger_bc_available_arm_mount_idle_verified"
                   if mount_verified else "xhand_finger_bc_available_arm_mount_unverified"),
        "arm_mount_result": str(args.arm_mount_result.resolve()) if args.arm_mount_result else None,
        "right_xhand_finger_joint_names_in_command_order": list(RIGHT_XHAND_FINGER_JOINT_NAMES),
        "windows": [],
    }
    with np.load(args.egoengine_trajectory, allow_pickle=False) as source:
        missing = set(REFERENCE_KEYS + ACTION_KEYS + OBSERVED_KEYS) - set(source.files)
        if missing:
            raise ValueError(f"missing EgoEngine reference fields: {sorted(missing)}")
        for key in REFERENCE_KEYS + ACTION_KEYS + OBSERVED_KEYS:
            if len(source[key]) != schedule.num_frames:
                raise ValueError(f"{key}: frame count mismatches VLM schedule")
        for w in schedule.bc_windows:
            name = f"bc_{w.start}_{w.end}"
            out = args.output_dir / f"{name}.npz"
            arrays = {
                key: source[key][w.start:w.end].copy()
                for key in REFERENCE_KEYS + ACTION_KEYS + OBSERVED_KEYS
            }
            arrays["source_frame_indices"] = np.arange(w.start, w.end)
            np.savez_compressed(out, **arrays)
            manifest["windows"].append({
                "start": w.start, "end": w.end,
                "reference_npz": out.name,
                "xhand_finger_actions": "available_from_source",
                "arm_wrist_ik_and_tracking": "pending",
                "local_mpc_rl_if_coupled_replay_fails": "pending",
                "physical_bc_validation": "pending",
            })
    (args.output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
