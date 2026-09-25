"""Measured state compatibility; never alter simulator state to pass a check."""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from .schema import Boundary


@dataclass(frozen=True)
class Tolerance:
    arm_rad: float = 0.04
    hand_rad: float = 0.08
    object_m: float = 0.025
    object_rad: float = 0.12
    grasp_m: float = 0.015
    grasp_rad: float = 0.12
    max_arm_speed_rad_s: float = 0.10
    max_hand_speed_rad_s: float = 0.12
    max_object_speed_m_s: float = 0.06


def quat_angle(a: np.ndarray, b: np.ndarray) -> float:
    return 2 * float(np.arccos(np.clip(abs(np.dot(a, b)), 0, 1)))


def check(actual: Boundary, expected: Boundary, tol: Tolerance = Tolerance()) -> dict:
    if actual.arm_joint_names != expected.arm_joint_names or actual.hand_joint_names != expected.hand_joint_names:
        raise ValueError("joint order mismatch")
    if actual.scene_id != expected.scene_id:
        raise ValueError("scene mismatch; cannot join anchors")
    errors = {
        "arm_q_rad": float(np.max(np.abs(actual.arm_q - expected.arm_q))),
        "hand_q_rad": float(np.max(np.abs(actual.hand_q - expected.hand_q))) if len(actual.hand_q) else 0.,
        "object_m": float(np.linalg.norm(actual.object_pose[:3] - expected.object_pose[:3])),
        "object_rad": quat_angle(actual.object_pose[3:], expected.object_pose[3:]),
        "grasp_m": float(np.linalg.norm(actual.grasp_pose_hand_object[:3] - expected.grasp_pose_hand_object[:3])),
        "grasp_rad": quat_angle(actual.grasp_pose_hand_object[3:], expected.grasp_pose_hand_object[3:]),
        "max_arm_speed_rad_s": float(np.max(np.abs(actual.arm_dq))),
        "max_hand_speed_rad_s": float(np.max(np.abs(actual.hand_dq))) if len(actual.hand_dq) else 0.,
        "object_speed_m_s": float(np.linalg.norm(actual.object_twist[:3])),
    }
    limits = {
        "arm_q_rad": tol.arm_rad, "hand_q_rad": tol.hand_rad, "object_m": tol.object_m,
        "object_rad": tol.object_rad, "grasp_m": tol.grasp_m, "grasp_rad": tol.grasp_rad,
        "max_arm_speed_rad_s": tol.max_arm_speed_rad_s,
        "max_hand_speed_rad_s": tol.max_hand_speed_rad_s,
        "object_speed_m_s": tol.max_object_speed_m_s,
    }
    violations = {k: v for k, v in errors.items() if v > limits[k]}
    if actual.grasped != expected.grasped:
        violations["grasped"] = actual.grasped
    return {"ok": not violations, "errors": errors, "violations": violations}


def static_anchor_compatibility(grasp_exit: Boundary, place_entry: Boundary) -> None:
    """In transport grasp and held-object offset must not change.
    Joint and world object positions SHOULD differ and are intentionally not checked.
    """
    if not grasp_exit.grasped or not place_entry.grasped:
        raise ValueError("both boundaries must have an established grasp")
    if (grasp_exit.arm_joint_names != place_entry.arm_joint_names or
            grasp_exit.hand_joint_names != place_entry.hand_joint_names or
            grasp_exit.scene_id != place_entry.scene_id):
        raise ValueError("incompatible scene / robot")
    if np.max(np.abs(grasp_exit.hand_q - place_entry.hand_q)) > 0.08:
        raise ValueError("hand grasp must remain fixed during relaxed segment")
    if np.linalg.norm(grasp_exit.grasp_pose_hand_object[:3] - place_entry.grasp_pose_hand_object[:3]) > 0.015:
        raise ValueError("different grasp transforms; regrasp anchor needed")
    if quat_angle(grasp_exit.grasp_pose_hand_object[3:], place_entry.grasp_pose_hand_object[3:]) > 0.12:
        raise ValueError("different grasp rotation; regrasp anchor needed")
