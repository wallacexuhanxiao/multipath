"""Fail-closed schema for measured, physically verified anchor boundaries."""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any
import numpy as np


def _array(data: Any, count: int | None = None) -> np.ndarray:
    value = np.asarray(data, dtype=np.float64)
    if value.ndim != 1 or (count is not None and value.size != count):
        raise ValueError(f"expected vector of length {count}; got {value.shape}")
    if not np.all(np.isfinite(value)):
        raise ValueError("nonfinite measured state")
    return value


def _pose(data: Any) -> np.ndarray:
    value = _array(data, 7)
    norm = np.linalg.norm(value[3:])
    if abs(norm - 1) > 1e-3:
        raise ValueError("pose quaternion must be unit xyzw")
    return value


@dataclass(frozen=True)
class Boundary:
    """State measured from PhysX, not an IK guess. Poses: xyz+xyzw."""
    arm_joint_names: tuple[str, ...]
    arm_q: np.ndarray
    arm_dq: np.ndarray
    hand_joint_names: tuple[str, ...]
    hand_q: np.ndarray
    hand_dq: np.ndarray
    ee_pose: np.ndarray
    object_pose: np.ndarray
    object_twist: np.ndarray
    grasp_pose_hand_object: np.ndarray
    grasped: bool
    scene_id: str
    measured: bool = True

    def __post_init__(self) -> None:
        n, m = len(self.arm_joint_names), len(self.hand_joint_names)
        if not n or len(set(self.arm_joint_names)) != n:
            raise ValueError("unique nonempty arm joint names required")
        if len(set(self.hand_joint_names)) != m:
            raise ValueError("duplicate hand joints")
        for key, size in (("arm_q", n), ("arm_dq", n), ("hand_q", m),
                          ("hand_dq", m), ("object_twist", 6)):
            object.__setattr__(self, key, _array(getattr(self, key), size))
        for key in ("ee_pose", "object_pose", "grasp_pose_hand_object"):
            object.__setattr__(self, key, _pose(getattr(self, key)))
        if not self.scene_id or not self.measured:
            raise ValueError("boundary must originate from measured physical state")

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "Boundary":
        return cls(**{**raw, "arm_joint_names": tuple(raw["arm_joint_names"]),
                      "hand_joint_names": tuple(raw["hand_joint_names"])})

    def to_dict(self) -> dict[str, Any]:
        result = vars(self).copy()
        for k, v in result.items():
            if isinstance(v, np.ndarray):
                result[k] = v.tolist()
        result["arm_joint_names"] = list(self.arm_joint_names)
        result["hand_joint_names"] = list(self.hand_joint_names)
        return result


@dataclass(frozen=True)
class Anchor:
    name: str
    entry: Boundary
    exit: Boundary
    commands: np.ndarray
    verified: bool
    source_commit: str
    frame_range: tuple[int, int]

    def __post_init__(self) -> None:
        if not self.verified:
            raise ValueError(f"{self.name}: no physical validation")
        if not self.source_commit or not self.entry.measured or not self.exit.measured:
            raise ValueError("missing verified source provenance")
        if self.entry.scene_id != self.exit.scene_id:
            raise ValueError("anchor scene changed")
        if self.entry.arm_joint_names != self.exit.arm_joint_names or self.entry.hand_joint_names != self.exit.hand_joint_names:
            raise ValueError("joint maps change inside anchor")
        cmd = np.asarray(self.commands, dtype=np.float64)
        expected = len(self.entry.arm_joint_names) + len(self.entry.hand_joint_names)
        if cmd.ndim != 2 or not len(cmd) or cmd.shape[1] != expected or not np.isfinite(cmd).all():
            raise ValueError(f"commands require [T,{expected}] finite values")
        object.__setattr__(self, "commands", cmd)
        if len(self.frame_range) != 2 or not (0 <= self.frame_range[0] < self.frame_range[1]):
            raise ValueError("invalid anchor frame range")

    @classmethod
    def load(cls, directory: str | Path) -> "Anchor":
        root = Path(directory)
        data = json.loads((root / "anchor.json").read_text())
        with np.load(root / "commands.npz", allow_pickle=False) as archive:
            cmds = archive["commands"].copy()
        return cls(name=data["name"], entry=Boundary.from_dict(data["entry"]),
                   exit=Boundary.from_dict(data["exit"]), commands=cmds,
                   verified=data["verified"], source_commit=data["source_commit"],
                   frame_range=tuple(data["frame_range"]))

    def save(self, directory: str | Path) -> None:
        root = Path(directory)
        root.mkdir(parents=True, exist_ok=True)
        (root / "anchor.json").write_text(json.dumps({
            "name": self.name, "entry": self.entry.to_dict(), "exit": self.exit.to_dict(),
            "verified": self.verified, "source_commit": self.source_commit,
            "frame_range": list(self.frame_range)}, indent=2) + "\n")
        np.savez_compressed(root / "commands.npz", commands=self.commands)


@dataclass(frozen=True)
class Plan:
    arm_q: np.ndarray
    dt: float
    goal_arm_joint_names: tuple[str, ...]

    def __post_init__(self) -> None:
        q = np.asarray(self.arm_q, dtype=np.float64)
        if q.ndim != 2 or q.shape[0] < 2 or q.shape[1] != len(self.goal_arm_joint_names) or not np.isfinite(q).all():
            raise ValueError("invalid planned joint trajectory")
        if not (np.isfinite(self.dt) and self.dt > 0):
            raise ValueError("invalid planning timestep")
        object.__setattr__(self, "arm_q", q)
