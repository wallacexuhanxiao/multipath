"""cuRobo joint-space transport for a robot exactly matching the PhysX scene.

The held mesh pose comes from the *observed* grasp-exit object pose, and it is
attached to the planner's dedicated collision link. See cuRobo's official
MotionGen.attach_external_objects_to_robot and plan_single_js documentation.
"""
from __future__ import annotations

from pathlib import Path
import numpy as np

from ..schema import Boundary, Plan


def as_curobo_pose(pose_xyzw: np.ndarray) -> list[float]:
    """Boundary xyz + xyzw -> cuRobo xyz + wxyz."""
    return [*map(float, pose_xyzw[:3]), float(pose_xyzw[6]),
            *map(float, pose_xyzw[3:6])]


class CuRoboPlanner:
    """A configured, attached-object planning adapter, not a simulator."""

    def __init__(
        self,
        robot_yaml: str,
        world_yaml: str,
        *,
        held_object_mesh: str,
        attachment_link: str = "attached_object",
        device: str = "cuda:0",
        dt: float = .05,
    ) -> None:
        for key, path in (("robot_yaml", robot_yaml), ("world_yaml", world_yaml),
                          ("held_object_mesh", held_object_mesh)):
            if not Path(path).is_file():
                raise FileNotFoundError(f"{key}: {path}")
        if not attachment_link:
            raise ValueError("robot YAML must declare the dedicated attachment link")
        if not (0 < dt < 1):
            raise ValueError("invalid timestep")
        try:
            from curobo.types.base import TensorDeviceType
            from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
        except ImportError as exc:
            raise RuntimeError("install a compatible cuRobo in the Isaac GPU environment") from exc
        tensor = TensorDeviceType(device=device)
        cfg = MotionGenConfig.load_from_robot_config(
            robot_yaml, world_yaml, tensor_args=tensor,
            interpolation_dt=dt, self_collision_check=True,
            collision_activation_distance=.01,
        )
        self._planner = MotionGen(cfg)
        self._planner.warmup()
        self._device = device
        self._mesh_path = held_object_mesh
        self._attachment_link = attachment_link
        self._dt = dt

    def _joint_state(self, q: np.ndarray, names: tuple[str, ...]):
        import torch
        from curobo.types.robot import JointState
        return JointState.from_position(
            torch.as_tensor(q, device=self._device, dtype=torch.float32)[None],
            joint_names=list(names),
        )

    def plan(self, start: Boundary, goal: Boundary) -> Plan:
        from curobo.geom.types import Mesh
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

        if start.arm_joint_names != goal.arm_joint_names or start.scene_id != goal.scene_id:
            raise ValueError("scene and named robot joints must match")
        if not start.grasped or not goal.grasped:
            raise ValueError("held-object transport requires measured grasp at both boundaries")
        planner_names = tuple(self._planner.joint_names)
        if planner_names != start.arm_joint_names:
            raise ValueError("cuRobo configuration does not match measured PhysX joint order")

        start_js = self._joint_state(start.arm_q, start.arm_joint_names)
        goal_js = self._joint_state(goal.arm_q, goal.arm_joint_names)
        held = Mesh(name="held_object", pose=as_curobo_pose(start.object_pose),
                    file_path=self._mesh_path)
        if not hasattr(self._planner, "attach_external_objects_to_robot"):
            raise RuntimeError("this cuRobo version lacks required external attachment API")
        attached = self._planner.attach_external_objects_to_robot(
            start_js, [held], link_name=self._attachment_link)
        if not attached:
            raise RuntimeError("failed to attach carried object to cuRobo collision model")
        # No replan can ignore the carried object's collision footprint.
        result = self._planner.plan_single_js(
            start_js, goal_js, MotionGenPlanConfig(
                max_attempts=30, enable_graph_attempt=5,
                check_start_validity=True))
        success = result.success
        if hasattr(success, "item"):
            success = bool(success.item())
        if not success:
            raise RuntimeError("cuRobo failed canonical joint-space planning")
        trajectory = result.get_interpolated_plan()
        if getattr(trajectory, "joint_names", None) is not None:
            if tuple(trajectory.joint_names) != start.arm_joint_names:
                raise ValueError("joint order changed in cuRobo result")
        q = trajectory.position.detach().cpu().numpy()
        if q.ndim == 3:
            if q.shape[0] != 1:
                raise ValueError("unexpected batch from plan_single_js")
            q = q[0]
        if np.max(np.abs(q[0] - start.arm_q)) > .04:
            raise RuntimeError("planned start mismatches measured grasp exit")
        if np.max(np.abs(q[-1] - goal.arm_q)) > .04:
            raise RuntimeError("planned endpoint misses canonical place-entry configuration")
        actual_dt = getattr(result, "interpolation_dt", None)
        if actual_dt is None:
            actual_dt = self._dt
        if hasattr(actual_dt, "item"):
            actual_dt = actual_dt.item()
        return Plan(q, float(actual_dt), start.arm_joint_names)
