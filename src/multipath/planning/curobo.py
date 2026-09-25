"""DexImit-inspired cuRobo planner for a *matching* robot and collision scene.

The caller must supply the exact cuRobo robot YAML for the Isaac embodiment.
No fake IK/arm joints, no hidden self-collision exclusion, no proxy-world
success claim. The held-object attachment is mandatory for Put-In transport.
"""
from __future__ import annotations

from pathlib import Path
import numpy as np
from ..schema import Boundary, Plan


class CuRoboPlanner:
    def __init__(self, robot_yaml: str, world_yaml: str, *,
                 held_object_mesh: str, held_object_pose: list[float],
                 device: str = "cuda:0", dt: float = .05):
        for key, path in (("robot_yaml", robot_yaml), ("world_yaml", world_yaml),
                          ("held_object_mesh", held_object_mesh)):
            if not Path(path).is_file():
                raise FileNotFoundError(f"{key}: {path}")
        if len(held_object_pose) != 7:
            raise ValueError("held_object_pose must be xyz+wxyz in cuRobo frame")
        try:
            from curobo.types.base import TensorDeviceType
            from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
            from curobo.types.robot import JointState
            from curobo.types.math import Pose
        except ImportError as exc:
            raise RuntimeError("install matching cuRobo in Isaac CUDA environment") from exc
        self._JointState, self._Pose = JointState, Pose
        self._dt = dt
        tensor = TensorDeviceType(device=device)
        cfg = MotionGenConfig.load_from_robot_config(
            robot_yaml, world_yaml, tensor_args=tensor, interpolation_dt=dt,
            self_collision_check=True, collision_activation_distance=.01,
        )
        self._planner = MotionGen(cfg)
        self._planner.warmup()
        self._device = device
        # Require attached-object support, rather than silently planning the
        # arm through a scene that ignores the object in its hand.
        from curobo.geom.types import Mesh
        if not hasattr(self._planner, "attach_objects_to_robot"):
            raise RuntimeError("cuRobo version lacks attached-object API; verify version")
        held_mesh = Mesh(name="held_object", pose=held_object_pose, file_path=held_object_mesh)
        self._held_object = held_mesh

    def plan(self, start: Boundary, goal: Boundary) -> Plan:
        """Plan to the verified place-entry EE pose; reject wrong canonical joint goal."""
        import torch
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
        if start.arm_joint_names != goal.arm_joint_names:
            raise ValueError("arm joint maps differ")
        # cuRobo expects wxyz, our measured/recorded state stores xyzw.
        def cu_pose(x: np.ndarray) -> list[float]:
            return [*map(float, x[:3]), float(x[6]), *map(float, x[3:6])]
        start_js = self._JointState.from_position(
            torch.as_tensor(start.arm_q, device=self._device, dtype=torch.float32)[None])
        if hasattr(self._planner, "attach_objects_to_robot"):
            # Official API accepts a list of obstacle names already present in
            # the world and an optional sphere radius. The exact attached
            # world obstacle must be inserted by the integration adapter.
            # Refuse unsafe planning until it has been configured externally.
            if not getattr(self, "_attachment_confirmed", False):
                raise RuntimeError("attach held object in matching cuRobo robot/world before planning")
        result = self._planner.plan_single(
            start_js, self._Pose.from_list(cu_pose(goal.ee_pose)),
            MotionGenPlanConfig(max_attempts=30, enable_graph_attempt=5))
        if not bool(result.success):
            raise RuntimeError("cuRobo could not reach anchor entrance")
        trajectory = result.get_interpolated_plan()
        if hasattr(trajectory, "joint_names") and trajectory.joint_names is not None:
            if tuple(trajectory.joint_names) != start.arm_joint_names:
                raise ValueError("cuRobo joint order mismatches PhysX")
        q = trajectory.position.squeeze(0).detach().cpu().numpy()
        if np.max(np.abs(q[0] - start.arm_q)) > .04:
            raise RuntimeError("planned start differs from actual measured state")
        if np.max(np.abs(q[-1] - goal.arm_q)) > .04:
            raise RuntimeError("pose goal did not reach canonical joint configuration")
        return Plan(q, self._dt, start.arm_joint_names)

    def confirm_attached_world_object(self) -> None:
        """Only call after integration successfully invokes actual cuRobo attachment."""
        raise NotImplementedError("verify cuRobo attach_objects_to_robot contract for installed version")
