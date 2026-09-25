import json
from dataclasses import replace
import numpy as np
import pytest
from multipath.schema import Boundary, Anchor, Plan
from multipath.transitions import check, static_anchor_compatibility
from multipath.orchestrator import run_single


def state(q=0., obj=0., grasp=True, scene="put_in"):
    return Boundary(
        arm_joint_names=("j1", "j2"), arm_q=[q, 0.], arm_dq=[0., 0.],
        hand_joint_names=("finger",), hand_q=[.8], hand_dq=[0.],
        ee_pose=[q, 0., 0., 0., 0., 0., 1.],
        object_pose=[obj, 0., 0., 0., 0., 0., 1.],
        object_twist=[0.]*6, grasp_pose_hand_object=[0., 0., .1, 0., 0., 0., 1.],
        grasped=grasp, scene_id=scene)


def anchors():
    g = Anchor("grasp", state(0., 0., False), state(0., 0.),
               np.array([[0., 0., .8]]), True, "real-sha", (0, 10))
    p = Anchor("put_in", state(1., 1.), state(1., 1., False),
               np.array([[1., 0., .8]]), True, "real-sha", (30, 50))
    return g, p


class MockPlanner:
    def plan(self, start, goal):
        return Plan([[0., 0.], [1., 0.]], .05, start.arm_joint_names)


class MockWorld:
    def __init__(self, g, p, fail_transport=False):
        self.g, self.p = g, p
        self.current = g.entry
        self.calls = []
        self.fail_transport = fail_transport
        self.failed = False

    def reset_initial(self):
        self.calls.append("reset_initial")
        self.current = self.g.entry
        return self.current

    def observe(self):
        return self.current

    def execute_joint_commands(self, commands, *, label, dt=None):
        self.calls.append(label)
        if label == "grasp":
            self.current = self.g.exit
        elif label == "relaxed_transport":
            self.current = self.p.entry
            if self.fail_transport:
                self.failed = True
        else:
            self.current = self.p.exit

    def correct_to(self, target):
        self.calls.append("correct_to")
        self.current = target
        return True

    def check_grasp(self):
        return self.current.grasped

    def check_placement(self):
        return not self.current.grasped and self.calls[-1] == "put_in"

    def no_hard_physics_failure(self):
        return not self.failed

    def save_actual_trace(self, folder):
        self.calls.append("save_trace")
        (folder / "mock_trace.json").write_text(json.dumps(self.calls))


def test_boundary_roundtrip(tmp_path):
    g, _ = anchors()
    g.save(tmp_path)
    loaded = Anchor.load(tmp_path)
    assert loaded.source_commit == "real-sha"
    np.testing.assert_allclose(loaded.commands, g.commands)
    assert loaded.entry.arm_joint_names == ("j1", "j2")


def test_invalid_unverified_anchor():
    g, _ = anchors()
    with pytest.raises(ValueError, match="physical validation"):
        replace(g, verified=False)


def test_mismatched_grasp_rejected():
    g, p = anchors()
    with pytest.raises(ValueError, match="different grasp transforms"):
        static_anchor_compatibility(g.exit, replace(p.entry, grasp_pose_hand_object=[.1, 0., .1, 0., 0., 0., 1.]))


def test_observed_boundary_detects_bad_state():
    g, _ = anchors()
    gate = check(replace(g.exit, arm_q=[.5, 0.]), g.exit)
    assert not gate["ok"] and "arm_q_rad" in gate["violations"]


def test_one_continuous_rollout(tmp_path):
    g, p = anchors()
    world = MockWorld(g, p)
    result = run_single(world, MockPlanner(), g, p, tmp_path)
    assert result.success
    assert world.calls == ["reset_initial", "grasp", "relaxed_transport", "put_in", "save_trace"]
    assert json.loads((tmp_path / "result.json").read_text())["success"]


def test_transport_failure_fails_closed(tmp_path):
    g, p = anchors()
    world = MockWorld(g, p, fail_transport=True)
    result = run_single(world, MockPlanner(), g, p, tmp_path)
    assert not result.success and result.stage == "transport"
    assert "put_in" not in world.calls and "save_trace" in world.calls
