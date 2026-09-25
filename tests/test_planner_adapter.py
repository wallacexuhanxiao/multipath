from types import SimpleNamespace
import sys
import types
import numpy as np
import pytest
from multipath.planning.curobo import CuRoboPlanner, as_curobo_pose
from test_v0 import state


def test_quaternion_order():
    assert as_curobo_pose(np.array([1, 2, 3, 0, .5, 0, .866025403784])) == [
        1., 2., 3., .866025403784, 0., .5, 0.
    ]


def test_fail_closed_on_joint_names():
    planner = CuRoboPlanner.__new__(CuRoboPlanner)
    planner._planner = SimpleNamespace(joint_names=["wrong"])
    planner._device = "cpu"
    with pytest.raises((ImportError, ValueError)):
        planner.plan(state(), state())
