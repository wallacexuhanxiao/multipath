"""Protect measured finger tracking lag caused by zero drive velocity targets."""
from types import SimpleNamespace
import numpy as np
from multipath.execution.mounted_scene import MountedScene


def test_local_hand_feedforward_uses_previous_command_and_bridge_stays_frozen():
    robot=SimpleNamespace(get_joint_positions=lambda:np.zeros(19),
                          get_joint_velocities=lambda:np.zeros(19))
    robot.apply_action=lambda a:setattr(robot,'action',a)
    s=SimpleNamespace(dt=1/120,arm=np.arange(7),fingers=np.arange(7,19),
        qcommand=np.zeros(19),velocity_command=np.zeros(19),robot=robot,
        world=SimpleNamespace(step=lambda **kw:None),Action=lambda **kw:kw,
        ticks=0,manifest={'fps':30},velocity=np.ones(7),acceleration=np.ones(7),
        previous_dq=np.zeros(7),max_velocity_ratio=0.,max_acceleration_ratio=0.)
    hand=np.arange(12)*.001
    MountedScene.step(s,np.zeros(7),hand,'LOCAL_PADDED',.25,np.ones(7)*.2)
    np.testing.assert_allclose(robot.action['joint_velocities'][s.fingers],hand/s.dt)
    np.testing.assert_allclose(robot.action['joint_velocities'][s.arm],.2)
    MountedScene.step(s,np.zeros(7),hand,'EXPORTED_CORE',.5,np.zeros(7))
    np.testing.assert_allclose(robot.action['joint_velocities'][s.fingers],0.)
    MountedScene.step(s,np.zeros(7),hand+.001,'RELAXED',.75,np.zeros(7))
    np.testing.assert_allclose(robot.action['joint_velocities'][s.fingers],0.)
