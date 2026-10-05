"""Regression for locally valid Anchors that cannot share a fixed-time bridge."""
from multipath.planning.bridge import bridge_precheck
from scipy.spatial.transform import Rotation


def state(pitch,twist):
    quat=Rotation.from_euler('ZYX',[0,pitch,0]).as_quat()[[3,0,1,2]]
    return dict(wrist_xyz_wxyz=[0,0,1,*quat],wrist_twist=twist,hand_q=[0]*12,
                contact={'mode':'grasped'},scene_id='same')


def test_fixed_time_rejects_displacement_and_velocity_boundaries():
    check=bridge_precheck(state(0,[0]*6),state(.82,[0,0,0,0,6,0]),.2,
                         dict(velocity_limits=[1,1,1,3,3,3],acceleration_limits=[5,5,5,15,15,15]))
    assert not check['feasible']
    assert check['minimum_duration_velocity_only_s']>.27
    assert 'terminal_velocity_exceeds_planning_budget' in check['reasons']
    assert 'boundary_velocity_change_exceeds_acceleration_budget' in check['reasons']
