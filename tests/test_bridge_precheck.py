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
    # Both separate checks pass, but opposing momentum still makes the turn
    # impossible in 0.2 s. This is the observed inter-Anchor failure mode.
    start=state(0,[0,0,0,1.3,0,0]);goal=state(0,[0,0,0,-.8,0,0])
    goal['wrist_xyz_wxyz'][3:]=Rotation.from_euler('ZYX',[0,0,-.36]).as_quat()[[3,0,1,2]].tolist()
    check=bridge_precheck(start,goal,.2,
                         dict(velocity_limits=[1,1,1,3,3,3],acceleration_limits=[5,5,5,15,15,15]))
    assert check['minimum_duration_velocity_only_s']<.2
    assert check['minimum_duration_velocity_change_only_s']<.2
    assert 'displacement_exceeds_acceleration_reachability' in check['reasons']
