"""Regression for locally valid Anchors that cannot share a fixed-time bridge."""
from multipath.planning.bridge import bridge_precheck
from scipy.spatial.transform import Rotation


def test_moving_boundaries_fit_without_stretching_source_time():
    import numpy as np
    import pytest
    from multipath.planning.timing import boundary_fitted_path,parameterize_path
    raw=np.linspace([0.,0.],[.04,.02],25)
    first=np.array([.2,-.1]);last=np.array([.1,.2]);dt=1/120
    velocity=np.ones(2);acceleration=np.full(2,5.)
    # Real bridge failure: a single scalar path speed cannot express these
    # boundary velocity directions. Fit geometry instead, then check collision.
    with pytest.raises(RuntimeError,match='incompatible'):
        parameterize_path(raw,24,dt,velocity,acceleration,first,last)
    path,report=boundary_fitted_path(raw,24,dt,velocity,acceleration,first,last)
    np.testing.assert_allclose(path[[0,-1]],raw[[0,-1]],atol=1e-10)
    np.testing.assert_allclose((path[1]-path[0])/dt,first,atol=1e-10)
    np.testing.assert_allclose((path[-1]-path[-2])/dt,last,atol=1e-10)
    assert report['budget_s']==.2
    assert report['command_velocity_ratio']<=1.0001
    assert report['command_acceleration_ratio']<=1.0001
    with pytest.raises(RuntimeError,match='No fixed-time'):
        boundary_fitted_path(np.linspace([0.,0.],[.3,.02],25),24,dt,
                             velocity,acceleration,first,last)


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
