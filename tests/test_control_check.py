import numpy as np
from multipath.anchors.control_check import control_feasibility


def test_zero_residual_is_not_rejected_for_interpolated_finger_zero():
    controls=np.zeros((10,26));controls[-1,11]=-2.5e-19
    limits=np.tile([-2.,2.],(19,1));limits[11,0]=0.
    args=(np.zeros(7),np.zeros(7),1/120,limits,np.ones(7)*2,np.ones(7)*15)
    assert control_feasibility(controls,*args)['feasible']
    controls[-1,11]=-.001
    assert not control_feasibility(controls,*args)['feasible']


def test_hand_fit_preserves_source_steps_and_measured_entry_velocity():
    from multipath.anchors.synthesis import fit_hand_startup
    desired=np.full((12,1),.2);q=np.array([.01]);dq=np.array([.1]);dt=.01
    fitted=fit_hand_startup(desired,q,dq,dt,np.array([[0.,1.]]),np.array([1.]))
    velocity=np.diff(np.vstack((q,fitted)),axis=0)/dt
    assert len(fitted)==len(desired)
    np.testing.assert_allclose(velocity[0],dq,atol=1e-7)
    assert np.max(abs(velocity))<=1.00001
