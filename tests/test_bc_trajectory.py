import numpy as np
from multipath.bc_trajectory import retime_bc


def test_retiming_preserves_bc_endpoints_and_respects_speed():
    # Fast measured wrist motion must be slowed together with the fingers.
    q=np.zeros((8,7));q[:,0]=np.linspace(0,1,8)
    fingers=np.tile(np.linspace(0,1,8)[:,None],(1,12))
    arm,hand,report=retime_bc(q,fingers)
    np.testing.assert_allclose(arm[[0,-1]],q[[0,-1]],atol=1e-10)
    np.testing.assert_allclose(hand[[0,-1]],fingers[[0,-1]],atol=1e-10)
    assert report['time_scale']>1
    assert np.max(np.abs(np.diff(arm,axis=0)*120))<=2.175*.7+1e-3
    assert len(arm)==len(hand)
