import pytest
from multipath.grasp_hold import update_preload


def test_grip_feedback_deadband_rate_limit_and_saturation():
    assert update_preload(0,.009,.11,1/120)==0
    assert update_preload(.02,.02,.3,1/120)==pytest.approx(.02125)
    preload=0
    for _ in range(240):preload=update_preload(preload,.03,.3,1/120)
    assert preload==pytest.approx(.1)
    assert update_preload(preload,0,0,1/120)==preload
    with pytest.raises(ValueError,match='Nonfinite'):
        update_preload(0,float('nan'),0,1/120)
