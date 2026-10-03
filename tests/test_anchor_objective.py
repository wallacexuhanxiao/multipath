"""Regress the observed false grasp credit and incompatible tracking boundary."""
import numpy as np
from multipath.validation.physics import hand_tool_contact, object_tracking_score
from multipath.anchors.residual_control import project_arm_command


def test_contact_requires_thumb_and_another_finger():
    paths=['/hand/right_hand_thumb_rota_tip','/hand/right_hand_index_rota_tip',
           '/hand/right_hand_mid_tip','/hand/right_hand_link']
    assert not hand_tool_contact(paths,[[0,0,0],[1,0,0],[1,0,0],[1,0,0]])['opposition_contact']
    assert hand_tool_contact(paths,[[1,0,0],[1,0,0],[0,0,0],[0,0,0]])['opposition_contact']
    assert not hand_tool_contact(paths,[[1,0,0],[0,0,0],[0,0,0],[1,0,0]])['opposition_contact']


def test_tracking_reward_and_termination_share_boundary():
    score=object_tracking_score({'position_m':.04,'rotation_rad':.28})
    assert score['reward']<0 and not score['feasible']
    score=object_tracking_score({'position_m':.03,'rotation_rad':.14})
    assert score['reward']>0 and score['feasible']


def test_residual_cannot_accelerate_into_unbrakeable_joint_limit():
    # Repeated request beyond the limit used to become an empty next-step box.
    q=np.array([.8]);dq=np.array([.5]);dt=.01;limits=np.array([[-1.,1.]])
    for _ in range(100):
        command,_,_=project_arm_command(np.array([2.]),q,dq,dt,limits,np.array([2.]),np.array([2.]))
        velocity=(command-q)/dt
        assert abs(velocity[0]-dq[0])<=2*dt+1e-7
        assert abs(command[0])<=1+1e-9
        q,dq=command,velocity
    assert q[0]>.99 and abs(dq[0])<1e-5
