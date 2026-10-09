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


def test_tracking_reward_and_termination_share_appendix_boundary():
    score=object_tracking_score({'position_m':.04,'rotation_rad':np.deg2rad(24.)})
    assert score['reward']<0 and not score['feasible']
    assert not object_tracking_score({'position_m':.051,'rotation_rad':0.})['feasible']
    score=object_tracking_score({'position_m':.03,'rotation_rad':.14})
    assert score['reward']>0 and score['feasible']
    assert object_tracking_score({'position_m':0.,'rotation_rad':np.deg2rad(30.)})['feasible']
    assert not object_tracking_score({'position_m':0.,'rotation_rad':np.deg2rad(30.1)})['feasible']


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


def test_release_anchor_allows_contact_change_inside_core():
    from types import SimpleNamespace
    from multipath.anchors.schema import InteractionAnchor
    from multipath.anchors.mpc import WindowEvaluation
    state={'time_s':0.,'contact':{'mode':'grasped'},'hand_object_transform':np.eye(4),
           'objects':{'tool':{'pose':[0,0,0,1,0,0,0]}}}
    measured=dict(state,contact={'mode':'free'})
    scene=SimpleNamespace(manifest={'fps':30},state=lambda:measured)
    data={'tool_pose':np.tile(np.eye(4),(6,1,1))}
    release=WindowEvaluation(scene,InteractionAnchor('release',0,4,0,6,'grasped','free'),data,state)
    assert release.observe(1,np.zeros(25))
    hold=WindowEvaluation(scene,InteractionAnchor('hold',0,4,0,6,'grasped','grasped'),data,state)
    assert not hold.observe(1,np.zeros(25)) and hold.failure['reason']=='core_grasp_retention'


def test_core_entry_must_match_its_own_source_boundary():
    from types import SimpleNamespace
    from multipath.anchors.schema import InteractionAnchor
    from multipath.anchors.mpc import WindowEvaluation
    state={'time_s':0.,'contact':{'mode':'grasped','opposition_contact':True,
           'thumb_contact':True,'other_finger_contact':True},'hand_object_transform':np.eye(4),
           'objects':{'tool':{'pose':[.08,0,0,1,0,0,0]}}}
    scene=SimpleNamespace(manifest={'fps':30},state=lambda:state)
    data={'tool_pose':np.tile(np.eye(4),(6,1,1))}
    value=WindowEvaluation(scene,InteractionAnchor('interaction',2,4,0,6,'grasped','grasped'),data,state)
    assert value.observe(1,np.zeros(25)) and value.last_reward==2.
    assert not value.observe(2,np.zeros(25))
    assert value.failure['reason']=='core_entry_source_boundary'
