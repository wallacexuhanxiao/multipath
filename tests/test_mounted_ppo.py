"""Zero residual must preserve the fitted baseline and its source clock."""
from types import SimpleNamespace
import numpy as np

from multipath.anchors import ppo
from multipath.anchors.schema import InteractionAnchor


def test_zero_policy_does_not_retarget_or_add_settle_steps(monkeypatch,tmp_path):
    n=12;dt=1/120
    # Feasible baseline starts with measured zero velocity then accelerates.
    v=np.minimum(np.arange(n)*.01,.05)
    q=np.cumsum(v)*dt
    controls=np.zeros((n,26));controls[:,:7]=q[:,None];controls[:,19:26]=v[:,None]
    initial=dict(arm_q=[0.]*7,arm_dq=[0.]*7,hand_q=[0.]*12,hand_dq=[0.]*12,
        wrist_xyz_wxyz=[0.,0.,0.,1.,0.,0.,0.],hand_object_transform=np.eye(4).tolist(),
        time_s=0.,contact={'mode':'free'},entry_origin='initial_state',entry_provenance={},
        objects={r:{'pose':[0.,0.,0.,1.,0.,0.,0.],'velocity':[0.]*6} for r in ['tool','target']})
    class Scene:
        dt=1/120;manifest={'fps':30,'provenance':{}}
        velocity=np.ones(7)*2;acceleration=15.
        arm=np.arange(7);fingers=np.arange(7,19)
        robot=SimpleNamespace(_articulation_view=SimpleNamespace(get_dof_limits=lambda:np.tile([-2.,2.],(19,1))))
        def begin_trial(self,*args,**kwargs):self.commands=[];self.s=dict(initial);self.records=[]
        def step(self,arm,hand,mode,frame,velocity):
            self.commands.append(np.r_[arm,hand,velocity]);self.s=dict(initial,arm_q=arm.tolist(),arm_dq=velocity.tolist(),hand_q=hand.tolist(),time_s=frame/30)
        def state(self):return self.s
        snapshot=state
        def solve(self,*args):raise AssertionError('Zero policy must not run IK')
    monkeypatch.setattr(ppo,'hand_limits_from_urdf',lambda s:np.ones(12))
    monkeypatch.setattr(ppo,'fitted_cartesian_prior',lambda s,c,t:np.tile([0.,0.,0.,1.,0.,0.,0.]+[0.]*12,(3,1)))
    # Core ends after this short prediction window, so this is a context probe.
    anchor=InteractionAnchor('local',10,20,0,21,'free','free')
    data={'tool_pose':np.tile(np.eye(4),(21,1,1))}
    scene=Scene();env=ppo.MountedResidualEnv(scene,anchor,data,controls,initial,0,tmp_path)
    env.reset()
    for _ in range(3):obs,reward,done,result=env.step(np.zeros(18))
    assert done and result['success']
    assert len(scene.commands)==n and scene.state()['time_s']==3/30
    np.testing.assert_allclose(scene.commands,controls,atol=1e-14)
    env.reset()
    def failed_observe(frame,command):
        env.objective.failure={'source_frame':frame,'reason':'core_entry_contact'}
        return False
    env.objective.observe=failed_observe
    _,reward,done,result=env.step(np.zeros(18))
    assert done and not result['success'] and reward==0.
