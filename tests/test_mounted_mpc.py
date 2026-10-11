"""Protect the observed bug: command-valid failed interactions were committed."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from multipath.anchors import mpc
from multipath.anchors.schema import InteractionAnchor


@pytest.mark.parametrize('policy,frame,contact,expected_reason', [
    ('strict',1,'grasped','core_object_fidelity'),
    ('diagnostic',1,'grasped',None),
    ('diagnostic',1,'free','core_grasp_retention'),
    ('diagnostic',0,'grasped','core_entry_source_boundary'),
    ('diagnostic',3,'grasped','core_exit_handoff'),
])
def test_tracking_diagnostic_preserves_physical_and_boundary_gates(policy,frame,contact,expected_reason):
    start=dict(time_s=0.,contact={'mode':'grasped'},hand_object_transform=np.eye(4).tolist(),
               objects={'tool':{'pose':[0.,0.,0.,1.,0.,0.,0.],'velocity':[0.]*6}})
    measured=dict(start,contact={'mode':contact},
                  objects={'tool':{'pose':[.06,0.,0.,1.,0.,0.,0.],'velocity':[0.]*6}})
    scene=SimpleNamespace(manifest={'fps':30},check_transition=lambda *a: {'success':False})
    anchor=InteractionAnchor('test',0,3,0,3,'grasped','grasped')
    check=mpc.WindowEvaluation(scene,anchor,{'tool_pose':np.tile(np.eye(4),(4,1,1))},start,
                               core_tracking_policy=policy)
    passed=check.observe(frame,np.array([0.,0.,0.,1.,0.,0.,0.]+[0.]*12),measured=measured)
    assert passed == (expected_reason is None)
    assert (check.failure or {}).get('reason') == expected_reason
    if frame>0:
        assert check.tracking_failures[0]['weighted_error'] == pytest.approx(1.2)


@pytest.mark.parametrize('supported,contact,passed',[(True,'grasped',True),(False,'grasped',False),(True,'free',False)])
def test_supported_contact_can_pivot_but_carry_or_contact_loss_still_fails(supported,contact,passed):
    transform=np.eye(4);transform[:3,:3]=mpc.Rotation.from_rotvec([0.,0.,.36]).as_matrix()
    start=dict(time_s=0.,contact={'mode':'grasped'},hand_object_transform=np.eye(4).tolist(),
               objects={'tool':{'pose':[0.,0.,0.,1.,0.,0.,0.],'velocity':[0.]*6}})
    measured=dict(start,contact={'mode':contact},hand_object_transform=transform.tolist())
    scene=SimpleNamespace(manifest={'fps':30},surface_supported_object=supported)
    check=mpc.WindowEvaluation(scene,InteractionAnchor('test',0,3,0,3,'grasped','grasped'),
        {'tool_pose':np.tile(np.eye(4),(4,1,1))},start,core_tracking_policy='diagnostic')
    assert check.observe(1,np.zeros(19),measured=measured) == passed
    assert check.grasp_transform_deviations[0]['rotation_rad'] == pytest.approx(.36)
    if not passed:assert check.failure['reason']=='core_grasp_retention'


@pytest.mark.parametrize('object_error', [0., .06])
def test_chunk_switching_never_commits_failed_interaction(monkeypatch, tmp_path, object_error):
    class Scene:
        dt = 1/120
        manifest = {'fps': 30}
        arm = np.arange(7); fingers = np.arange(7,19)
        velocity = np.ones(7)*2; acceleration = 15.
        robot = SimpleNamespace(_articulation_view=SimpleNamespace(
            get_dof_limits=lambda: np.tile([-2.,2.], (19,1))))

        def __init__(self):
            self.frame = 0.; self.committed = []; self.solved_targets = []

        def state(self):
            error = object_error if self.frame >= 15 else 0.
            return dict(arm_q=[0.]*7, arm_dq=[0.]*7, hand_q=[0.]*12, hand_dq=[0.]*12,
                        time_s=self.frame/30, contact={'mode':'free'}, hand_object_transform=np.eye(4).tolist(),
                        wrist_xyz_wxyz=[0.,0.,0.,1.,0.,0.,0.], scene_id='test',
                        objects={'tool':{'pose':[error,0.,0.,1.,0.,0.,0.], 'velocity':[0.]*6}})

        snapshot = state

        def solve(self, target, seed):
            self.solved_targets.append(np.asarray(target).copy())
            return np.zeros(7)

        def begin_trial(self, out, state, mesh=False):
            self.frame = state['time_s']*30
            self.out = Path(out); self.out.mkdir(parents=True, exist_ok=True); self.records=[]

        def step(self, arm, hand, mode, frame, velocity):
            self.frame = frame; self.records.append(self.state())
            if mode == 'LOCAL_COMMIT': self.committed.append(frame)

    # Mock only kinematic proposal generation; this test isolates selection and
    # source-clock execution, not Isaac or IK numerical correctness.
    monkeypatch.setattr(mpc,'hand_limits_from_urdf',lambda scene: np.ones(12))
    monkeypatch.setattr(mpc,'fit_arm_startup',lambda target,*a,**kw: target)
    monkeypatch.setattr(mpc,'fit_hand_startup',lambda target,*a,**kw: target)
    monkeypatch.setattr(mpc,'fit_timed_controls',lambda arm,hand,*a,**kw: (np.zeros((len(arm)*4,26)),{}))
    monkeypatch.setattr(mpc,'fitted_cartesian_prior',lambda scene,controls,tpf:
                        np.tile([0.,0.,0.,1.,0.,0.,0.]+[0.]*12,(len(controls)//tpf,1)))
    anchor = InteractionAnchor('test',0,47,0,53,'free','free')
    data = {'wrist_hand_q':np.tile([0.,0.,0.,1.,0.,0.,0.]+[0.]*12,(53,1)),
            'tool_pose':np.tile(np.eye(4),(53,1,1))}
    scene = Scene(); baseline = np.zeros((53*4,26))
    run = lambda: mpc.mounted_mpc(scene,anchor,data,baseline,scene.state(),tmp_path,
                                 config=mpc.MPCConfig(candidates=2,iterations=1))
    if object_error:
        with pytest.raises(mpc.RefinementRequired):run()
        assert scene.committed == []
        assert any(np.linalg.norm(t[4:7])>1e-6 for t in scene.solved_targets)
        trials=json.loads((tmp_path/'search.json').read_text())
        assert all(t['feasible_controls'] and not t['interaction_feasible'] for t in trials)
        assert all(t['failure']['source_frame']==15 for t in trials)
        zero=next(t for t in trials if t['mode']=='MPPI_ZERO')
        assert zero['return']==trials[0]['return']
        assert zero['failure']==trials[0]['failure']
    else:
        result=run()
        assert result.shape==baseline.shape
        np.testing.assert_allclose(scene.committed,np.arange(1,213)/4)
        chunks=json.loads((tmp_path/'chunks.json').read_text())
        assert [(c['end_tick'],c['prediction_end_tick']) for c in chunks]==[(80,160),(160,212),(212,212)]
        assert all(c['selected']=='REPLAY' and c['commit_success'] for c in chunks)
