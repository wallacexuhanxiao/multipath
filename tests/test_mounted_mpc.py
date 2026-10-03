"""Protect the observed bug: command-valid failed interactions were committed."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from multipath.anchors import mpc
from multipath.anchors.schema import InteractionAnchor


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
            self.frame = 0.; self.committed = []

        def state(self):
            error = object_error if self.frame >= 15 else 0.
            return dict(arm_q=[0.]*7, arm_dq=[0.]*7, hand_q=[0.]*12, hand_dq=[0.]*12,
                        time_s=self.frame/30, contact={'mode':'free'}, hand_object_transform=np.eye(4).tolist(),
                        wrist_xyz_wxyz=[0.,0.,0.,1.,0.,0.,0.], scene_id='test',
                        objects={'tool':{'pose':[error,0.,0.,1.,0.,0.,0.], 'velocity':[0.]*6}})

        snapshot = state

        def solve(self, target, seed):
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
    anchor = InteractionAnchor('test',0,47,0,53,'free','free')
    data = {'wrist_hand_q':np.tile([0.,0.,0.,1.,0.,0.,0.]+[0.]*12,(53,1)),
            'tool_pose':np.tile(np.eye(4),(53,1,1))}
    scene = Scene(); baseline = np.zeros((53*4,26))
    run = lambda: mpc.mounted_mpc(scene,anchor,data,baseline,scene.state(),tmp_path,
                                 config=mpc.MPCConfig(candidates=2,iterations=1))
    if object_error:
        with pytest.raises(mpc.RefinementRequired):run()
        assert scene.committed == []
        trials=json.loads((tmp_path/'search.json').read_text())
        assert all(t['feasible_controls'] and not t['interaction_feasible'] for t in trials)
        assert all(t['failure']['source_frame']==15 for t in trials)
    else:
        result=run()
        assert result.shape==baseline.shape
        np.testing.assert_allclose(scene.committed,np.arange(1,213)/4)
        chunks=json.loads((tmp_path/'chunks.json').read_text())
        assert [(c['end_tick'],c['prediction_end_tick']) for c in chunks]==[(80,160),(160,212),(212,212)]
        assert all(c['selected']=='REPLAY' and c['commit_success'] for c in chunks)
