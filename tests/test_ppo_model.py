"""Minimal optimizer check; physical success is checked by mounted rollouts."""
import numpy as np
import pytest

torch = pytest.importorskip('torch')
from multipath.anchors.ppo_model import SmallResidualPPO


def test_small_residual_ppo_learns_and_saves(tmp_path):
    torch.set_num_threads(1)
    model = SmallResidualPPO(24)
    assert model.parameter_count < 20000
    np.testing.assert_array_equal(model.act(np.zeros(24), deterministic=True)[0], np.zeros(18))
    episodes = []
    for _ in range(2):
        episode = []
        for i in range(12):
            obs = np.full(24, i / 12, dtype=np.float32)
            action, info = model.act(obs)
            assert np.isfinite(action).all() and (np.abs(action) <= 1.).all()
            episode.append(dict(obs=obs, **info, reward=-float(np.square(action - .25).sum()),
                                done=i == 11))
        episodes.append(episode)
    before = model.actor[-1].weight.detach().clone()
    metrics = model.update(episodes)
    assert all(np.isfinite(value) for value in metrics.values())
    assert not torch.equal(before, model.actor[-1].weight)
    checkpoint = tmp_path / 'small_ppo.pt'
    model.save(checkpoint, {'physically_validated': False})
    saved = torch.load(checkpoint, weights_only=True)
    assert saved['hidden_sizes'] == [64, 64]
    assert saved['metadata']['physically_validated'] is False


def test_nonfinite_and_incomplete_data_rejected():
    model = SmallResidualPPO(3)
    with pytest.raises(ValueError, match='finite observation'):
        model.act([0., np.nan, 0.])
    _, info = model.act(np.zeros(3))
    tr = dict(obs=np.zeros(3), **info, reward=0., done=False)
    with pytest.raises(ValueError, match='terminal'):
        model.update([[tr]])
    with pytest.raises(ValueError, match='Nonfinite'):
        model.update([[dict(tr, reward=np.nan, done=True)]])
