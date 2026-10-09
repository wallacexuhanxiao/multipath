"""Small CPU residual PPO for overfitting a single local interaction.

Actions are tanh-squashed Gaussian samples. PPO uses their stored *latent*
Gaussian likelihoods: the tanh Jacobian is the same for the old and new
policies at a sampled action and therefore cancels in the likelihood ratio.
The entropy bonus is explicitly the latent Gaussian entropy.
"""
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.distributions import Normal


def _network(obs_dim, out_dim):
    return nn.Sequential(nn.Linear(obs_dim, 64), nn.ELU(),
                         nn.Linear(64, 64), nn.ELU(), nn.Linear(64, out_dim))


class SmallResidualPPO(nn.Module):
    """Separate 64x64 actor/critic, with a zero-residual initial actor mean."""

    def __init__(self, obs_dim, action_dim=18, seed=0):
        super().__init__()
        self.obs_dim, self.action_dim = int(obs_dim), int(action_dim)
        if self.obs_dim <= 0 or self.action_dim <= 0:
            raise ValueError('Observation and action dimensions must be positive')
        self.seed = int(seed)
        with torch.random.fork_rng(devices=[]):
            torch.manual_seed(self.seed)
            self.actor = _network(self.obs_dim, self.action_dim)
            self.critic = _network(self.obs_dim, 1)
            nn.init.zeros_(self.actor[-1].weight)
            nn.init.zeros_(self.actor[-1].bias)
        self.log_std = nn.Parameter(torch.full((self.action_dim,), -1.0))
        self.register_buffer('exploration_scale',torch.tensor(1.))
        self.optimizer = torch.optim.Adam(self.parameters(), lr=3e-4)
        self.rng = torch.Generator(device='cpu').manual_seed(self.seed)
        self.updates = 0

    @property
    def parameter_count(self):
        return sum(p.numel() for p in self.parameters())

    def _distribution(self, obs):
        return Normal(self.actor(obs), self.log_std.clamp(-5., 1.).exp()*self.exploration_scale)

    def set_exploration_scale(self,scale):
        """One shared residual-noise schedule, fixed during each PPO batch."""
        self.exploration_scale.fill_(float(scale))

    @torch.no_grad()
    def act(self, obs, deterministic=False):
        obs = torch.as_tensor(np.asarray(obs), dtype=torch.float32)
        if obs.shape != (self.obs_dim,) or not torch.isfinite(obs).all():
            raise ValueError('Expected one finite observation vector')
        dist = self._distribution(obs)
        latent = dist.mean if deterministic else dist.mean + dist.stddev * torch.randn(
            (self.action_dim,), generator=self.rng)
        value = self.critic(obs).squeeze(-1)
        log_prob = dist.log_prob(latent).sum(-1)
        if not all(torch.isfinite(x).all() for x in (latent, value, log_prob)):
            raise FloatingPointError('Nonfinite PPO policy output')
        return latent.tanh().numpy(), {
            'latent': latent.numpy(), 'log_prob': float(log_prob), 'value': float(value)}

    def update(self, episodes):
        """Update from complete episodes; each must end in done=True.

        A source-window end is terminal for this local task. Do not silently
        bootstrap a truncated episode from an unrelated reset snapshot.
        """
        rows, advantages, returns = [], [], []
        for episode in episodes:
            if not episode:
                continue
            if not episode[-1]['done']:
                raise ValueError('PPO requires complete terminal local episodes')
            episode_adv = np.empty(len(episode), dtype=np.float32)
            gae, next_value = 0., 0.
            for i in reversed(range(len(episode))):
                tr = episode[i]
                live = 1. - float(bool(tr['done']))
                delta = float(tr['reward']) + .99 * next_value * live - float(tr['value'])
                gae = delta + .99 * .95 * live * gae
                episode_adv[i] = gae
                next_value = float(tr['value'])
            rows.extend(episode)
            advantages.extend(episode_adv)
            returns.extend(episode_adv + np.array([t['value'] for t in episode]))
        if not rows:
            raise ValueError('No PPO transitions supplied')
        tensor = lambda x: torch.as_tensor(np.asarray(x), dtype=torch.float32)
        obs = tensor([t['obs'] for t in rows])
        latent = tensor([t['latent'] for t in rows])
        old_log_prob = tensor([t['log_prob'] for t in rows])
        advantage, target = tensor(advantages), tensor(returns)
        if obs.shape != (len(rows), self.obs_dim) or latent.shape != (len(rows), self.action_dim):
            raise ValueError('PPO observation/action shape mismatch')
        if old_log_prob.shape != (len(rows),):
            raise ValueError('Expected one summed log probability per action')
        if not all(torch.isfinite(x).all() for x in (obs, latent, old_log_prob, advantage, target)):
            raise ValueError('Nonfinite PPO training data')
        advantage = (advantage - advantage.mean()) / advantage.std(unbiased=False).clamp_min(1e-8)
        history = []
        for _ in range(4):
            order = torch.randperm(len(rows), generator=self.rng)
            for idx in torch.tensor_split(order, min(4, len(rows))):
                dist = self._distribution(obs[idx])
                log_prob = dist.log_prob(latent[idx]).sum(-1)
                ratio = (log_prob - old_log_prob[idx]).exp()
                policy_loss = -torch.minimum(ratio * advantage[idx],
                                            ratio.clamp(.8, 1.2) * advantage[idx]).mean()
                value_loss = .5 * (self.critic(obs[idx]).squeeze(-1) - target[idx]).square().mean()
                entropy = dist.entropy().sum(-1).mean()
                loss = policy_loss + value_loss - .002 * entropy
                if not torch.isfinite(loss):
                    raise FloatingPointError('Nonfinite PPO loss')
                self.optimizer.zero_grad(set_to_none=True)
                loss.backward()
                grad_norm = nn.utils.clip_grad_norm_(self.parameters(), 1., error_if_nonfinite=True)
                self.optimizer.step()
                with torch.no_grad():
                    self.log_std.clamp_(-5., 1.)
                history.append([float(loss.detach()), float(policy_loss.detach()),
                                float(value_loss.detach()), float(entropy.detach()), float(grad_norm)])
        self.updates += 1
        means = np.mean(history, axis=0)
        return dict(zip(('loss', 'policy_loss', 'value_loss', 'latent_entropy', 'gradient_norm'),
                        map(float, means)), transitions=len(rows), episodes=sum(bool(e) for e in episodes),
                    updates=self.updates, parameter_count=self.parameter_count)

    def save(self, path, metadata):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        torch.save({'model': self.state_dict(), 'optimizer': self.optimizer.state_dict(),
                    'obs_dim': self.obs_dim, 'action_dim': self.action_dim,
                    'hidden_sizes': [64, 64], 'seed': self.seed, 'updates': self.updates,
                    'rng_state': self.rng.get_state(), 'metadata': metadata}, path)
