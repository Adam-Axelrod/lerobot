"""ACT options added for the Meca500 microscope task: precision-step loss weighting and joint-state
corruption. The model is replaced by fixed predictions so the loss arithmetic is checked exactly."""

import torch
from torch import nn

from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.utils.constants import ACTION, OBS_STATE


class _FixedModel(nn.Module):
    def __init__(self, actions_hat):
        super().__init__()
        self.actions_hat = actions_hat
        self.seen_state = None

    def forward(self, batch):
        self.seen_state = batch[OBS_STATE].clone()
        return self.actions_hat, (None, None)


def _policy(**kwargs):
    policy = ACTPolicy.__new__(ACTPolicy)
    nn.Module.__init__(policy)
    policy.config = ACTConfig(use_vae=False, **kwargs)
    return policy


def _batch():
    # 1 sample, 4 steps, 2 action dims (joint, precision flag normalised: >0 is on)
    action = torch.tensor([[[0.0, -1.0], [0.0, -1.0], [0.0, 1.0], [0.0, 1.0]]])
    return {
        ACTION: action,
        "action_is_pad": torch.zeros(1, 4, dtype=torch.bool),
        OBS_STATE: torch.ones(1, 3),
    }


def test_default_loss_is_unchanged_plain_l1():
    policy = _policy()
    hat = _batch()[ACTION].clone()
    hat[0, :, 0] = torch.tensor([1.0, 1.0, 0.1, 0.1])  # coarse steps wrong by 1, precise by 0.1
    policy.model = _FixedModel(hat)
    loss, info = policy.forward(_batch())
    torch.testing.assert_close(loss, torch.tensor((1 + 1 + 0.1 + 0.1) / 8))
    assert "weighted_l1_loss" not in info


def test_precision_weight_scales_the_precise_steps_only():
    policy = _policy(precision_loss_weight=10.0)
    hat = _batch()[ACTION].clone()
    hat[0, :, 0] = torch.tensor([1.0, 1.0, 0.1, 0.1])
    policy.model = _FixedModel(hat)
    loss, info = policy.forward(_batch())
    # weighted mean over steps (weights 1, 1, 10, 10), per action dim
    expected = (1 + 1 + 10 * 0.1 + 10 * 0.1) / ((1 + 1 + 10 + 10) * 2)
    torch.testing.assert_close(loss, torch.tensor(expected))
    torch.testing.assert_close(torch.tensor(info["l1_loss"]), torch.tensor((1 + 1 + 0.1 + 0.1) / 8))


def test_state_corruption_only_in_training():
    policy = _policy(state_dropout=1.0)
    policy.model = _FixedModel(_batch()[ACTION].clone())
    policy.train()
    policy.forward(_batch())
    assert torch.all(policy.model.seen_state == 0)  # dropped to the dataset mean
    policy.eval()
    policy.forward(_batch())
    assert torch.all(policy.model.seen_state == 1)  # untouched outside training

    noisy = _policy(state_noise_std=0.5)
    noisy.model = _FixedModel(_batch()[ACTION].clone())
    noisy.train()
    noisy.forward(_batch())
    assert not torch.allclose(noisy.model.seen_state, torch.ones(1, 3))


def test_state_slide_moves_the_state_along_the_basis_within_the_radius():
    # joint 0 moves 1 deg per mm of x, joint 1 2 deg per mm of y; state std 2 deg for both
    basis = torch.tensor([[1.0, 0.0], [0.0, 2.0]]).reshape(1, 4).repeat(512, 1)
    batch = {
        ACTION: torch.zeros(512, 4, 2),
        "action_is_pad": torch.zeros(512, 4, dtype=torch.bool),
        OBS_STATE: torch.zeros(512, 2),
        "slide_basis": basis,
    }
    policy = _policy(state_slide_mm=3.0, state_slide_std=[2.0, 2.0])
    policy.model = _FixedModel(torch.zeros(512, 4, 2))
    policy.train()
    policy.forward(batch)
    dx = -policy.model.seen_state[:, 0] * 2.0 / 1.0  # back to mm
    dy = -policy.model.seen_state[:, 1] * 2.0 / 2.0
    radius = (dx**2 + dy**2).sqrt()
    assert radius.max() <= 3.0 + 1e-5 and radius.max() > 2.5
    assert dx.std() > 0.5 and dy.std() > 0.5  # both directions, not one axis
    policy.eval()
    policy.forward(batch)
    assert torch.all(policy.model.seen_state == 0)  # inference never slides
