"""The delta conversion must be exactly invertible.

Integrating the per-step deltas back onto the starting pose has to reproduce the
original absolute trajectory, because that integration is literally what
``Meca500.send_action`` does at deployment when ``relative_actions=True``. If the
two disagree, the arm traces a different path from the one that was demonstrated -
and it would do so silently, which is the failure mode this fork keeps hitting.
"""

import numpy as np
import pytest

JOINTS = 6


def _difference(actions: np.ndarray) -> np.ndarray:
    """Forward per-step increments for one episode; the last frame holds."""
    out = actions.copy()
    out[:, :JOINTS] = np.vstack(
        [np.diff(actions[:, :JOINTS], axis=0), np.zeros((1, JOINTS), dtype=actions.dtype)]
    )
    return out


def _integrate(deltas: np.ndarray, start: np.ndarray) -> np.ndarray:
    """Replay the deltas from a starting pose, as send_action does step by step."""
    out = np.zeros_like(deltas)
    pos = start.astype(np.float64).copy()
    for k in range(len(deltas)):
        out[k, :JOINTS] = pos
        pos = pos + deltas[k, :JOINTS]
    out[:, JOINTS:] = deltas[:, JOINTS:]
    return out


def test_difference_then_integrate_round_trips():
    rng = np.random.default_rng(0)
    # A trajectory with the same character as a demo: coarse approach then a fine
    # phase whose steps are ~100x smaller, which is where precision is lost if the
    # conversion is done in float32 or with a sloppy accumulation.
    coarse = np.cumsum(rng.normal(0, 0.5, size=(200, JOINTS)), axis=0) + 40.0
    fine = coarse[-1] + np.cumsum(rng.normal(0, 0.0045, size=(200, JOINTS)), axis=0)
    joints = np.vstack([coarse, fine])
    gate = np.r_[np.zeros(200), np.ones(200)][:, None]
    actions = np.hstack([joints, gate])

    deltas = _difference(actions)
    rebuilt = _integrate(deltas, actions[0, :JOINTS])

    np.testing.assert_allclose(rebuilt[:, :JOINTS], actions[:, :JOINTS], atol=1e-9)
    # The gate is a gate, not a position: it must survive untouched.
    np.testing.assert_array_equal(rebuilt[:, JOINTS:], actions[:, JOINTS:])


def test_gate_channel_is_not_differenced():
    actions = np.hstack([np.zeros((10, JOINTS)), np.r_[np.zeros(5), np.ones(5)][:, None]])
    deltas = _difference(actions)
    # A differenced gate would be zero everywhere except a single +1 spike.
    np.testing.assert_array_equal(deltas[:, JOINTS], actions[:, JOINTS])


def test_final_frame_holds():
    actions = np.hstack([np.arange(10, dtype=float)[:, None].repeat(JOINTS, 1), np.zeros((10, 1))])
    deltas = _difference(actions)
    assert np.all(deltas[-1, :JOINTS] == 0.0), "last frame must command no motion"
    assert np.all(deltas[:-1, :JOINTS] == 1.0)


def _lift(actions: np.ndarray) -> tuple[float, float]:
    """How large a 0.0045 deg fine step is, relative to each representation's own scale."""
    fine_step = 0.0045
    deltas = _difference(actions)
    return (
        fine_step / actions[:, :JOINTS].std(axis=0).mean(),
        fine_step / deltas[:, :JOINTS].std(axis=0).mean(),
    )


def test_fine_steps_survive_the_representation():
    """The mechanism: differencing lifts a fine step relative to the normalizer.

    The synthetic trajectory below only spreads over ~3 deg, where the real rig spreads
    over ~25, so the lift here is far smaller than on real data. This asserts only that
    the mechanism works; test_real_dataset_lift pins the magnitude that actually matters.
    """
    rng = np.random.default_rng(1)
    coarse = np.cumsum(rng.normal(0, 0.5, size=(500, JOINTS)), axis=0)
    fine = coarse[-1] + np.cumsum(rng.normal(0, 0.0045, size=(500, JOINTS)), axis=0)
    actions = np.hstack([np.vstack([coarse, fine]), np.zeros((1000, 1))])

    abs_ratio, delta_ratio = _lift(actions)
    assert delta_ratio > 5 * abs_ratio, (
        f"differencing should lift the fine step several-fold "
        f"(got {delta_ratio / abs_ratio:.1f}x: {abs_ratio:.2e} -> {delta_ratio:.2e})"
    )


@pytest.mark.parametrize("repo", ["AdamAxelrod/microscope_pipette_2026-09-10"])
def test_real_dataset_round_trips(repo):
    """Round-trip the actual recorded data, if it is present on this machine."""
    import glob
    from pathlib import Path

    import pandas as pd

    from lerobot.utils.constants import HF_LEROBOT_HOME

    root = Path(HF_LEROBOT_HOME) / repo
    files = sorted(glob.glob(str(root / "data" / "**" / "*.parquet"), recursive=True))
    if not files:
        pytest.skip(f"{repo} not downloaded on this machine")

    df = pd.concat([pd.read_parquet(f) for f in files], ignore_index=True)
    act = np.stack(df["action"].to_numpy()).astype(np.float64)
    ep = df["episode_index"].to_numpy()

    for e in np.unique(ep)[:10]:
        block = act[ep == e]
        rebuilt = _integrate(_difference(block), block[0, :JOINTS])
        np.testing.assert_allclose(
            rebuilt[:, :JOINTS], block[:, :JOINTS], atol=1e-6, err_msg=f"episode {e} did not round-trip"
        )


@pytest.mark.parametrize("repo", ["AdamAxelrod/microscope_pipette_2026-09-10"])
def test_real_dataset_lift(repo):
    """On the real rig the lift is ~86x, which is what puts a fine step above the floor.

    The trained policy's measured output floor is ~0.010 deg/step (0.0% of rollout steps
    commanded exactly zero, median nonzero step 0.010). A 0.0045 deg fine correction is
    below that in absolute representation and above it in delta representation - the
    whole reason for this conversion.
    """
    import glob
    from pathlib import Path

    import pandas as pd

    from lerobot.utils.constants import HF_LEROBOT_HOME

    files = sorted(glob.glob(str(Path(HF_LEROBOT_HOME) / repo / "data" / "**" / "*.parquet"), recursive=True))
    if not files:
        pytest.skip(f"{repo} not downloaded on this machine")

    df = pd.concat([pd.read_parquet(f) for f in files], ignore_index=True)
    act = np.stack(df["action"].to_numpy()).astype(np.float64)
    ep = df["episode_index"].to_numpy()

    # Difference WITHIN episodes. Differencing the concatenated array would treat each
    # reset jump between episodes as a single enormous step and inflate the delta std
    # roughly eightfold, which silently understates the lift.
    deltas = np.concatenate([_difference(act[ep == e]) for e in np.unique(ep)])
    fine_step = 0.0045
    abs_ratio = fine_step / act[:, :JOINTS].std(axis=0).mean()
    delta_ratio = fine_step / deltas[:, :JOINTS].std(axis=0).mean()
    assert delta_ratio / abs_ratio > 20, (
        f"expected a >20x lift on real data, got {delta_ratio / abs_ratio:.1f}x "
        f"({abs_ratio:.2e} -> {delta_ratio:.2e})"
    )
