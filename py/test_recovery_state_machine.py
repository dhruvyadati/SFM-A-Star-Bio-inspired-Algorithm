"""State-machine tests for ProgressAwareRelease.

Run: python3 -m pytest py/test_recovery_state_machine.py -q

Packaged because throwaway checks are not an audit trail, and because two real
defects were found here that unit-level reasoning alone did not surface:

  * a live re-arm counter could never satisfy itself, since the trigger requires
    progress BELOW threshold while re-arm requires progress ABOVE it -- so no
    second episode could ever start (fixed by latching);
  * the episode cap was evaluated only when an episode ENDED, enforcing it one
    episode late: correct for cap >= 1, but a cap of 0 still ran one episode
    (fixed by checking the budget at trigger time).
"""
import sys
from dataclasses import dataclass
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from recovery import ProgressAwareRelease, RecoveryConfig


@dataclass
class P:
    dt: float = 0.01


def cfg(**kw):
    base = dict(window=1.0, theta_act=0.8, theta_prog=0.2, relax=0.6,
                exit_hold=1.0, max_recovery=1.0, max_episodes=2, cooldown=2.0)
    base.update(kw)
    return RecoveryConfig(**base)


def run(c, phases, hard=False):
    """phases = [(steps, filter_active, arc_consumed_per_step), ...]"""
    r = ProgressAwareRelease(P(), c)
    s, released = 50.0, 0
    for steps, active, rate in phases:
        for _ in range(steps):
            s -= rate
            out = r.update(0.0 if active else 1.0, active, s, hard)
            if active and out > 0.0:
                released += 1
    return r, released


STALL = [(3000, True, 0.0)]
CYCLE = [(300, True, 0.0), (600, False, 0.05)] * 4


def test_braking_with_progress_never_triggers():
    r, _ = run(cfg(), [(2000, True, 0.05)])
    assert r.episodes == 0


def test_stalled_and_braking_triggers():
    r, rel = run(cfg(), STALL)
    assert r.episodes >= 1 and rel > 0


def test_stalled_without_braking_never_triggers():
    """Dense-crowd congestion with no filter involvement is not the target."""
    r, _ = run(cfg(), [(2000, False, 0.0)])
    assert r.episodes == 0


def test_hard_gap_override_is_absolute():
    r, rel = run(cfg(), STALL, hard=True)
    assert rel == 0


def test_permanent_stall_yields_exactly_one_episode():
    """A timer-based cooldown would manufacture repeated episodes here."""
    r, _ = run(cfg(), STALL)
    assert r.episodes == 1


def test_rearm_requires_demonstrated_progress():
    r, _ = run(cfg(), [(300, True, 0.0), (600, False, 0.05), (300, True, 0.0)])
    assert r.episodes == 2


def test_brief_progress_below_cooldown_does_not_rearm():
    r, _ = run(cfg(), [(300, True, 0.0), (100, False, 0.05), (300, True, 0.0)])
    assert r.episodes == 1


@pytest.mark.parametrize("cap", [0, 1, 2, 3])
def test_episode_cap_is_exact(cap):
    r, _ = run(cfg(max_episodes=cap), CYCLE)
    assert r.episodes == cap


def test_cap_zero_releases_nothing():
    r, rel = run(cfg(max_episodes=0), STALL)
    assert rel == 0 and r.recovery_step_count == 0


def test_controller_retires_after_cap():
    r, _ = run(cfg(max_episodes=2), CYCLE)
    assert r.exhausted and r.suppressed > 0


def test_total_release_time_bounded_by_cap_times_duration():
    c = cfg(max_episodes=2, max_recovery=1.0)
    r, rel = run(c, CYCLE)
    assert rel * P.dt <= c.max_episodes * c.max_recovery + 1e-9


def test_every_episode_has_a_termination_reason():
    r, _ = run(cfg(max_episodes=2), CYCLE)
    assert r.ended_progress + r.ended_duration == r.episodes


def test_window_must_be_a_multiple_of_dt():
    with pytest.raises(ValueError):
        ProgressAwareRelease(P(), cfg(window=0.0035))


def test_window_spans_exactly_the_configured_duration():
    r = ProgressAwareRelease(P(), cfg(window=3.0))
    assert (r.dist_hist.maxlen - 1) * P.dt == pytest.approx(3.0)


def test_release_never_tightens_the_brake():
    r = ProgressAwareRelease(P(), cfg())
    for _ in range(500):
        out = r.update(0.4, True, 50.0, False)
        assert out >= 0.4 - 1e-12
