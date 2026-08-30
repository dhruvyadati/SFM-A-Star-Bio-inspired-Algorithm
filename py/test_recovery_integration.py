"""End-to-end integration tests for the recovery layer.

Run: python3 -m pytest py/test_recovery_integration.py -q

These are the gate before any calibration sweep. Unit tests establish that the
state machine behaves as specified; only these establish that wiring it into
the simulation changed nothing it was not supposed to change.

Six properties, each of which would invalidate the sweep if false:

  1. DISABLED INERTNESS  recovery infrastructure present but off is
     field-by-field identical to ordinary hybrid-TTC.
  2. NO-TRIGGER INERTNESS  enabled but never triggering is also identical.
     Distinct from (1): it exercises the live code path.
  3. EMERGENCY PRECEDENCE  whenever a mandatory flag holds, the applied scale
     equals the TTC command exactly -- tested DURING an active episode, not
     only from idle.
  4. ACCOUNTING IDENTITY  logged release, episodes and terminations reconcile
     with the trace.
  5. DETERMINISTIC REPLAY  identical inputs give identical complete results.
  6. CAP-EXHAUSTION INERTNESS  after the controller retires it behaves exactly
     like standard TTC for the remainder of the trial.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
from sfm_explore import Scenario, run_trial
from recovery import RecoveryConfig

FCAL = Path("v2/runs/v2/export_fcal")
CONF = Path("v2/runs/v2/export_confirm")

# Stage A screening defaults. NOT selected values.
STAGE_A = dict(window=3.0, theta_act=0.80, theta_prog=0.20, relax=0.60,
               exit_hold=1.0, max_recovery=3.0, max_episodes=2, cooldown=2.0)

SHARED = ["outcome", "steps", "ped_collisions", "obs_collisions", "path_length",
          "stop_time", "mean_speed", "min_ped_gap", "min_obs_gap",
          "ttc_active_frac", "first_activation_step",
          "sustained_activation_step", "stop_episodes"]


def scenarios(n=4):
    for d in (FCAL, CONF):
        if d.exists():
            ps = sorted(d.glob("*.json"))[:n]
            if ps:
                return ps
    pytest.skip("no scenario files available")


def fields(r):
    return {k: getattr(r, k) for k in SHARED}


def test_1_disabled_inertness():
    """Recovery present but disabled must not perturb anything."""
    for p in scenarios():
        S = Scenario.load(str(p))
        base = run_trial(S, "hybrid", ttc=True)
        off = run_trial(Scenario.load(str(p)), "hybrid", ttc=True,
                        recovery_cfg=RecoveryConfig(**{**STAGE_A, "enabled": False}))
        assert fields(base) == fields(off), f"{p.name} perturbed while disabled"
        assert off.recovery_episodes == 0


def test_2_no_trigger_inertness():
    """Enabled but never triggering must also be identical.

    Achieved with an unreachable activation threshold, so the live code path
    runs every step and only the trigger is suppressed.
    """
    cfg = RecoveryConfig(**{**STAGE_A, "theta_act": 2.0})
    for p in scenarios():
        S = Scenario.load(str(p))
        base = run_trial(S, "hybrid", ttc=True)
        on = run_trial(Scenario.load(str(p)), "hybrid", ttc=True, recovery_cfg=cfg)
        assert on.recovery_episodes == 0
        assert fields(base) == fields(on), f"{p.name} perturbed without triggering"


# A scenario KNOWN to produce mandatory emergency states while a recovery
# episode is active. Pinned so the most safety-critical interaction cannot
# silently skip. Located by scanning the calibration set; the first attempt at
# that scan reported zero because `theta_prog=1e9` also makes re-arm
# unreachable, so recovery ran ~3 s of a ~70 s trial instead of always-on.
# `cooldown=0` is what actually keeps it active.
EMERGENCY_FIXTURE = "fcal_baseline_2147483649.json"
ALWAYS_ON = dict(STAGE_A, theta_act=0.0, theta_prog=1e9,
                 max_recovery=60.0, max_episodes=999, cooldown=0.0)


def test_3_emergency_precedence_including_mid_episode():
    """A mandatory flag must force applied == commanded, even mid-episode.

    This is the interaction most likely to hide a wiring error: recovery is
    actively releasing when an emergency arrives, so precedence must preempt an
    in-flight release rather than merely prevent a new one.
    """
    fx = FCAL / EMERGENCY_FIXTURE
    if not fx.exists():
        pytest.skip(f"{EMERGENCY_FIXTURE} absent")
    tr = []
    run_trial(Scenario.load(str(fx)), "hybrid", ttc=True,
              recovery_cfg=RecoveryConfig(**ALWAYS_ON), trace=tr)
    mid = [e for e in tr if e["mandatory"] and e["recovery_active"]]
    assert mid, "fixture no longer produces the interaction — reselect it"
    for e in tr:
        if e["mandatory"]:
            assert e["applied_scale"] == e["ttc_scale"], (
                f"emergency relaxed at step {e['step']}: "
                f"{e['ttc_scale']} -> {e['applied_scale']}")
    assert all(e["release"] == 0.0 for e in mid)


def test_4_accounting_identity():
    cfg = RecoveryConfig(**{**STAGE_A, "theta_act": 0.0, "theta_prog": 1e9})
    for p in scenarios():
        tr = []
        r = run_trial(Scenario.load(str(p)), "hybrid", ttc=True,
                      recovery_cfg=cfg, trace=tr)
        rel = sum(e["release"] for e in tr if e["recovery_active"]) * 0.01
        assert r.recovery_release_integral == pytest.approx(rel, abs=1e-9)
        assert r.recovery_active_steps == sum(1 for e in tr if e["recovery_active"])
        assert r.recovery_episodes == (r.recovery_ended_progress
                                       + r.recovery_ended_duration
                                       + (1 if tr and tr[-1]["recovery_active"] else 0))
        assert r.recovery_episodes <= cfg.max_episodes
        # Release only ever loosens, never tightens.
        assert all(e["release"] >= -1e-12 for e in tr)


def test_5_deterministic_replay():
    cfg = RecoveryConfig(**STAGE_A)
    for p in scenarios(3):
        a = run_trial(Scenario.load(str(p)), "hybrid", ttc=True, recovery_cfg=cfg)
        b = run_trial(Scenario.load(str(p)), "hybrid", ttc=True, recovery_cfg=cfg)
        assert fields(a) == fields(b)
        assert a.recovery_episodes == b.recovery_episodes
        assert a.recovery_release_integral == b.recovery_release_integral


def test_5b_arm_isolation_and_order_independence():
    """Running configurations in a different order must not change results."""
    cfgs = [RecoveryConfig(**{**STAGE_A, "relax": r}) for r in (0.4, 0.6, 0.8)]
    p = scenarios(1)[0]
    fwd = [run_trial(Scenario.load(str(p)), "hybrid", ttc=True, recovery_cfg=c)
           for c in cfgs]
    rev = [run_trial(Scenario.load(str(p)), "hybrid", ttc=True, recovery_cfg=c)
           for c in reversed(cfgs)][::-1]
    for a, b in zip(fwd, rev):
        assert fields(a) == fields(b)


def test_6_cap_exhaustion_inertness():
    """After retirement the controller must behave exactly like standard TTC.

    A cap of zero retires it before it can ever act, so the whole trial must
    match plain hybrid-TTC.
    """
    cfg = RecoveryConfig(**{**STAGE_A, "max_episodes": 0,
                            "theta_act": 0.0, "theta_prog": 1e9})
    for p in scenarios():
        base = run_trial(Scenario.load(str(p)), "hybrid", ttc=True)
        spent = run_trial(Scenario.load(str(p)), "hybrid", ttc=True, recovery_cfg=cfg)
        assert spent.recovery_episodes == 0
        assert spent.recovery_release_integral == 0.0
        assert fields(base) == fields(spent), f"{p.name} perturbed after exhaustion"
