"""Integration fixture set: 10 targeted traces before any calibration.

Run: python3 py/integration_fixtures.py

THIS IS AN ENGINEERING INTEGRATION CHECK. It is not calibration and selects no
parameter. Its purpose is to confirm the wired system behaves sensibly across
the regimes the controller will meet, with traces archived rather than only
aggregate outcomes -- an aggregate cannot show that the right thing happened
for the right reason.

Configurations here are chosen to EXERCISE regimes, not to perform well. Two
deliberately use degenerate values (`theta_prog=1e9`, `max_episodes=0`) that
the sweep will never use, because boundary values expose errors that realistic
ones conceal: the episode cap was enforced one episode late and only a cap of
zero revealed it.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from sfm_explore import Scenario, run_trial
from recovery import RecoveryConfig

FCAL = Path("v2/runs/v2/export_fcal")
OUT = Path("v2/runs/v2/integration_fixtures.json")
TRACE_DIR = Path("v2/runs/v2/integration_traces")

STAGE_A = dict(window=3.0, theta_act=0.80, theta_prog=0.20, relax=0.60,
               exit_hold=1.0, max_recovery=3.0, max_episodes=2, cooldown=2.0)


def cfg(**kw):
    return RecoveryConfig(**{**STAGE_A, **kw})


# (label, what it exercises, ttc on, recovery config or None)
CASES = [
    ("no_ttc", "no filter at all: recovery must be absent end-to-end",
     False, None),
    ("ttc_no_recovery", "reference arm", True, None),
    ("recovery_disabled", "infrastructure present, switched off",
     True, cfg(enabled=False)),
    ("recovery_never_triggers", "live code path, trigger unreachable",
     True, cfg(theta_act=2.0)),
    ("stage_a_defaults", "the configuration screening will actually start from",
     True, cfg()),
    ("always_on", "recovery active almost always: emergency interaction",
     True, cfg(theta_act=0.0, theta_prog=1e9, max_recovery=60.0,
               max_episodes=999, cooldown=0.0)),
    ("cap_zero", "retired before acting: must equal plain TTC",
     True, cfg(max_episodes=0, theta_act=0.0, theta_prog=1e9)),
    ("cap_one", "single episode then retirement",
     True, cfg(max_episodes=1, theta_act=0.0, theta_prog=1e9)),
    ("aggressive_release", "full release of the discretionary brake",
     True, cfg(relax=1.0)),
    ("tight_trigger", "strict progress demand, short window",
     True, cfg(window=2.0, theta_prog=0.30)),
]


def summarise(label, why, r, tr):
    d = dict(fixture=label, exercises=why, outcome=r.outcome, steps=int(r.steps),
             contacts=int(r.ped_collisions + r.obs_collisions),
             min_ped_gap=float(r.min_ped_gap), stop_time=float(r.stop_time),
             ttc_active_frac=float(r.ttc_active_frac),
             recovery_episodes=int(r.recovery_episodes),
             recovery_active_steps=int(r.recovery_active_steps),
             release_integral=float(r.recovery_release_integral),
             ended_progress=int(r.recovery_ended_progress),
             ended_duration=int(r.recovery_ended_duration),
             suppressed=int(r.recovery_suppressed),
             exhausted=bool(r.recovery_exhausted),
             mandatory_steps=int(r.mandatory_override_steps))
    if tr:
        mand = [e for e in tr if e["mandatory"]]
        d["trace_steps"] = len(tr)
        d["mandatory_in_episode"] = sum(1 for e in mand if e["recovery_active"])
        d["max_release"] = max((e["release"] for e in tr), default=0.0)
        d["precedence_violations"] = sum(
            1 for e in mand if e["applied_scale"] != e["ttc_scale"])
    return d


if __name__ == "__main__":
    if not FCAL.exists():
        sys.exit(f"{FCAL} not found")
    scen = sorted(FCAL.glob("*.json"))[1]      # the pinned emergency fixture
    TRACE_DIR.mkdir(parents=True, exist_ok=True)
    print(f"scenario: {scen.name}\n")

    rows, ref = [], None
    for label, why, ttc, rc in CASES:
        tr = [] if rc is not None else None
        R = run_trial(Scenario.load(str(scen)), "hybrid", ttc=ttc,
                      recovery_cfg=rc, trace=tr)
        row = summarise(label, why, R, tr)
        if label == "ttc_no_recovery":
            ref = row
        # Inertness expectations, asserted rather than eyeballed.
        if label in ("recovery_disabled", "recovery_never_triggers", "cap_zero"):
            row["matches_reference"] = (ref is not None
                                        and row["steps"] == ref["steps"]
                                        and row["contacts"] == ref["contacts"]
                                        and row["outcome"] == ref["outcome"])
        rows.append(row)
        if tr:
            (TRACE_DIR / f"{label}.json").write_text(json.dumps(tr))
        print(f"  {label:<24} {R.outcome:<9} steps={R.steps:<6} "
              f"eps={R.recovery_episodes} rel={R.recovery_release_integral:7.3f} "
              f"mand={R.mandatory_override_steps}")

    OUT.write_text(json.dumps(rows, indent=1))
    print(f"\nwrote {OUT} and {TRACE_DIR}/")

    bad = [r for r in rows if r.get("precedence_violations")]
    inert = [r for r in rows if r.get("matches_reference") is False]
    print(f"\n  precedence violations : {sum(r.get('precedence_violations', 0) for r in rows)}")
    print(f"  inertness failures    : {[r['fixture'] for r in inert]}")
    sys.exit(1 if bad or inert else 0)
