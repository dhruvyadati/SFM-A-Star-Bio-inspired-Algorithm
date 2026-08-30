"""Synthetic-data test for the dose-response analysis.

This exists because a silent bug survived visual inspection: `rate_goal` was
resolved as a category lookup, but goal arrival is not a category (it is safe
OR unsafe), so every goal-arrival rate would have printed 0.000 and every
mobility-cost onset would have fired at the first level. The tables would have
looked entirely plausible.

The test plants a KNOWN dose-response pattern and asserts the analysis recovers
it. A summary that cannot recover a planted effect cannot be trusted to report
a real one.

Run: python3 py/Test_AnalyzeDose.py
"""
from __future__ import annotations

import json
import random
import subprocess
import sys
import tempfile
from pathlib import Path

LEVELS = {
    "ttcHorizon": [1.0, 1.5, 2.0, 3.0],
    "ttcStop": [0.4, 0.8, 1.2],
    "ttcMinGap": [0.20, 0.40, 0.60],
}


def synth(path, n_scen=60, seed=3):
    """Planted pattern: stronger intervention -> more activation, more
    deadlock, less goal arrival; unguided starts far closer to deadlock so it
    should cross the mobility-cost onset at a weaker level."""
    rng = random.Random(seed)
    # MUST mirror the real generator's shared-centre labelling: dose_response.py
    # runs the three sweeps' common centre ONCE under "ttcHorizon=2.0" rather
    # than emitting "ttcStop=0.8" and "ttcMinGap=0.4" separately. Emitting them
    # separately makes analyze_dose's one-factor assertion fail, which is what
    # happened when the centre fix landed and this test was not re-run.
    DEFAULTS = {"ttcHorizon": 2.0, "ttcStop": 0.8, "ttcMinGap": 0.40}
    cfgs = ["no-filter"] + [f"{f}={lv}" for f, L in LEVELS.items()
                            for lv in L if lv != DEFAULTS[f]] + ["ttcHorizon=2.0"]
    rows = []
    for i in range(n_scen):
        s = f"conf_synth_{i}"
        for pl in ("hybrid", "sfm"):
            for c in cfgs:
                strength = 0.0 if c == "no-filter" else float(c.split("=")[1])
                dead = 0.02 + 0.25 * strength / 3 + (0.60 if pl == "sfm" else 0.0)
                if rng.random() < min(dead, 0.95):
                    cat, out, nc = "deadlock", "deadlock", 0
                elif rng.random() < 0.5:
                    cat, out, nc = "safe", "success", 0
                else:
                    cat, out, nc = "unsafe", "success", 1
                act = 0.0 if c == "no-filter" else min(0.5, 0.05 + 0.12 * strength)
                rows.append(dict(
                    scenario=s, planner=pl, config=c, category=cat, outcome=out,
                    contacts=nc, steps=5000, stop_time=1.0 + 3 * strength,
                    mean_speed=0.6, path_length=70.0, ttc_active_frac=act,
                    first_activation_step=-1 if c == "no-filter"
                    else max(1, int(400 - 60 * strength)),
                    sustained_activation_step=-1,
                    mean_speed_active=float("nan") if c == "no-filter"
                    else max(0.05, 0.40 - 0.08 * strength),
                    mean_brake_active=float("nan") if c == "no-filter"
                    else max(0.05, 0.80 - 0.15 * strength)))
    Path(path).write_text(json.dumps(rows))
    return len(rows)


def main():
    here = Path(__file__).parent
    with tempfile.TemporaryDirectory() as td:
        fp = Path(td) / "synth_dose.json"
        n = synth(fp)
        r = subprocess.run([sys.executable, str(here / "analyze_dose.py"), str(fp)],
                           capture_output=True, text=True, cwd=here.parent)
        out = r.stdout
        if r.returncode != 0:
            print(out[-3000:], r.stderr[-3000:])
            raise SystemExit("analyze_dose.py FAILED to run")

        checks = [
            ("runs without error", r.returncode == 0),
            ("structural validation ran", "one-factor-at-a-time labelling: OK" in out),
            ("category rates sum to 1", "category rates sum to 1 in every cell: OK" in out),
            # The planted pattern is monotone, so no L1 CONTROLLER metric may
            # be flagged COUNTER. A COUNTER here means the direction convention
            # or the rank correlation is wrong, not that the data is
            # interesting.
            ("no spurious L1 COUNTER", not _counter_in_level(out, "L1 CONTROLLER")),
            # The rate_goal bug made every goal-arrival rate exactly zero.
            ("goal-arrival rate is not identically zero",
             "goal-arrival rate" in out and not _block_all_zero(out, "goal-arrival rate")),
            ("prediction 7 reached a verdict", "=> " in out),
            ("onset logic fired at least once", "<- onset" in out),
        ]
        bad = [name for name, ok in checks if not ok]
        for name, ok in checks:
            print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        print(f"\n{n} synthetic runs, {len(checks) - len(bad)}/{len(checks)} checks passed")
        if bad:
            raise SystemExit("FAILED: " + "; ".join(bad))
        print("Test_AnalyzeDose: PASS")


def _block_all_zero(out, label):
    """True if every FINITE value in the labelled metric's BLOCK is zero.

    The metric label now heads a block, with one indented line per level and a
    trailing `->` verdict, so a single-line scan finds no numbers at all and
    would vacuously "pass". This walks the block instead.

    Non-finite values must be excluded. With the rate_goal bug live the values
    are all 0.000 alongside a NaN rho, and `nan != 0.0` evaluates True, so a
    naive scan treats the NaN as evidence of a non-zero rate and passes while
    the defect is present. This helper was written that way first; deliberately
    reintroducing the bug is what exposed it.
    """
    lines = out.splitlines()
    found_any = False
    for i, line in enumerate(lines):
        if line.strip() != label:
            continue
        for nxt in lines[i + 1:]:
            s = nxt.strip()
            if not s or s.startswith("->"):
                break
            vals = [float(t.strip("[],")) for t in s.split()
                    if _isnum(t.strip("[],"))]
            finite = [v for v in vals if v == v and abs(v) != float("inf")]
            if finite:
                found_any = True
                if any(v != 0.0 for v in finite):
                    return False
    # No numbers found at all means the block was never located: treat as
    # all-zero so the check fails loudly rather than passing vacuously.
    return True if found_any else True


def _counter_in_level(out, level_title):
    """Does a COUNTER verdict appear inside the named evidence level?"""
    inside = False
    for line in out.splitlines():
        s = line.strip()
        if s.startswith("L1 ") or s.startswith("L2 ") or s.startswith("L3 "):
            inside = s.startswith(level_title)
            continue
        if inside and "COUNTER" in s:
            return True
    return False


def _isnum(t):
    try:
        float(t)
        return True
    except ValueError:
        return False


if __name__ == "__main__":
    main()
