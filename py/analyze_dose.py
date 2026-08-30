"""Analysis of the dose-response sweep, following Amendments A.5, D and D.1.

Written BEFORE the sweep produced output. Every rule it applies -- the onset
definition, the relative 0.75x criterion, the predicted directions, the
separation of evidence levels -- was frozen beforehand and is reproduced here
rather than chosen to suit the numbers.

FOUR LEVELS OF EVIDENCE, reported separately and never merged. They are
nested: a failure at a later level does NOT invalidate an earlier one.

  L1 CONTROLLER    changing the parameter changes activation or braking as the
                   control law mechanically requires. Failure here WOULD
                   falsify the activation-to-slowing pathway.
  L2 BEHAVIOUR     stopping, deadlock and goal completion move in the
                   predicted direction.
  L3 SYSTEM        safe success shows the proposed trade-off / turning point.
  L4 ARCHITECTURE  the mobility-cost onset differs between guided and
                   unguided control (A.5 prediction 7).

Braking may rise monotonically at L1 while safe success stays noisy or
non-monotonic at L3. That is an expected consequence of nonlinear robot-crowd
dynamics, not a refutation, and the report says so next to the result.

THE NINE CONFIGURATIONS ARE NOT ONE SCALE. Each varies exactly one parameter
from the frozen defaults, so they form three separate ordered sweeps sharing a
common centre (ttcHorizon=2.0 is the default). Spearman is computed within a
factor only; `validate()` asserts the labelling still supports that reading.

SPEARMAN DOES NOT CARRY THE CLAIM. With three or four levels per factor a rank
coefficient is a direction summary and nothing more, so every level is also
reported as a raw estimate with a bootstrap interval. The curve is shown, not
just its sign.

STATUS: a paired follow-up experiment on the confirmed scenario set (D.1). Not
an independent replication.
"""
from __future__ import annotations

import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

SRC = Path(sys.argv[1] if len(sys.argv) > 1 else "v2/runs/v2/dose_response.json")
ARCH = Path("v2/runs/v2/confirm_factorial.json")
RNG = np.random.default_rng(9091)

LEVELS = {
    "ttcHorizon": [1.0, 1.5, 2.0, 3.0],
    "ttcStop": [0.4, 0.8, 1.2],
    "ttcMinGap": [0.20, 0.40, 0.60],
}
# For all three factors a LARGER value is a STRONGER intervention, so the
# predicted signs share one convention.
L1_CONTROLLER = [
    ("ttc_active_frac", "activation breadth", +1),
    ("first_act_s", "first activation (s)", -1),
    ("mean_brake_active", "brake factor while active", -1),
    ("mean_speed_active", "speed while active (m/s)", -1),
]
L2_BEHAVIOUR = [
    ("stop_time", "stationary time (s)", +1),
    ("rate_deadlock", "deadlock rate", +1),
    ("rate_goal", "goal-arrival rate", -1),
]
L3_SYSTEM = [("rate_safe", "safe-success rate", 0)]

# The three sweeps SHARE their centre point: at ttcStop=0.8 and ttcMinGap=0.40
# the parameter set is identical to ttcHorizon=2.0, which is the frozen default
# setting. dose_response.py runs that configuration ONCE and labels it
# "ttcHorizon=2.0" rather than simulating the same 500 scenarios three times.
# The analysis therefore has to resolve each factor's centre level back to that
# shared label. Without this the ttcStop and ttcMinGap sweeps would silently
# report as "configurations missing" and be dropped entirely.
DEFAULTS = {"ttcHorizon": 2.0, "ttcStop": 0.8, "ttcMinGap": 0.40}
CENTRE = "ttcHorizon=2.0"


def cfg_label(factor, lv):
    return CENTRE if lv == DEFAULTS[factor] else f"{factor}={lv}"


def spearman(x, y):
    """Rank correlation, no scipy dependency. Direction summary only."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    ok = ~(np.isnan(x) | np.isnan(y))
    x, y = x[ok], y[ok]
    if len(x) < 3:
        return float("nan")

    def rank(v):
        o = np.argsort(v, kind="mergesort")
        r = np.empty(len(v), float)
        r[o] = np.arange(len(v), dtype=float)
        _, inv, cnt = np.unique(v, return_inverse=True, return_counts=True)
        for i in np.where(cnt > 1)[0]:
            m = inv == i
            r[m] = r[m].mean()
        return r

    rx, ry = rank(x), rank(y)
    rx -= rx.mean()
    ry -= ry.mean()
    d = np.sqrt((rx**2).sum() * (ry**2).sum())
    return float((rx * ry).sum() / d) if d else float("nan")


def mean_ci(v, n=4000):
    """Bootstrap interval for the mean at a single level."""
    v = np.asarray(v, float)
    v = v[~np.isnan(v)]
    if len(v) < 2:
        return float("nan"), float("nan"), float("nan")
    idx = RNG.integers(0, len(v), size=(n, len(v)))
    m = v[idx].mean(axis=1)
    return float(v.mean()), float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))


def paired_ci(a, b, n=4000):
    """Paired percentile bootstrap for mean(b) - mean(a), same scenarios."""
    d = np.asarray(b, float) - np.asarray(a, float)
    ok = ~np.isnan(d)
    d = d[ok]
    if len(d) < 2:
        return float("nan"), float("nan"), float("nan")
    idx = RNG.integers(0, len(d), size=(n, len(d)))
    m = d[idx].mean(axis=1)
    return float(d.mean()), float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))


rows = json.load(open(SRC))
by = defaultdict(lambda: defaultdict(dict))
for r in rows:
    by[r["planner"]][r["config"]][r["scenario"]] = r
planners = sorted(by)
scens = sorted(by[planners[0]]["no-filter"])
print(f"loaded {len(rows)} runs | planners {planners} | scenarios {len(scens)}")


def vec(planner, config, key):
    """Per-scenario values in a fixed scenario order, so pairing is real."""
    d = by[planner][config]
    out = []
    for s in scens:
        r = d.get(s)
        if r is None:
            out.append(np.nan)
        elif key == "rate_goal":
            # Goal arrival is NOT a category: it is safe OR unsafe arrival.
            # Testing category == "goal" silently yields all zeros.
            out.append(1.0 if r["category"] in ("safe", "unsafe") else 0.0)
        elif key.startswith("rate_"):
            cat = key[5:]
            assert cat in ("safe", "unsafe", "deadlock", "other"), cat
            out.append(1.0 if r["category"] == cat else 0.0)
        elif key == "first_act_s":
            fa = r["first_activation_step"]
            out.append(fa * 0.01 if fa > 0 else np.nan)
        else:
            out.append(float(r[key]))
    return np.array(out, float)


def nanmean(v):
    v = v[~np.isnan(v)]
    return float(v.mean()) if len(v) else float("nan")


def arch_category(r):
    """Same rule as transition_table.py, so replay compares like with like."""
    if r["outcome"] == "success":
        return "safe" if r["contacts"] == 0 else "unsafe"
    return "deadlock" if r["outcome"] == "deadlock" else "other"


def validate():
    """Structural checks that must pass before any number is interpreted."""
    expected = {"no-filter"} | {cfg_label(f, lv) for f, L in LEVELS.items() for lv in L}
    for planner in planners:
        got = set(by[planner])
        assert got == expected, f"{planner}: config mismatch {got ^ expected}"
        for c in got - {"no-filter"}:
            factor, _, lv = c.partition("=")
            assert factor in LEVELS, f"unknown factor {factor}"
            assert float(lv) in LEVELS[factor], f"unknown level {c}"
        # Every factor's centre must resolve to the shared configuration.
        for f in LEVELS:
            assert cfg_label(f, DEFAULTS[f]) == CENTRE, f"{f} centre mislabelled"
            assert CENTRE in got, "shared centre configuration absent"
        for c in sorted(got):
            tot = sum(nanmean(vec(planner, c, f"rate_{k}"))
                      for k in ("safe", "unsafe", "deadlock", "other"))
            assert abs(tot - 1.0) < 1e-9, f"{planner}/{c}: rates sum to {tot}"
    print("  one-factor-at-a-time labelling: OK (3 scales, shared centre 2.0)")
    print("  category rates sum to 1 in every cell: OK")

    # Completeness and missingness, reported rather than assumed.
    n_exp = len(scens) * len(planners) * len(expected)
    print(f"  expected cells {n_exp}, observed {len(rows)}"
          + ("  OK" if len(rows) == n_exp else "  <-- MISSING RUNS"))
    miss = [(p, c, len(by[p][c])) for p in planners for c in sorted(by[p])
            if len(by[p][c]) != len(scens)]
    for p, c, n in miss:
        print(f"    incomplete: {p}/{c} has {n}/{len(scens)} scenarios")
    if not miss:
        print("  every configuration covers every scenario: OK")

    if not ARCH.exists():
        print("  archived confirmatory file absent — replay check SKIPPED")
        return
    a = defaultdict(dict)
    for r in json.load(open(ARCH)):
        if not r.get("failed"):
            a[r["arm"]][r["scenario"]] = r
    pairs = [("hybrid", "no-filter", "hybrid"), ("sfm", "no-filter", "sfm"),
             ("hybrid", "ttcHorizon=2.0", "hybrid-ttc"),
             ("sfm", "ttcHorizon=2.0", "sfm-ttc")]
    worst = 0.0
    for planner, cfg, arm in pairs:
        if planner not in by or cfg not in by[planner] or arm not in a:
            continue
        cats = [arch_category(r) for r in a[arm].values()]
        for cat in ("safe", "unsafe", "deadlock"):
            mine = nanmean(vec(planner, cfg, f"rate_{cat}"))
            ref = float(np.mean([c == cat for c in cats]))
            worst = max(worst, abs(mine - ref))
            flag = "" if abs(mine - ref) < 1e-9 else "   <-- DRIFT"
            print(f"  replay {planner}/{cfg:<15} vs {arm:<11} "
                  f"{cat:<9} {mine:.4f} vs {ref:.4f}{flag}")
    print(f"  worst replay drift: {worst:.2e}"
          + ("  OK" if worst < 1e-9 else "  <-- INVESTIGATE before interpreting"))


def report_level(planner, factor, levels, cfgs, title, spec, note):
    print(f"\n  {title}")
    if note:
        print(f"    {note}")
    for key, lab, sign in spec:
        vals, cis = [], []
        for c in cfgs:
            m, lo, hi = mean_ci(vec(planner, c, key))
            vals.append(m)
            cis.append((lo, hi))
        rho = spearman(levels, vals)
        print(f"    {lab}")
        for lv, m, (lo, hi) in zip(levels, vals, cis):
            print(f"      {factor}={lv:<6} {m:>9.3f}  [{lo:>7.3f},{hi:>7.3f}]")
        if sign == 0:
            i = int(np.nanargmax(vals))
            interior = 0 < i < len(vals) - 1
            # An interior peak is only meaningful if it is separated from the
            # endpoints by more than their intervals; otherwise it is a ranking
            # of noise.
            sep = (interior and
                   cis[i][0] > max(cis[0][1], cis[-1][1]))
            v = (f"peak at {levels[i]} "
                 f"({'INTERIOR' if interior else 'at boundary'}"
                 f"{', separated from endpoints' if sep else ', NOT separated from endpoints'})")
        else:
            pred = "increase" if sign > 0 else "decrease"
            v = (f"rho {rho:+.2f}, predicted {pred} — "
                 + ("consistent" if (np.isnan(rho) or np.sign(rho) == sign)
                    else "COUNTER"))
        print(f"      -> {v}")


print("\nSTRUCTURAL VALIDATION")
validate()

print("\n" + "=" * 92)
print("DOSE-RESPONSE SWEEP — mechanistic, EXPLORATORY (Amendments A.5, D, D.1)")
print("A paired follow-up on the confirmed scenario set, NOT an independent replication.")
print("=" * 92)

for planner in planners:
    print(f"\n{'#' * 92}\n## planner: {planner}\n{'#' * 92}")
    goal_ref = vec(planner, "no-filter", "rate_goal")
    print(f"\nfilter disabled: safe {nanmean(vec(planner, 'no-filter', 'rate_safe')):.3f}  "
          f"unsafe {nanmean(vec(planner, 'no-filter', 'rate_unsafe')):.3f}  "
          f"deadlock {nanmean(vec(planner, 'no-filter', 'rate_deadlock')):.3f}  "
          f"goal arrival {nanmean(goal_ref):.3f}")

    for factor, levels in LEVELS.items():
        cfgs = [cfg_label(factor, lv) for lv in levels]
        if any(c not in by[planner] for c in cfgs):
            print(f"\n  {factor}: configurations missing — skipped")
            continue
        print(f"\n{'-' * 92}\n  FACTOR {factor}   levels {levels}   "
              f"(larger = stronger intervention)\n{'-' * 92}")

        report_level(planner, factor, levels, cfgs,
                     "L1 CONTROLLER — mechanical consequence of the control law",
                     L1_CONTROLLER,
                     "Failure HERE would falsify the activation-to-slowing pathway.")
        report_level(planner, factor, levels, cfgs,
                     "L2 BEHAVIOUR — motion and outcome response", L2_BEHAVIOUR,
                     "Expected on average. Failure does NOT by itself refute L1.")
        report_level(planner, factor, levels, cfgs,
                     "L3 SYSTEM — the proposed trade-off", L3_SYSTEM,
                     "A turning point is the prediction; noise here does not refute L1/L2.")

        print("\n    Paired change vs filter disabled (same scenarios):")
        for key, lab in (("rate_safe", "safe success"), ("rate_deadlock", "deadlock")):
            base = vec(planner, "no-filter", key)
            for c, lv in zip(cfgs, levels):
                d, lo, hi = paired_ci(base, vec(planner, c, key))
                star = "" if lo <= 0 <= hi else "  *"
                print(f"      {lab:<14} {factor}={lv:<6} {d:>+7.3f} "
                      f"[{lo:+.3f},{hi:+.3f}]{star}")

print("\n" + "=" * 92)
print("L4 ARCHITECTURE — A.5 prediction 7, the sharpest test")
print("Onset (frozen in Amendment D): smallest swept level at which that arm's")
print("goal-arrival rate falls to <= 0.75x its own filter-disabled rate.")
print("Relative, not absolute, because the arms' baselines differ ~4x.")
print("=" * 92)

for factor, levels in LEVELS.items():
    print(f"\n  {factor}")
    onsets = {}
    for planner in planners:
        base = nanmean(vec(planner, "no-filter", "rate_goal"))
        hit = None
        for lv in levels:
            c = cfg_label(factor, lv)
            if c not in by[planner]:
                continue
            g, lo, hi = mean_ci(vec(planner, c, "rate_goal"))
            mark = ""
            if hit is None and g <= 0.75 * base:
                hit, mark = lv, "  <- onset"
            print(f"    {planner:<8} {factor}={lv:<6} goal {g:.3f} [{lo:.3f},{hi:.3f}]  "
                  f"({g / base if base else float('nan'):.2f}x baseline {base:.3f}){mark}")
        onsets[planner] = hit
    guided, unguided = onsets.get("hybrid"), onsets.get("sfm")
    if guided is None and unguided is None:
        v = ("neither arm reached onset within the swept range — a NULL. "
             "No ordering claim follows.")
    elif unguided is None:
        v = "guided reached onset, unguided did not — CONTRARY to prediction 7"
    elif guided is None:
        v = ("unguided reached onset within the swept range, guided did not — "
             "CONSISTENT with prediction 7")
    elif unguided < guided:
        v = f"unguided onset {unguided} < guided {guided} — CONSISTENT with prediction 7"
    elif unguided == guided:
        v = f"both at {guided} — no separation at the swept resolution"
    else:
        v = f"unguided onset {unguided} > guided {guided} — CONTRARY to prediction 7"
    print(f"    => {v}")

print("\nReporting reminders:")
print("  * L1 failure falsifies the pathway; L2/L3 failure does not.")
print("  * Show the level-by-level curve with intervals, not rho alone.")
print("  * 'Not reached within the swept range' is a null, never an ordering claim.")
print("  * This is a paired follow-up on the confirmed scenario set (D.1).")
