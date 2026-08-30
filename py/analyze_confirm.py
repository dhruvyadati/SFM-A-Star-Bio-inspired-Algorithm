"""Frozen confirmatory analysis. Implements CONFIRMATORY_PROTOCOL.md exactly.

Written before any confirmatory outcome was inspected. The primary test uses
the identical statistic the power simulation used.

Order is fixed: primary first, then the ordered secondary hierarchy, then the
prespecified mediator analyses. Nothing below the primary is a separate
confirmatory discovery.
"""
import json
import sys
from collections import defaultdict
from math import comb
from pathlib import Path

import numpy as np

RNG = np.random.default_rng(20260731)
CONDS = ["baseline", "dense", "cluttered", "bigObs"]
MEANINGFUL = 0.075          # smallest meaningful effect, frozen
ALPHA = 0.05


def mcnemar_exact(x, y):
    """Two-sided exact paired binomial. x, y are 0/1 arrays over scenarios."""
    n01 = int(np.sum((x == 0) & (y == 1)))
    n10 = int(np.sum((x == 1) & (y == 0)))
    n = n01 + n10
    if n == 0:
        return 1.0, n01, n10
    k = min(n01, n10)
    p = min(1.0, 2 * sum(comb(n, i) for i in range(k + 1)) / 2 ** n)
    return p, n01, n10


def strat_delta(pairs_by_cond, a, b):
    """Equal-condition benchmark average of the paired difference."""
    return float(np.mean([
        (pairs_by_cond[c][b] - pairs_by_cond[c][a]).mean() for c in CONDS
    ]))


def strat_ci(pairs_by_cond, a, b, n_boot=10000):
    """Percentile bootstrap over scenarios WITHIN strata, equal stratum weights."""
    out = np.empty(n_boot)
    for i in range(n_boot):
        acc = []
        for c in CONDS:
            xa, xb = pairs_by_cond[c][a], pairs_by_cond[c][b]
            idx = RNG.integers(0, len(xa), len(xa))
            acc.append((xb[idx] - xa[idx]).mean())
        out[i] = np.mean(acc)
    return float(np.percentile(out, 2.5)), float(np.percentile(out, 97.5))


def load(path="v2/runs/v2/confirm_factorial.json"):
    res = json.load(open(path))
    failed = [r for r in res if r.get("failed")]
    by = defaultdict(dict)
    cond = {}
    for r in res:
        if r.get("failed"):
            continue
        by[r["scenario"]][r["arm"]] = r
        cond[r["scenario"]] = r["condition"]
    # A scenario failing in ANY arm is dropped from all four.
    complete = [s for s in sorted(by) if len(by[s]) == 4]
    dropped = len(by) - len(complete)
    return by, cond, complete, failed, dropped


def field_by_cond(by, cond, complete, field):
    out = {}
    for c in CONDS:
        ss = [s for s in complete if cond[s] == c]
        out[c] = {a: np.array([by[s][a][field] for s in ss], float)
                  for a in ("sfm", "sfm-ttc", "hybrid", "hybrid-ttc")}
    return out


if __name__ == "__main__":
    src = sys.argv[1] if len(sys.argv) > 1 else "v2/runs/v2/confirm_factorial.json"
    by, cond, complete, failed, dropped = load(src)

    print("=" * 72)
    print("CONFIRMATORY ANALYSIS — frozen protocol, executed once")
    print("=" * 72)
    print(f"scenarios with all four arms : {len(complete)}")
    print(f"runs failed                  : {len(failed)}")
    print(f"scenarios dropped (any arm)  : {dropped}")
    for c in CONDS:
        print(f"  {c:<10} {sum(1 for s in complete if cond[s] == c)}")

    safe = field_by_cond(by, cond, complete, "safe")

    # ---------------- PRIMARY -----------------------------------------
    print("\n" + "=" * 72)
    print("PRIMARY: does TTC improve safe-goal success under A* guidance?")
    print("=" * 72)
    d = strat_delta(safe, "hybrid", "hybrid-ttc")
    lo, hi = strat_ci(safe, "hybrid", "hybrid-ttc")
    xs = np.concatenate([safe[c]["hybrid"] for c in CONDS])
    ys = np.concatenate([safe[c]["hybrid-ttc"] for c in CONDS])
    p, n01, n10 = mcnemar_exact(xs, ys)
    print(f"  equal-condition Delta : {d:+.4f}")
    print(f"  95% CI (bootstrap)    : [{lo:+.4f}, {hi:+.4f}]")
    print(f"  exact McNemar         : p = {p:.3e}   discordant {n01}/{n10}")
    print(f"  smallest meaningful   : {MEANINGFUL:+.3f}")

    print("\n  DECISION (rules fixed in advance):")
    if lo > MEANINGFUL:
        print("  -> Interval entirely above the meaningful threshold.")
        print("     Strong evidence of a practically meaningful improvement.")
    elif lo > 0 and hi > MEANINGFUL:
        print("  -> Positive, interval spans the meaningful threshold.")
        print("     Evidence of benefit; INSUFFICIENT evidence it reaches the")
        print("     magnitude prespecified as worth having.")
    elif lo > 0:
        print("  -> Positive and excludes zero, but entirely BELOW the")
        print("     meaningful threshold. Benefit replicates but is smaller")
        print("     than the level set as worth having.")
    elif hi < 0:
        # Distinct from "did not replicate": an interval entirely below zero
        # says the filter REDUCES safe-goal success, which is a finding, not
        # an absence of one.
        print("  -> Interval entirely BELOW zero: the filter reduced safe-goal")
        print("     success. This is a harmful effect, not a failure to replicate.")
    else:
        print("  -> Interval includes zero: the development benefit did NOT")
        print("     independently replicate.")

    # ---------------- SECONDARY, ordered -------------------------------
    print("\n" + "=" * 72)
    print("SECONDARY (ordered, prespecified — not separate discoveries)")
    print("=" * 72)
    order = [
        ("unsafe_goal", "hybrid", "hybrid-ttc", "1. unsafe arrival, hyb->hyb-ttc"),
        ("deadlock",    "hybrid", "hybrid-ttc", "2. deadlock, hyb->hyb-ttc"),
        ("goal",        "hybrid", "hybrid-ttc", "3. goal arrival, hyb->hyb-ttc"),
        ("safe", "sfm-ttc", "hybrid-ttc",       "4. guidance effect, with TTC"),
        ("safe", "sfm",     "hybrid",           "5. guidance effect, no TTC"),
        ("safe", "sfm",     "sfm-ttc",          "6. TTC effect, unguided"),
    ]
    for field, a, b, label in order:
        f = field_by_cond(by, cond, complete, field)
        dd = strat_delta(f, a, b)
        l, h = strat_ci(f, a, b, n_boot=4000)
        pp, k01, k10 = mcnemar_exact(
            np.concatenate([f[c][a] for c in CONDS]),
            np.concatenate([f[c][b] for c in CONDS]))
        print(f"  {label:<34} {dd:+.4f} [{l:+.4f},{h:+.4f}]  "
              f"disc {k01}/{k10}  p={pp:.2e}")

    # 7. interaction — secondary factorial, NOT powered as primary
    print()
    inter = []
    for c in CONDS:
        s = safe[c]
        inter.append(((s["hybrid-ttc"] - s["hybrid"]) -
                      (s["sfm-ttc"] - s["sfm"])).mean())
    di = float(np.mean(inter))
    boot = np.empty(4000)
    for i in range(4000):
        acc = []
        for c in CONDS:
            s = safe[c]
            idx = RNG.integers(0, len(s["hybrid"]), len(s["hybrid"]))
            acc.append(((s["hybrid-ttc"][idx] - s["hybrid"][idx]) -
                        (s["sfm-ttc"][idx] - s["sfm"][idx])).mean())
        boot[i] = np.mean(acc)
    print(f"  7. guidance x TTC interaction      {di:+.4f} "
          f"[{np.percentile(boot,2.5):+.4f},{np.percentile(boot,97.5):+.4f}]")
    print("     (secondary factorial; study not primarily powered for this)")

    # 8. condition-specific — descriptive
    print("\n  8. condition-specific primary effect (descriptive, not powered)")
    for c in CONDS:
        s = safe[c]
        dc = (s["hybrid-ttc"] - s["hybrid"]).mean()
        print(f"     {c:<12} {dc:+.4f}   n={len(s['hybrid'])}")

    # ---------------- outcome table ------------------------------------
    print("\n  Mutually exclusive outcomes by arm (%)")
    print(f"  {'arm':<12} {'safe':>8} {'unsafe':>8} {'deadlock':>9} {'timeout':>8}")
    for a in ("sfm", "sfm-ttc", "hybrid", "hybrid-ttc"):
        rs = [by[s][a] for s in complete]
        n = len(rs)
        print(f"  {a:<12} {100*sum(r['safe'] for r in rs)/n:>8.1f} "
              f"{100*sum(r['unsafe_goal'] for r in rs)/n:>8.1f} "
              f"{100*sum(r['deadlock'] for r in rs)/n:>9.1f} "
              f"{100*sum(r['timeout'] for r in rs)/n:>8.1f}")

    # ---------------- MEDIATORS (Amendment A.2) ------------------------
    print("\n" + "=" * 72)
    print("MEDIATOR ANALYSES (Amendment A.2 — exploratory, post-treatment)")
    print("=" * 72)
    print("  Activation and motion response, paired")
    print(f"  {'comparison':<24} {'ttcActive':>10} {'stopTime':>10} "
          f"{'meanSpeed':>10} {'stopEps':>9}")
    for a, b, lab in (("hybrid", "hybrid-ttc", "hybrid -> hybrid-ttc"),
                      ("sfm", "sfm-ttc", "sfm -> sfm-ttc")):
        ra = [by[s][a] for s in complete]
        rb = [by[s][b] for s in complete]
        print(f"  {lab:<24} "
              f"{np.mean([r['ttc_active_frac'] for r in rb]):>10.3f} "
              f"{np.mean([r['stop_time'] for r in rb]) - np.mean([r['stop_time'] for r in ra]):>+10.2f} "
              f"{np.mean([r['mean_speed'] for r in rb]) - np.mean([r['mean_speed'] for r in ra]):>+10.3f} "
              f"{np.mean([r['stop_episodes'] for r in rb]) - np.mean([r['stop_episodes'] for r in ra]):>+9.2f}")

    print("\n  Temporal ordering: does deadlock follow sustained activation?")
    for a in ("hybrid-ttc", "sfm-ttc"):
        rs = [by[s][a] for s in complete
              if by[s][a]["deadlock"] and by[s][a]["deadlock_onset_step"] > 0]
        if not rs:
            print(f"    {a:<12} no deadlocks with recorded onset")
            continue
        after = sum(1 for r in rs
                    if 0 < r["sustained_activation_step"] < r["deadlock_onset_step"])
        never = sum(1 for r in rs if r["sustained_activation_step"] < 0)
        print(f"    {a:<12} {len(rs)} deadlocks: {after} preceded by sustained "
              f"activation, {never} with no sustained activation at all")
    print("\n  Deadlock preceding activation would weaken the proposed pathway.")
    print("  These are post-treatment variables; no mediated proportion is claimed.")
