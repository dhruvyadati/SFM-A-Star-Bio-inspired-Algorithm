"""Outcome-transition decomposition, paired within identical scenarios.

Refines Amendment A.2 item 4. A.2 planned the AGGREGATE decomposition

    d(safe) = -d(unsafe) - d(deadlock) - d(timeout)

which shows the net movement between categories but not which scenarios moved
where. Two filters with identical aggregate deltas can differ completely: one
rescuing 40 unsafe arrivals while breaking 30 safe ones looks the same as one
rescuing 10 and breaking none.

The full 4x4 matrix is reported, not only the interesting cells, so the
diagonal (no outcome-level change) is visible alongside the transitions.

STATUS: recorded in Amendment C before any confirmatory outcome was accessed.
It is an extension of a planned analysis, not part of the original frozen
protocol, and is described that way.

Run AFTER the frozen primary and secondary analyses.
"""
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

CATS = ["safe", "unsafe", "deadlock", "other"]
CONDS = ["baseline", "dense", "cluttered", "bigObs"]
RNG = np.random.default_rng(11235)


def category(r):
    """Mutually exclusive outcome category, matching the frozen endpoint."""
    if r["outcome"] == "success":
        return "safe" if r["contacts"] == 0 else "unsafe"
    if r["outcome"] == "deadlock":
        return "deadlock"
    return "other"          # timeout or any non-terminating outcome


def matrix(by, complete, a, b):
    m = {u: {v: 0 for v in CATS} for u in CATS}
    for s in complete:
        m[category(by[s][a])][category(by[s][b])] += 1
    return m


def show(m, a, b, n):
    print(f"\n  rows = {a} (without), columns = {b} (with), n = {n}")
    print(f"  {'':<10}" + "".join(f"{c:>10}" for c in CATS) + f"{'row tot':>10}")
    print("  " + "-" * 60)
    for u in CATS:
        tot = sum(m[u].values())
        print(f"  {u:<10}" + "".join(f"{m[u][v]:>10}" for v in CATS)
              + f"{tot:>10}")
    print(f"  {'col tot':<10}"
          + "".join(f"{sum(m[u][v] for u in CATS):>10}" for v in CATS))
    unchanged = sum(m[c][c] for c in CATS)
    print(f"\n  unchanged outcome: {unchanged}/{n} ({100*unchanged/n:.1f}%)")


def highlight(m, n):
    rows = [
        ("unsafe", "safe", "genuine rescue: contact averted, goal still reached"),
        ("unsafe", "deadlock", "safety bought by abandoning progress"),
        ("safe", "deadlock", "harmful overconstraint: a good run broken"),
        ("deadlock", "safe", "improved recovery"),
        ("safe", "unsafe", "safety LOST"),
    ]
    print(f"\n  {'transition':<22} {'count':>7} {'% of n':>8}  interpretation")
    print("  " + "-" * 78)
    for u, v, txt in rows:
        c = m[u][v]
        print(f"  {u + ' -> ' + v:<22} {c:>7} {100*c/n:>7.1f}%  {txt}")

    net = m["unsafe"]["safe"] - m["safe"]["unsafe"]
    cost = m["safe"]["deadlock"] + m["unsafe"]["deadlock"]
    gain = m["deadlock"]["safe"] + m["unsafe"]["safe"]
    print(f"\n  net safety rescues (unsafe->safe minus safe->unsafe) : {net:+d}")
    print(f"  runs converted TO deadlock                           : {cost}")
    print(f"  runs converted FROM deadlock or unsafe TO safe       : {gain}")
    print("\n  A filter that rescues many unsafe arrivals while converting as")
    print("  many good runs into deadlock is a trade, not an improvement. The")
    print("  aggregate delta alone cannot distinguish those cases.")


if __name__ == "__main__":
    src = sys.argv[1] if len(sys.argv) > 1 else "v2/runs/v2/confirm_factorial.json"
    if not Path(src).exists():
        sys.exit(f"{src} not found — run confirmation first")

    res = json.load(open(src))
    by, cond = defaultdict(dict), {}
    for r in res:
        if r.get("failed"):
            continue
        by[r["scenario"]][r["arm"]] = r
        cond[r["scenario"]] = r["condition"]
    complete = [s for s in sorted(by) if len(by[s]) == 4]

    print("=" * 72)
    print("OUTCOME-TRANSITION DECOMPOSITION (extends Amendment A.2 item 4)")
    print("=" * 72)

    for a, b in (("hybrid", "hybrid-ttc"), ("sfm", "sfm-ttc")):
        print("\n" + "=" * 72)
        print(f"{a}  ->  {b}")
        print("=" * 72)
        m = matrix(by, complete, a, b)
        show(m, a, b, len(complete))
        highlight(m, len(complete))

    print("\n" + "=" * 72)
    print("Per condition, hybrid -> hybrid-ttc (descriptive)")
    print("=" * 72)
    print(f"  {'condition':<12} {'n':>5} {'uns->safe':>10} {'uns->dead':>10} "
          f"{'safe->dead':>11} {'dead->safe':>11}")
    for c in CONDS:
        ss = [s for s in complete if cond[s] == c]
        if not ss:
            continue
        m = matrix(by, ss, "hybrid", "hybrid-ttc")
        print(f"  {c:<12} {len(ss):>5} {m['unsafe']['safe']:>10} "
              f"{m['unsafe']['deadlock']:>10} {m['safe']['deadlock']:>11} "
              f"{m['deadlock']['safe']:>11}")
