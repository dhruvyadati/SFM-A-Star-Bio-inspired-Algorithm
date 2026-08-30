"""Stability of the interaction estimate as the development sample grows.

Checkpoints at 18 / 30 / 50 / 75 scenarios per condition are NESTED subsets of
one growing dataset, not independent samples. Their purpose is to show whether
the pattern settles, not to run repeated significance tests -- reading a
p-value at each checkpoint and stopping when it looks good would be exactly
the optional-stopping error the freeze exists to prevent.

Confidence intervals accompany every point estimate, because at n=18 a single
scenario moves a rate by 5.6 percentage points and raw estimates can make
sampling noise look like structure.

Three outcomes are all scientifically useful and none is the target:
  * heterogeneity persists  -> environmental dependence is the headline
  * estimates converge      -> the spread was small-sample noise
  * estimates wander        -> the interaction is too unstable for a general
                               claim, and the paper narrows to descriptive
                               system evaluation
"""
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

RNG = np.random.default_rng(31415)
CHECKPOINTS = [18, 30, 50, 75]
CONDS = ["baseline", "dense", "cluttered", "bigObs"]
ARM_ORDER = ["sfm", "sfm-ttc", "hybrid", "hybrid-ttc"]


def contrast(p):
    """Per-scenario interaction contrast: (G,T)-(G,0) - [(0,T)-(0,0)]."""
    return (p[:, 3] - p[:, 2]) - (p[:, 1] - p[:, 0])


def boot_ci(d, n_boot=4000, alpha=0.05):
    if len(d) < 3:
        return (float("nan"), float("nan"))
    idx = RNG.integers(0, len(d), size=(n_boot, len(d)))
    means = d[idx].mean(axis=1)
    return (float(np.percentile(means, 100 * alpha / 2)),
            float(np.percentile(means, 100 * (1 - alpha / 2))))


def load(path="v2/runs/v2/dev_factorial_full.json"):
    res = json.load(open(path))
    by_scen = defaultdict(dict)
    cond = {}
    for r in res:
        by_scen[r["scenario"]][r["arm"]] = r
        cond[r["scenario"]] = r["condition"]
    # Sorted scenario name gives a stable, reproducible ordering, so the
    # "first k" of a condition is well defined and identical across runs.
    per_cond = defaultdict(list)
    for s in sorted(by_scen):
        d = by_scen[s]
        if len(d) == 4:
            per_cond[cond[s]].append([d[a]["safe"] for a in ARM_ORDER])
    return {c: np.array(v) for c, v in per_cond.items()}, res


if __name__ == "__main__":
    src = sys.argv[1] if len(sys.argv) > 1 else "v2/runs/v2/dev_factorial_full.json"
    per_cond, res = load(src)

    print("=== Interaction stability across development sample size ===\n")
    avail = {c: len(v) for c, v in per_cond.items()}
    print("scenarios available per condition:", avail, "\n")

    print(f"{'per cond':>9} " + "".join(f"{c:>22}" for c in CONDS)
          + f"{'equal-weighted':>22}")
    print("-" * 122)
    for k in CHECKPOINTS:
        if any(avail.get(c, 0) < k for c in CONDS):
            continue
        row = f"{k:>9} "
        deltas = []
        for c in CONDS:
            d = contrast(per_cond[c][:k])
            lo, hi = boot_ci(d)
            deltas.append(d.mean())
            row += f"{d.mean():>+8.3f} [{lo:+.2f},{hi:+.2f}]"
        # Equal-condition benchmark estimand: the mean of the four condition
        # effects, NOT a sample-size-weighted pool. It answers "what is the
        # average interaction if each condition family counts equally", which
        # is a constructed benchmark question, not a deployment-wide effect.
        eq = float(np.mean(deltas))
        allc = np.concatenate([contrast(per_cond[c][:k]) for c in CONDS])
        lo, hi = boot_ci(allc)
        row += f"{eq:>+8.3f} [{lo:+.2f},{hi:+.2f}]"
        print(row)

    kmax = min(avail.get(c, 0) for c in CONDS)
    print(f"\n=== Detail at {kmax} per condition ===\n")
    print(f"{'condition':<11} {'delta':>8} {'95% CI':>18} {'informative':>12} "
          f"{'all-fail':>9}  safe% by arm")
    print("-" * 104)
    deltas, widths = [], []
    for c in CONDS:
        p = per_cond[c][:kmax]
        d = contrast(p)
        lo, hi = boot_ci(d)
        deltas.append(d.mean())
        widths.append(hi - lo)
        rates = " ".join(f"{100*p[:, i].mean():5.1f}" for i in range(4))
        print(f"{c:<11} {d.mean():>+8.3f} [{lo:>+6.3f},{hi:>+6.3f}] "
              f"{np.count_nonzero(d)/len(d):>12.2f} "
              f"{(p.sum(axis=1) == 0).mean():>9.2f}  {rates}")

    spread = max(deltas) - min(deltas)
    mean_w = float(np.mean(widths))
    print(f"\nspread across conditions : {spread:+.3f}")
    print(f"mean CI width            : {mean_w:.3f}")
    print("\nHeterogeneity is only interpretable if the spread exceeds what the")
    print("intervals alone would produce. If spread is comparable to the mean CI")
    print("width, the condition differences are not established.")
    if spread > mean_w:
        print(f"=> spread ({spread:.3f}) EXCEEDS mean CI width ({mean_w:.3f}):"
              " heterogeneity is plausible")
    else:
        print(f"=> spread ({spread:.3f}) is within mean CI width ({mean_w:.3f}):"
              " condition differences NOT established")
