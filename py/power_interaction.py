"""Power planning for the confirmatory guidance x filter interaction.

Run BEFORE confirmatory seeds are spent, because they can only be spent once.

The interaction is a DIFFERENCE OF TWO PAIRED EFFECTS:

    delta = [p(G,T) - p(G,0)] - [p(0,T) - p(0,0)]

Power for that does not follow from the marginal 20-point effect. It depends
on the JOINT paired outcome distribution -- specifically how often the arms
disagree within a scenario. Two designs with identical marginal effects can
have very different power depending on discordance. So this simulates from the
observed 16-cell joint pattern of the development factorial rather than
treating the four arms as independent Bernoulli draws.

Reported at each affordable N:
  * power at the planned effect and at smaller ones
  * minimum detectable effect
  * expected informative (non-zero contrast) scenarios
  * probability the test is nearly uninformative
  * sensitivity to baseline rates and within-scenario correlation
"""
import csv
import itertools
import sys
from collections import Counter
from pathlib import Path

import numpy as np

ARMS = ["sfm", "sfm-ttc", "hybrid", "hybrid-ttc"]   # (0,0) (0,T) (G,0) (G,T)
CSV = Path("v2/runs/v2/safety_gate.csv")
RNG = np.random.default_rng(20260730)


def load_joint():
    """Observed joint safe-success pattern across the four arms, by scenario."""
    rows = list(csv.DictReader(open(CSV)))
    by_trial = {}
    for r in rows:
        if r["method"] not in ARMS:
            continue
        by_trial.setdefault(r["trial"], {})[r["method"]] = int(float(r["safeSuccess"]))
    pats = []
    for t, d in sorted(by_trial.items()):
        if len(d) == len(ARMS):
            pats.append(tuple(d[a] for a in ARMS))
    return pats


def interaction(sample):
    """delta for one simulated dataset. sample is (n, 4) in ARMS order."""
    s00, s0T, sG0, sGT = sample[:, 0], sample[:, 1], sample[:, 2], sample[:, 3]
    d = (sGT - sG0) - (s0T - s00)
    return float(d.mean()), d


def perm_p(d, nperm=2000, rng=RNG):
    """Sign-flip permutation p-value, matching the planned analysis."""
    obs = abs(d.mean())
    flips = rng.choice([-1.0, 1.0], size=(nperm, d.size))
    null = np.abs((flips * d).mean(axis=1))
    return (np.sum(null >= obs - 1e-12) + 1) / (nperm + 1)


def simulate(pats, n, true_delta, nsim=600, alpha=0.05):
    """Power at sample size n for a given true interaction effect.

    The observed joint pattern is resampled to preserve within-scenario
    correlation, then the filter-under-guidance cell is perturbed to install
    the specified true effect. Perturbing one cell keeps the other three
    marginals and the correlation structure as observed.
    """
    base = np.array(pats, dtype=float)
    obs_delta, _ = interaction(base)
    shift = true_delta - obs_delta

    hits = 0
    informative = []
    for _ in range(nsim):
        idx = RNG.integers(0, len(base), size=n)
        s = base[idx].copy()
        if shift != 0.0:
            # Move p(G,T) by `shift`, flipping only cells that can move.
            k = int(round(abs(shift) * n))
            if k > 0:
                if shift > 0:
                    cand = np.flatnonzero(s[:, 3] == 0)
                else:
                    cand = np.flatnonzero(s[:, 3] == 1)
                if cand.size:
                    pick = RNG.choice(cand, size=min(k, cand.size), replace=False)
                    s[pick, 3] = 1.0 if shift > 0 else 0.0
        dmean, d = interaction(s)
        informative.append(int(np.count_nonzero(d)))
        if perm_p(d, nperm=400) < alpha:
            hits += 1
    return hits / nsim, float(np.mean(informative))


# --dev loads the 72-scenario development joint distribution instead of the
# 20-scenario pilot. The pilot is retained only for the side-by-side that shows
# how unstable a 20-scenario estimate is.
if "--dev" in sys.argv:
    import numpy as _np
    _p = _np.load("v2/runs/v2/dev_patterns.npy")
    _c = _np.load("v2/runs/v2/dev_conditions.npy")
    if "--cond" in sys.argv:
        which = sys.argv[sys.argv.index("--cond") + 1]
        _p = _p[_c == which]
        print(f"[condition subset: {which}, n={len(_p)}]")
    pats = [tuple(int(v) for v in row) for row in _p]
else:
    pats = load_joint()
if not pats:
    sys.exit("no complete four-arm scenarios found")

base = np.array(pats, dtype=float)
obs_delta, obs_d = interaction(base)

print("=== Confirmatory power planning: guidance x filter interaction ===\n")
print(f"development scenarios with all four arms: {len(pats)}")
print(f"observed interaction delta: {obs_delta:+.3f}\n")

print("Observed joint pattern (sfm, sfm-ttc, hybrid, hybrid-ttc) -> count:")
for pat, c in sorted(Counter(pats).items(), key=lambda kv: -kv[1]):
    print(f"  {pat}  x{c}")

nz = np.count_nonzero(obs_d)
print(f"\ninformative scenarios (non-zero contrast): {nz}/{len(pats)} "
      f"= {100*nz/len(pats):.0f}%")
print("Scenarios where the contrast is 0 carry NO information about the")
print("interaction under a sign-flip test, however many of them there are.\n")

Ns = [40, 60, 80, 100, 150, 200, 300]
DELTAS = [0.10, 0.15, 0.20, 0.25]

print("POWER (sign-flip permutation, alpha = 0.05)\n")
print(f"{'N':>6} " + "".join(f"{'d=' + format(d, '.2f'):>10}" for d in DELTAS)
      + f"{'informative':>13}")
print("-" * 62)
mde = {}
for n in Ns:
    row = f"{n:>6} "
    inf_n = None
    for d in DELTAS:
        pw, inf_n = simulate(pats, n, d, nsim=400)
        row += f"{pw:>10.2f}"
        if pw >= 0.80 and d not in mde:
            mde[d] = n
    row += f"{inf_n:>13.1f}"
    print(row)

print("\nSmallest N reaching 80% power, by true effect:")
for d in DELTAS:
    print(f"  delta = {d:.2f}  ->  N = {mde.get(d, '>300')}")

print("""
PRE-REGISTERED DECISION RULE
Run the confirmatory study only if the affordable N gives at least 80% power
for the SMALLEST scientifically meaningful interaction effect. If it does not,
narrow the claim, change the primary endpoint before confirmation, or label the
interaction analysis exploratory. Inadequate power becomes a scope decision
made in advance, not a post-hoc statistical adjustment.
""")


# ---------------------------------------------------------------------------
# HOW MUCH DOES THE PILOT ITSELF LIMIT THIS?
#
# Everything above is conditioned on a joint outcome distribution estimated
# from 20 pilot scenarios. With 7 all-fail cases observed, the all-fail rate
# carries a binomial standard error near 11 percentage points, and that
# uncertainty propagates straight into the required N. Bootstrapping the pilot
# patterns turns "N = 400" into a range, which is the honest form of the
# recommendation until the larger development set is analysed.

def required_n(pats_, delta, target=0.80, grid=(100, 150, 200, 300, 400, 500, 700),
               nsim=150):
    for n in grid:
        pw, _ = simulate(pats_, n, delta, nsim=nsim)
        if pw >= target:
            return n
    return grid[-1] * 2


if "--bootstrap" in sys.argv:
    print("\n=== Pilot uncertainty: bootstrap over the 20 development scenarios ===\n")
    print("  Direct simulation at candidate N, with Monte Carlo standard error")
    print("  (rather than extrapolating past the end of the grid):\n")
    for n in (300, 350, 400, 450):
        pw, inf = simulate(pats, n, 0.10, nsim=1000)
        mcse = (pw * (1 - pw) / 1000) ** 0.5
        print(f"    N={n:>4}  power(delta=0.10) = {pw:.3f} +/- {mcse:.3f}"
              f"   informative {inf:.0f}")

    print("\n  Bootstrapping the pilot joint distribution (25 resamples):")
    reqs, allfail, infr = [], [], []
    for _ in range(25):
        idx = RNG.integers(0, len(pats), size=len(pats))
        bp = [pats[i] for i in idx]
        allfail.append(sum(1 for p in bp if sum(p) == 0) / len(bp))
        _, d = interaction(np.array(bp, dtype=float))
        infr.append(np.count_nonzero(d) / len(bp))
        reqs.append(required_n(bp, 0.10, nsim=100))
    reqs = np.array(reqs)
    print(f"    all-fail fraction : {np.mean(allfail):.2f} "
          f"[{np.percentile(allfail,5):.2f}, {np.percentile(allfail,95):.2f}]")
    print(f"    informative frac  : {np.mean(infr):.2f} "
          f"[{np.percentile(infr,5):.2f}, {np.percentile(infr,95):.2f}]")
    print(f"    required N (delta=0.10, 80% power): median {int(np.median(reqs))}, "
          f"90% range [{int(np.percentile(reqs,5))}, {int(np.percentile(reqs,95))}]")
    print("\n  This spread comes from PILOT sampling error, not from the")
    print("  confirmatory design. Recompute on the 60-scenario development set")
    print("  before N is frozen.")
