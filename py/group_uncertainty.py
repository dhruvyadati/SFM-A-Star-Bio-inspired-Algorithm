"""Uncertainty on the transition-group comparisons.

POST-OUTCOME EXPLORATORY.

The earlier table reported group MEANS ("+3.26 s vs +0.63 s", "5x more stop
time"). Stop-time distributions are plausibly skewed and the groups differ
greatly in size (131 vs 37), so those ratios need medians, IQRs and intervals
before they can be quoted.
"""
import json
from collections import defaultdict
import numpy as np

RNG = np.random.default_rng(1729)


def category(r):
    if r["outcome"] == "success":
        return "safe" if r["contacts"] == 0 else "unsafe"
    return "deadlock" if r["outcome"] == "deadlock" else "other"


def ci(d, n=8000):
    idx = RNG.integers(0, len(d), size=(n, len(d)))
    m = d[idx].mean(axis=1)
    return np.percentile(m, 2.5), np.percentile(m, 97.5)


res = json.load(open("v2/runs/v2/confirm_factorial.json"))
by = defaultdict(dict)
for r in res:
    by[r["scenario"]][r["arm"]] = r
complete = [s for s in sorted(by) if len(by[s]) == 4]

G = {
  "safe -> safe":       lambda s: category(by[s]["hybrid"]) == "safe" and category(by[s]["hybrid-ttc"]) == "safe",
  "safe -> unsafe":     lambda s: category(by[s]["hybrid"]) == "safe" and category(by[s]["hybrid-ttc"]) == "unsafe",
  "unsafe -> safe":     lambda s: category(by[s]["hybrid"]) == "unsafe" and category(by[s]["hybrid-ttc"]) == "safe",
  "unsafe -> deadlock": lambda s: category(by[s]["hybrid"]) == "unsafe" and category(by[s]["hybrid-ttc"]) == "deadlock",
}

print("PAIRED CHANGE IN STOP TIME (hybrid-ttc minus hybrid), by group")
print("Distributions are skewed, so medians and IQRs are reported alongside means.\n")
print(f"{'group':<20} {'n':>4} {'mean':>8} {'95% CI':>18} {'median':>8} "
      f"{'IQR':>16} {'max':>7}")
print("-" * 88)
store = {}
for g, f in G.items():
    ss = [s for s in complete if f(s)]
    d = np.array([by[s]["hybrid-ttc"]["stop_time"] - by[s]["hybrid"]["stop_time"]
                  for s in ss])
    store[g] = d
    lo, hi = ci(d)
    q1, q3 = np.percentile(d, [25, 75])
    print(f"{g:<20} {len(d):>4} {d.mean():>+8.2f} [{lo:>+6.2f},{hi:>+6.2f}] "
          f"{np.median(d):>+8.2f} [{q1:>+6.2f},{q3:>+6.2f}] {d.max():>+7.1f}")

print("\nDIFFERENCE vs safe->safe, with bootstrap interval on the difference")
print(f"{'group':<20} {'mean diff':>11} {'95% CI':>20} {'median diff':>12}")
print("-" * 68)
base = store["safe -> safe"]
for g in ("safe -> unsafe", "unsafe -> safe", "unsafe -> deadlock"):
    d = store[g]
    boot = np.array([d[RNG.integers(0, len(d), len(d))].mean()
                     - base[RNG.integers(0, len(base), len(base))].mean()
                     for _ in range(8000)])
    print(f"{g:<20} {d.mean()-base.mean():>+11.2f} "
          f"[{np.percentile(boot,2.5):>+8.2f},{np.percentile(boot,97.5):>+8.2f}] "
          f"{np.median(d)-np.median(base):>+12.2f}")

print("\nACTIVATION FRACTION by group")
print(f"{'group':<20} {'n':>4} {'mean':>8} {'95% CI':>18} {'median':>8}")
print("-" * 62)
for g, f in G.items():
    ss = [s for s in complete if f(s)]
    a = np.array([by[s]["hybrid-ttc"]["ttc_active_frac"] for s in ss])
    lo, hi = ci(a)
    print(f"{g:<20} {len(a):>4} {a.mean():>8.3f} [{lo:>6.3f},{hi:>6.3f}] "
          f"{np.median(a):>8.3f}")

print("\nThe mean ratio quoted earlier (~5x) is sensitive to skew and to the")
print("unequal group sizes. Report the interval on the DIFFERENCE, not the ratio.")
