"""Analysis of the ORCA-responsive pedestrian study (Amendments E, E.1).

WRITTEN BEFORE THE STUDY PRODUCED OUTPUT.

Attribution rule, from E.1. Swapping SFM pedestrians for ORCA pedestrians does
not vary one quantity called "yielding": it changes interaction range,
anticipation, speed selection, collision geometry, crowd organisation,
congestion propagation and trajectory smoothness at once. Every difference here
is therefore attributable to REPLACING THE PEDESTRIAN MODEL, not to yielding.
Whether increased avoidance is the plausible explanation is assessed from
activation, stopping, duration and transitions -- as evidence, never assumed.

Transitions, not averages, carry the mechanism result. Two pedestrian models
can show the same net safe-success change while one rescues many unsafe runs
and breaks many safe ones and the other does neither.

Bootstrap generators are derived PER METRIC from a stable key, so an interval
never depends on how many draws an earlier computation consumed -- the
execution-order defect recorded in D.3.
"""
from __future__ import annotations

import hashlib
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

SRC = Path("v2/runs/v2/reciprocal.json")
ARCH = Path("v2/runs/v2/confirm_factorial.json")
CATS = ["safe", "unsafe", "deadlock", "other"]
THRESHOLD = 0.075


def rng_for(key: str) -> np.random.Generator:
    """Stable per-metric generator: reproducible regardless of call order."""
    h = hashlib.sha256(key.encode()).digest()[:8]
    return np.random.default_rng(int.from_bytes(h, "little"))


def paired_ci(a, b, key, n=8000):
    d = np.asarray(b, float) - np.asarray(a, float)
    g = rng_for(key)
    idx = g.integers(0, len(d), size=(n, len(d)))
    m = d[idx].mean(axis=1)
    return float(d.mean()), float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))


def branch(d, lo, hi):
    """The five frozen decision branches, zero tested before threshold."""
    if lo <= 0 <= hi:
        return "includes zero — no effect established"
    if hi < 0:
        return "entirely below zero — REDUCTION in safe-goal success"
    if lo > THRESHOLD:
        return f"entirely above {THRESHOLD} — benefit exceeds the meaningful magnitude"
    if lo > 0 and hi <= THRESHOLD:
        return f"positive but entirely below {THRESHOLD} — benefit smaller than meaningful"
    return f"positive, interval spans {THRESHOLD} — insufficient evidence it reaches it"


def category(r):
    if r["outcome"] == "success":
        return "safe" if r["contacts"] == 0 else "unsafe"
    return "deadlock" if r["outcome"] == "deadlock" else "other"


rows = json.load(open(SRC))
by = defaultdict(lambda: defaultdict(dict))     # [set][resp][arm][scen]
for r in rows:
    by[r["set"]][r["responsibility"]].setdefault(r["arm"], {})[r["scenario"]] = r

arch = defaultdict(dict)
if ARCH.exists():
    for r in json.load(open(ARCH)):
        if not r.get("failed"):
            arch[r["arm"]][r["scenario"]] = r

print("=" * 90)
print("ORCA-RESPONSIVE PEDESTRIAN STUDY (Amendments E, E.1)")
print("Post-confirmation model-generalisation experiment.")
print("NOT an independent confirmation. Paired initial conditions from the")
print("frozen confirmatory scenarios; outcomes newly generated.")
print("=" * 90)

# ---- integrity ---------------------------------------------------------
print("\nINTEGRITY")
for setname in sorted(by):
    for resp in sorted(by[setname]):
        arms = by[setname][resp]
        sizes = {a: len(v) for a, v in arms.items()}
        ok = len(set(sizes.values())) == 1
        print(f"  {setname:<8} {resp:<12} arms={sorted(sizes)} "
              f"n={sorted(set(sizes.values()))} {'OK' if ok else '<-- UNEVEN'}")


def vec(setname, resp, arm, scens, cat="safe"):
    d = by[setname][resp][arm]
    return np.array([1.0 if category(d[s]) == cat else 0.0 for s in scens])


def matrix(a_map, b_map, scens):
    m = {u: {v: 0 for v in CATS} for u in CATS}
    for s in scens:
        m[category(a_map[s])][category(b_map[s])] += 1
    return m


def show_transitions(m, label, n):
    print(f"\n  {label}")
    print(f"    {'':<10}" + "".join(f"{c:>10}" for c in CATS))
    for u in CATS:
        print(f"    {u:<10}" + "".join(f"{m[u][v]:>10}" for v in CATS))
    key = [("unsafe", "safe", "rescue"), ("safe", "unsafe", "safety LOST"),
           ("unsafe", "deadlock", "safety via abandoning progress"),
           ("safe", "deadlock", "harmful overconstraint")]
    for u, v, txt in key:
        print(f"    {u+' -> '+v:<22} {m[u][v]:>4}  {100*m[u][v]/n:>5.1f}%  {txt}")


for setname in ("confirm", "dev"):
    if setname not in by:
        continue
    for resp in sorted(by[setname]):
        arms = by[setname][resp]
        if "disabled" not in arms or "ttc0.40" not in arms:
            continue
        scens = sorted(set(arms["disabled"]) & set(arms["ttc0.40"]))
        tag = ("ORCA-responsive pedestrians" if resp == "reciprocal"
               else "ORCA-responsive pedestrians, inflated robot radius")
        role = "PRIMARY" if (setname == "confirm" and resp == "reciprocal") else "secondary"
        print(f"\n{'#' * 90}\n## {setname} / {tag}   [{role}]   n={len(scens)}\n{'#' * 90}")

        rates = {a: {c: float(np.mean([category(arms[a][s]) == c for s in scens]))
                     for c in CATS} for a in sorted(arms)}
        print(f"\n  {'arm':<12}" + "".join(f"{c:>10}" for c in CATS))
        for a in ("disabled", "ttc0.40", "ttc0.20"):
            if a in rates:
                print(f"  {a:<12}" + "".join(f"{100*rates[a][c]:>10.1f}" for c in CATS))

        d, lo, hi = paired_ci(vec(setname, resp, "disabled", scens),
                              vec(setname, resp, "ttc0.40", scens),
                              f"{setname}/{resp}/primary")
        print(f"\n  PRIMARY  p_safe(ttc0.40) - p_safe(disabled)")
        print(f"    {d:+.4f}  95% CI [{lo:+.4f},{hi:+.4f}]")
        print(f"    -> {branch(d, lo, hi)}")

        show_transitions(matrix(arms["disabled"], arms["ttc0.40"], scens),
                         "disabled -> ttc0.40 (mechanism test)", len(scens))

        if "ttc0.20" in arms:
            d2, lo2, hi2 = paired_ci(vec(setname, resp, "ttc0.40", scens),
                                     vec(setname, resp, "ttc0.20", scens),
                                     f"{setname}/{resp}/floor")
            print(f"\n  SECONDARY  p_safe(ttc0.20) - p_safe(ttc0.40)")
            print(f"    {d2:+.4f}  95% CI [{lo2:+.4f},{hi2:+.4f}]")
            print("    External-model evidence for a previously selected candidate.")
            print("    Does NOT replace an independent seed-disjoint SFM confirmation.")

        # ---- comparison against archived SFM outcomes -------------------
        if setname == "confirm" and arch:
            sf = sorted(set(scens) & set(arch.get("hybrid", {})))
            if sf:
                m_sfm = matrix(arch["hybrid"], arch["hybrid-ttc"], sf)
                m_orc = matrix(arms["disabled"], arms["ttc0.40"], sf)
                print(f"\n  SAME SCENARIOS, TWO PEDESTRIAN MODELS  (n={len(sf)})")
                print(f"    {'transition':<24}{'SFM peds':>10}{'ORCA-resp':>11}{'change':>9}")
                for u, v in (("safe", "unsafe"), ("unsafe", "safe"),
                             ("unsafe", "deadlock"), ("safe", "deadlock")):
                    a_, b_ = m_sfm[u][v], m_orc[u][v]
                    print(f"    {u+' -> '+v:<24}{a_:>10}{b_:>11}{b_-a_:>+9}")
                for k, lab in (("ttc_active_frac", "activation"),
                               ("stop_time", "stop time (s)"),
                               ("steps", "duration (steps)")):
                    a_ = np.mean([arch["hybrid-ttc"][s][k] for s in sf])
                    b_ = np.mean([arms["ttc0.40"][s][k] for s in sf])
                    print(f"    {lab:<24}{a_:>10.3f}{b_:>11.3f}{b_-a_:>+9.3f}")
                print("\n    Attribution (E.1): differences are attributable to replacing")
                print("    the SFM pedestrian dynamics with the specified ORCA-based model,")
                print("    NOT specifically to pedestrian yielding.")

print("\n" + "=" * 90)
print("Reading the mechanism result:")
print("  safe->unsafe falls sharply : exposure amplified by SFM pedestrian dynamics")
print("  safe->unsafe persists      : hard TTC braking creates exposure regardless")
print("  TTC benefit disappears     : benefit and cost both model-dependent")
print("  TTC benefit increases      : avoidance helps recovery from intervention")
print("  mixed across ped_yield     : conclusions bounded by responsibility allocation")
print("Each is a publishable answer. None is the hoped-for one.")
