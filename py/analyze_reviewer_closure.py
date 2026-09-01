"""Analysis for reviewer items 1.11, 1.34, 1.37.

WRITTEN BEFORE THE RESULTS EXISTED, from the frozen protocol
(resubmission/REVIEWER_CLOSURE_PROTOCOL.md). Analysis only; nothing is
simulated here.

1.11 and 1.37 are paired confirmatory-style contrasts against the existing
`hybrid` arm. 1.34 is a DESCRIPTIVE SCREEN: no cell is ranked as a
recommendation, no level is called best, and the nominal parameters are not
changed on the basis of it.
"""
import json, sys
from math import comb
from pathlib import Path
from collections import defaultdict
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

CLOSURE = Path("v2/runs/v2/reviewer_closure.json")
CONFIRM = Path("v2/runs/v2/confirm_factorial.json")
OUT     = Path("v2/runs/v2/reviewer_closure_output.txt")

_buf = []
def emit(s=""):
    print(s, flush=True); _buf.append(s)

def section(t):
    emit(); emit("=" * 74); emit(t.upper()); emit("=" * 74)

def mcnemar_exact(b, c):
    n = b + c
    if n == 0: return 1.0
    k = min(b, c)
    return float(min(1.0, 2 * sum(comb(n, i) for i in range(k + 1)) / (2 ** n)))

def boot(x, y, n=10000, seed=11):
    rng = np.random.default_rng(seed)
    i = rng.integers(0, len(x), (n, len(x)))
    return np.percentile((y[i] - x[i]).mean(1), [2.5, 97.5])


def paired(ref, arm, keys, label, note=""):
    """Paired contrast, arm minus reference, on the shared scenario set."""
    emit(); emit("  %s   (n = %d paired scenarios)%s" % (label, len(keys), note))
    emit("    {:<14}{:>10}{:>10}{:>12}{:>22}{:>12}".format(
        "outcome", "ref", "arm", "delta", "95% CI", "McNemar p"))
    for k in ["safe", "goal", "deadlock"]:
        x = np.array([ref[s][k] for s in keys], float)
        y = np.array([arm[s][k] for s in keys], float)
        lo, hi = boot(x, y)
        up = int(np.sum((y == 1) & (x == 0)))
        dn = int(np.sum((x == 1) & (y == 0)))
        p = mcnemar_exact(up, dn)
        emit("    {:<14}{:>10.3f}{:>10.3f}{:>+12.3f}{:>22}{:>12}".format(
            k, x.mean(), y.mean(), y.mean() - x.mean(),
            "[%+.3f, %+.3f]" % (lo, hi),
            ("%.2e" % p) if p < 1e-3 else ("%.3f" % p)))
        emit("    {:<14}{:>54}".format("", "discordant  arm-only %d / ref-only %d" % (up, dn)))


if __name__ == "__main__":
    D = json.loads(CLOSURE.read_text())
    rows = D["rows"]
    conf = json.loads(CONFIRM.read_text())

    ref = {r["scenario"]: r for r in conf if r["arm"] == "hybrid"}
    got = defaultdict(dict)
    nfail = 0
    for r in rows:
        if r.get("failed"):
            nfail += 1; continue
        got[(r["item"], r["config"])]["conf_" + r["scenario"]] = r

    emit("REVIEWER ITEMS 1.11 / 1.34 / 1.37 -- RESULTS")
    emit("Frozen protocol: resubmission/REVIEWER_CLOSURE_PROTOCOL.md")
    emit("Analysis written before results existed. Executed once.")
    emit("")
    emit("  runs        : %d   (failed %d)" % (len(rows), nfail))
    emit("  wall clock  : %.1f min" % (D["wall_clock_s"] / 60))
    emit("  period      : %.1f s (1.11, frozen single value)" % D["period_s"])

    # ---- protocol validity check -------------------------------------------
    section("validity check -- 1.34 nominal cell must reproduce the reference arm")
    nom = got[("1.34", "nominal")]
    ks = sorted(set(nom) & set(ref))
    same = sum(1 for s in ks if nom[s]["safe"] == ref[s]["safe"]
               and nom[s]["goal"] == ref[s]["goal"]
               and nom[s]["deadlock"] == ref[s]["deadlock"])
    emit("  nominal cell scenarios matched to the frozen hybrid arm : %d" % len(ks))
    emit("  identical safe/goal/deadlock outcomes                   : %d/%d" % (same, len(ks)))
    ok = (same == len(ks) and len(ks) == 100)
    emit("  -> %s" % ("PASS: the screen's nominal cell reproduces the frozen arm."
                      if ok else
                      "*** FAIL: nominal does not reproduce the frozen arm. "
                      "The screen is not comparable and must not be interpreted. ***"))

    # ---- 1.11 ---------------------------------------------------------------
    section("1.11 -- A* + periodic replanning baseline")
    arm = got[("1.11", "hybrid-periodic")]
    ks = sorted(set(arm) & set(ref))
    paired(ref, arm, ks, "hybrid-periodic vs hybrid (one-shot)")
    rp = [arm[s]["replans"] for s in ks]
    fl = [arm[s]["replans_failed"] for s in ks]
    emit()
    emit("  replans per scenario: mean %.1f  median %.0f  max %d   (failed: mean %.2f)"
         % (np.mean(rp), np.median(rp), max(rp), np.mean(fl)))

    # ---- 1.37 ---------------------------------------------------------------
    section("1.37 -- collinear-pruning ablation")
    arm = got[("1.37", "hybrid-nopruning")]
    ks = sorted(set(arm) & set(ref))
    paired(ref, arm, ks, "hybrid-nopruning vs hybrid (pruned)")
    a = np.array([ref[s]["min_obs_gap"] for s in ks], float)
    b = np.array([arm[s]["min_obs_gap"] for s in ks], float)
    lo, hi = boot(a, b)
    emit()
    emit("  min obstacle gap (reviewer 1.38 -- does pruning cut corners?)")
    emit("    pruned %.3f m   unpruned %.3f m   delta %+.3f m  [%+.3f, %+.3f]"
         % (a.mean(), b.mean(), b.mean() - a.mean(), lo, hi))
    emit("    A pruned path that cut corners would show a SMALLER gap than unpruned.")

    emit()
    emit("  Components already closed by the frozen factorial, not re-run:")
    emit("    SFM local control      -> `sfm` arm")
    emit("    one-shot A* guidance   -> hybrid vs sfm, safe +0.310, discordant 170/15")
    emit("    waypoint thresholding  -> 1.34 factor rWaypoint below")

    # ---- 1.34 ---------------------------------------------------------------
    section("1.34 -- bounded one-factor-at-a-time sensitivity screen")
    emit("  DESCRIPTIVE ONLY. n = 100 per cell. No cell is ranked as a")
    emit("  recommendation, no level is 'best', and the nominal parameters are")
    emit("  NOT changed on the basis of this screen.")
    emit()
    base = got[("1.34", "nominal")]
    bk = sorted(base)
    bs = np.mean([base[s]["safe"] for s in bk])
    bg = np.mean([base[s]["goal"] for s in bk])
    bd = np.mean([base[s]["deadlock"] for s in bk])
    emit("  {:<20}{:>9}{:>9}{:>11}{:>12}".format(
        "cell", "safe", "goal", "deadlock", "d(safe)"))
    emit("  {:<20}{:>9.3f}{:>9.3f}{:>11.3f}{:>12}".format(
        "NOMINAL", bs, bg, bd, "--"))
    for fac, (lo_, hi_) in D["factors"].items():
        for lvl in (lo_, hi_):
            cfg = "%s=%s" % (fac, lvl)
            cell = got[("1.34", cfg)]
            if not cell: continue
            ks = sorted(set(cell) & set(bk))
            s_ = np.mean([cell[s]["safe"] for s in ks])
            g_ = np.mean([cell[s]["goal"] for s in ks])
            d_ = np.mean([cell[s]["deadlock"] for s in ks])
            emit("  {:<20}{:>9.3f}{:>9.3f}{:>11.3f}{:>+12.3f}".format(
                cfg, s_, g_, d_, s_ - bs))
    emit()
    emit("  Read this as a screen for FRAGILITY, not for tuning: a factor whose")
    emit("  two levels straddle nominal with small movement indicates the")
    emit("  conclusion does not hinge on that parameter.")

    OUT.write_text("\n".join(_buf) + "\n")
    print("\n[wrote %s]" % OUT)
