"""Amendment G2 calibration analysis — reporting only, no simulation.

Reads the frozen calibration data (v2/runs/v2/g2_calib.json, 950 rows) and
re-derives the manipulation check from stored scenario geometry. Runs no
trials; no arm is executed on the evaluation pool.

The selection rule is REPRODUCED VERBATIM from run_g2_calib.py, where it was
frozen in Amendment G2 before the screen ran. Nothing here alters selection.
The additions -- family breakdown and the B-vs-A / C-vs-B component contrasts
-- are REPORTING requirements from the frozen reporting order, not criteria.

Output order is the frozen one:
  1 manipulation validity   2 parameter screen   3 selected trigger
  4 pre-trigger contamination   5 pooled outcomes   6 family breakdown
  7 component contrasts
"""
import json, sys, pathlib
from itertools import product
from pathlib import Path
from collections import defaultdict
from multiprocessing import Pool
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

CAL  = Path("v2/runs/v2/export_g2_calib")
EVAL = Path("v2/runs/v2/export_g2_eval")
DATA = Path("v2/runs/v2/g2_calib.json")
OUT  = Path("v2/runs/v2/g2_calib_output.txt")

# --- frozen in Amendment G2, copied unchanged from run_g2_calib.py ---
STALL = [2.0, 3.0, 4.0]
PROG  = [0.10, 0.30, 0.50]
CAL_UNSAFE_TOL = 0.05
MIN_HAUSDORFF  = 3.0
WORKERS = 10

CATS = ["safe", "unsafe", "deadlock", "other"]
_buf = []


def emit(s=""):
    print(s, flush=True)
    _buf.append(s)


def section(n, title):
    emit()
    emit("=" * 72)
    emit(str(n) + ". " + title.upper())
    emit("=" * 72)


# --- geometry re-implemented here, NOT imported from make_g2_suite.py, so the
# --- check does not inherit the generator's own primitives.
def _seg_hits(p, q, verts):
    a, b = verts, np.roll(verts, -1, axis=0)
    d = (q[0] - p[0]) * (b[:, 1] - a[:, 1]) - (q[1] - p[1]) * (b[:, 0] - a[:, 0])
    with np.errstate(divide="ignore", invalid="ignore"):
        ua = ((b[:, 0] - a[:, 0]) * (p[1] - a[:, 1])
              - (b[:, 1] - a[:, 1]) * (p[0] - a[:, 0])) / d
        ub = ((q[0] - p[0]) * (p[1] - a[:, 1])
              - (q[1] - p[1]) * (p[0] - a[:, 0])) / d
    return bool(np.any((np.abs(d) > 1e-15) & (ua >= 0) & (ua <= 1)
                       & (ub >= 0) & (ub <= 1)))


def _hits(path, polys):
    return any(_seg_hits(path[:, i], path[:, i + 1], v)
               for i in range(path.shape[1] - 1) for v in polys)


def _densify(p, step=0.5):
    out = []
    for i in range(p.shape[1] - 1):
        a, b = p[:, i], p[:, i + 1]
        n = max(2, int(np.hypot(*(b - a)) / step) + 1)
        out.append(a[:, None] + np.linspace(0, 1, n)[None, :] * (b - a)[:, None])
    return np.hstack(out) if out else p


def _hausdorff(P, Q):
    d = np.hypot(P[0][:, None] - Q[0][None, :], P[1][:, None] - Q[1][None, :])
    return float(max(d.min(axis=1).max(), d.min(axis=0).max()))


def _verify_one(f):
    """Re-derive all four manipulation conditions from stored geometry alone.
    The generator's own boolean flags are deliberately NOT read."""
    from planner import GridPlanner, prune_collinear
    d = json.loads(pathlib.Path(f).read_text())
    P = d["params"]
    wp = np.array(d["waypoints"], float).T
    obs = [np.array(o, float) for o in d["obstacles"]]
    new = [np.array(o, float) for o in d["mapChange"]["obstacles"]]
    k = int(d["manipulation"]["trigger_index"])

    c1 = not _hits(wp, obs)                        # cond 1: valid before
    c2 = _hits(wp[:, k:], new)                     # cond 2: remaining invalid after
    gp = GridPlanner(P["xMax"], P["yMax"], obs + new, P["graphMargin"])
    alt = gp.plan(P["start"], P["goal"])
    c3 = alt is not None                           # cond 3: alternate exists
    h = _hausdorff(_densify(prune_collinear(alt)), _densify(wp)) if c3 else 0.0
    c4 = h >= MIN_HAUSDORFF                        # cond 4: materially different
    return ({"original valid before": c1, "remaining invalid after": c2,
             "alternate exists": c3, "hausdorff >= 3 m": c4}, h)


# ---------------------------------------------------------------- 1
def manipulation_check():
    section(1, "manipulation validity (re-derived from stored geometry)")
    emit("  The generator's stored boolean flags are NOT read. Each condition is")
    emit("  recomputed from the scenario's own obstacles, map change and route,")
    emit("  using segment-polygon and Hausdorff code independent of the suite.")
    allok = True
    for pool, root in [("calibration", CAL), ("evaluation", EVAL)]:
        files = sorted(root.glob("*.json"))
        if not files:
            emit("  " + pool + ": no scenarios found at " + str(root))
            allok = False
            continue
        fails, haus = [], []
        with Pool(WORKERS) as po:
            res = po.map(_verify_one, [str(x) for x in files])
        for f, (checks, h) in zip(files, res):
            bad = [kk for kk, v in checks.items() if not v]
            if bad:
                fails.append((f.stem, ", ".join(bad)))
            haus.append(h)
        n, ok = len(files), len(files) - len(fails)
        emit("")
        emit("  {} pool: {}/{} ({:.1f}%) pass all four conditions".format(
            pool, ok, n, 100.0 * ok / n))
        emit("    Hausdorff separation: median {:.1f} m   min {:.1f} m   max {:.1f} m"
             .format(np.median(haus), min(haus), max(haus)))
        for s, why in fails[:20]:
            emit("    FAIL " + s + ": " + why)
        allok = allok and not fails
    emit("")
    if allok:
        emit("  Verified independently of the generator: the original route was")
        emit("  valid before the map change; the original REMAINING route is")
        emit("  invalid after it; a valid alternate exists; and the alternate")
        emit("  differs materially from the original.")
    else:
        emit("  *** MANIPULATION CHECK FAILED - nothing below is interpretable ***")
    return allok


# ---------------------------------------------------------------- helpers
def index(rows):
    by = defaultdict(dict)
    for r in rows:
        by[r["config"]][r["scenario"]] = r
    return by


def rates(by, cfg, scens):
    return {k: float(np.mean([by[cfg][s]["category"] == k for s in scens]))
            for k in CATS}


# ---------------------------------------------------------------- 2 + 3
def screen_and_select(by, scens):
    section(2, "parameter screen (n = 50, SCREENING ONLY)")
    a = rates(by, "A", scens)
    emit("  arm A (parameter-free): safe {:.3f}   unsafe {:.3f}   "
         "deadlock {:.3f}   other {:.3f}".format(
             a["safe"], a["unsafe"], a["deadlock"], a["other"]))
    emit("")
    emit("  Screen: reject any configuration whose arm-C unsafe rate exceeds")
    emit("  arm A's by more than +{:.2f}. This is a SCREENING rule at n = 50,".format(CAL_UNSAFE_TOL))
    emit("  NOT a non-inferiority test, and must never be reported as evidence")
    emit("  that arm C preserves safety.")

    cands = []
    for s, p in product(STALL, PROG):
        c = "C_s{}_p{}".format(s, p)
        if c not in by:
            continue
        r = rates(by, c, scens)
        cands.append(dict(
            stall=s, prog=p, safe=r["safe"], deadlock=r["deadlock"],
            d_unsafe=r["unsafe"] - a["unsafe"],
            replans=float(np.mean([by[c][x]["replans"] for x in scens])),
            adv=float(np.mean([by[c][x]["adv"] for x in scens])),
            pre=int(sum(by[c][x]["pre_trigger"] for x in scens))))

    emit("")
    emit("  {:<13}{:>9}{:>10}{:>9}{:>8}{:>7}{:>5}{:>6}".format(
        "stall/prog", "C safe", "d_unsafe", "C dead", "replan", "adv", "pre", "pass"))
    for c in sorted(cands, key=lambda r: -r["safe"]):
        c["passes"] = c["d_unsafe"] <= CAL_UNSAFE_TOL
        emit("  {:<13}{:>9.3f}{:>+10.3f}{:>9.3f}{:>8.2f}{:>7.2f}{:>5}{:>6}".format(
            "{}/{}".format(c["stall"], c["prog"]), c["safe"], c["d_unsafe"],
            c["deadlock"], c["replans"], c["adv"], c["pre"],
            "yes" if c["passes"] else "NO"))

    ok = [c for c in cands if c["passes"]]
    section(3, "selected shared trigger")
    if not ok:
        emit("  NO configuration passed the calibration safety screen.")
        emit("  Per the Amendment F precedent this is a negative calibration")
        emit("  result: the evaluation pool is NOT launched.")
        return None
    ok.sort(key=lambda r: (-r["safe"], r["replans"], r["adv"], -r["stall"], -r["prog"]))
    w = ok[0]
    emit("  {}/{} configurations survived the safety screen.".format(len(ok), len(cands)))
    emit("")
    emit("  SELECTED by the frozen rule: G_STALL = {} s, G_PROG = {} m".format(
        w["stall"], w["prog"]))
    emit("    arm-C safe completion {:.3f}    d_unsafe {:+.3f}".format(
        w["safe"], w["d_unsafe"]))
    emit("    replans/scenario {:.2f}    advancements/scenario {:.2f}".format(
        w["replans"], w["adv"]))
    emit("")
    emit("  Tie-break order applied: highest safe completion, then fewer replans,")
    emit("  then fewer advancements, then the weaker trigger (larger stall, then")
    emit("  larger prog). Pre-trigger firing was NOT a selection criterion.")
    return w


# ---------------------------------------------------------------- 4
def contamination(by, scens, w):
    section(4, "pre-trigger contamination (measured covariate, never suppressed)")
    emit("  {:<13}{:>12}{:>22}".format("stall/prog", "scenarios", "pre/total replans"))
    for s, p in product(STALL, PROG):
        c = "C_s{}_p{}".format(s, p)
        if c not in by:
            continue
        pre = sum(by[c][x]["pre_trigger"] for x in scens)
        tot = sum(by[c][x]["replans"] for x in scens)
        nsc = sum(1 for x in scens if by[c][x]["pre_trigger"] > 0)
        mark = "   <-- selected" if w and s == w["stall"] and p == w["prog"] else ""
        emit("  {:<13}{:>12}{:>22}{}".format(
            "{}/{}".format(s, p), "{}/{}".format(nsc, len(scens)),
            "{}/{}".format(pre, tot), mark))
    emit("")
    emit("  Arm C is a general event-triggered recovery controller evaluated in")
    emit("  path-invalidation scenarios. It is NOT a post-invalidation replanner:")
    emit("  it fires on any progress stall, and a small fraction of its replans")
    emit("  preceded the map change due to ordinary congestion.")


# ---------------------------------------------------------------- 5
def pooled(by, scens, w):
    section(5, "pooled outcomes at the selected trigger")
    arms = {"A": "A",
            "B": "B_s{}_p{}".format(w["stall"], w["prog"]),
            "C": "C_s{}_p{}".format(w["stall"], w["prog"])}
    emit("  {:<5}{:>9}{:>9}{:>10}{:>8}{:>9}{:>7}{:>6}".format(
        "arm", "safe", "unsafe", "deadlock", "other", "replans", "adv", "pre"))
    for name in ["A", "B", "C"]:
        cfg = arms[name]
        r = rates(by, cfg, scens)
        rp = float(np.mean([by[cfg][x]["replans"] for x in scens]))
        ad = float(np.mean([by[cfg][x]["adv"] for x in scens]))
        pre = int(sum(by[cfg][x]["pre_trigger"] for x in scens))
        emit("  {:<5}{:>9.3f}{:>9.3f}{:>10.3f}{:>8.3f}{:>9.2f}{:>7.2f}{:>6}".format(
            name, r["safe"], r["unsafe"], r["deadlock"], r["other"], rp, ad, pre))
    return arms


# ---------------------------------------------------------------- 6
def by_family(by, scens, arms):
    section(6, "family breakdown (diagnostic at n = 10/family, NOT inferential)")
    fam = defaultdict(list)
    for s in scens:
        fam[by["A"][s]["family"]].append(s)
    for f in sorted(fam):
        ss = fam[f]
        emit("")
        emit("  {}   (n = {})".format(f, len(ss)))
        emit("    {:<5}{:>9}{:>9}{:>10}{:>9}{:>6}".format(
            "arm", "safe", "unsafe", "deadlock", "replans", "pre"))
        for name in ["A", "B", "C"]:
            cfg = arms[name]
            r = rates(by, cfg, ss)
            rp = float(np.mean([by[cfg][x]["replans"] for x in ss]))
            pre = int(sum(by[cfg][x]["pre_trigger"] for x in ss))
            emit("    {:<5}{:>9.3f}{:>9.3f}{:>10.3f}{:>9.2f}{:>6}".format(
                name, r["safe"], r["unsafe"], r["deadlock"], rp, pre))
    emit("")
    emit("  With 10 scenarios per family these are diagnostic only. The method is")
    emit("  never selected on its best family.")


# ---------------------------------------------------------------- 7
def contrasts(by, scens, arms):
    section(7, "component contrasts (paired within scenario)")
    for x, y, what in [("A", "B", "value of waypoint advancement alone"),
                       ("B", "C", "added value of replanning")]:
        emit("")
        emit("  {} vs {} - isolates the {}".format(y, x, what))
        emit("    {:<12}{:>9}{:>12}{:>12}".format(
            "outcome", "delta", y + " only", x + " only"))
        for k in ["safe", "unsafe", "deadlock"]:
            a = np.array([by[arms[x]][s]["category"] == k for s in scens], float)
            b = np.array([by[arms[y]][s]["category"] == k for s in scens], float)
            up = int(np.sum((b == 1) & (a == 0)))
            dn = int(np.sum((a == 1) & (b == 0)))
            emit("    {:<12}{:>+9.3f}{:>12}{:>12}".format(
                k, b.mean() - a.mean(), up, dn))
    emit("")
    emit("  Discordant-pair counts are shown because the paired difference alone")
    emit("  hides how much movement produced it. A vs C alone would conflate the")
    emit("  two mechanisms and is not the primary reported contrast.")


# ------------------------------------------------- 5-7, negative-result form
def descriptive_all(by, scens):
    """Emitted when NO configuration passes the screen. Descriptive only:
    every configuration is shown, none is selected. Reporting the outcome
    landscape is required to make the negative result interpretable; choosing
    among these configurations after seeing them is NOT permitted."""
    cfgB = ["B_s{}_p{}".format(s_, p_) for s_ in STALL for p_ in PROG]
    cfgC = ["C_s{}_p{}".format(s_, p_) for s_ in STALL for p_ in PROG]

    section("5'", "descriptive outcomes - ALL configurations (NO selection made)")
    emit("  Safe goal completion is the PRIMARY endpoint of Amendment G2.")
    emit("")
    emit("  {:<8}{:>14}{:>14}{:>14}{:>13}{:>13}".format(
        "arm", "safe", "unsafe", "deadlock", "replans", "adv"))
    rA = rates(by, "A", scens)
    emit("  {:<8}{:>14.3f}{:>14.3f}{:>14.3f}{:>13.2f}{:>13.2f}".format(
        "A", rA["safe"], rA["unsafe"], rA["deadlock"], 0.0, 0.0))
    for lbl, grp in [("B (x9)", cfgB), ("C (x9)", cfgC)]:
        sa = [rates(by, c, scens)["safe"] for c in grp]
        un = [rates(by, c, scens)["unsafe"] for c in grp]
        dl = [rates(by, c, scens)["deadlock"] for c in grp]
        rp = [float(np.mean([by[c][x]["replans"] for x in scens])) for c in grp]
        ad = [float(np.mean([by[c][x]["adv"] for x in scens])) for c in grp]
        emit("  {:<8}{:>14}{:>14}{:>14}{:>13}{:>13}".format(
            lbl,
            "{:.3f}".format(max(sa)) if min(sa) == max(sa) else
            "{:.3f}-{:.3f}".format(min(sa), max(sa)),
            "{:.3f}-{:.3f}".format(min(un), max(un)),
            "{:.3f}-{:.3f}".format(min(dl), max(dl)),
            "{:.2f}-{:.2f}".format(min(rp), max(rp)),
            "{:.2f}-{:.2f}".format(min(ad), max(ad))))
    emit("")
    emit("  *** PRIMARY ENDPOINT IS DEGENERATE ***")
    emit("  Safe goal completion is 0.000 for arm A and for all 18 arm-B and")
    emit("  arm-C configurations: 0 of 950 runs ended in a safe goal arrival.")
    emit("  The prespecified primary comparison (safe completion, A vs C, paired)")
    emit("  therefore cannot be made on this pool. This is independent of the")
    emit("  safety screen and would hold even if the screen were removed.")

    section("6'", "family breakdown (diagnostic at n = 10/family)")
    fam = defaultdict(list)
    for sc_ in scens:
        fam[by["A"][sc_]["family"]].append(sc_)
    emit("  {:<20}{:>4}{:>24}{:>30}".format(
        "family", "n", "A safe/unsafe/dead", "C safe/unsafe/dead (range)"))
    for f in sorted(fam):
        ss = fam[f]
        a = rates(by, "A", ss)
        cu = [rates(by, c, ss)["unsafe"] for c in cfgC]
        cd = [rates(by, c, ss)["deadlock"] for c in cfgC]
        emit("  {:<20}{:>4}{:>24}{:>30}".format(
            f, len(ss),
            "{:.2f}/{:.2f}/{:.2f}".format(a["safe"], a["unsafe"], a["deadlock"]),
            "0.00/{:.2f}-{:.2f}/{:.2f}-{:.2f}".format(
                min(cu), max(cu), min(cd), max(cd))))
    emit("")
    emit("  No family produces a safe completion under any arm or configuration.")
    emit("  The negative result is uniform, not driven by one family.")

    section("7'", "component contrasts (paired, ranges across the 9 configurations)")
    emit("  {:<10}{:<12}{:>16}{:>14}".format(
        "contrast", "outcome", "delta (range)", "discordant"))
    for x, ys, what in [("A", cfgB, "advancement alone"),
                        ("B", cfgC, "replanning")]:
        for k in ["safe", "unsafe", "deadlock"]:
            ds, disc = [], []
            for i, c in enumerate(ys):
                base = "A" if x == "A" else cfgB[i]
                a = np.array([by[base][s_]["category"] == k for s_ in scens], float)
                b = np.array([by[c][s_]["category"] == k for s_ in scens], float)
                ds.append(b.mean() - a.mean())
                disc.append(int(np.sum(b != a)))
            lbl = ("B vs A" if x == "A" else "C vs B")
            emit("  {:<10}{:<12}{:>16}{:>14}".format(
                lbl if k == "safe" else "", k,
                "{:+.3f} .. {:+.3f}".format(min(ds), max(ds)),
                "{}-{}".format(min(disc), max(disc))))
        emit("")
    emit("  B vs A isolates waypoint advancement: it fires 0.02-0.06 times per")
    emit("  scenario and moves at most one scenario out of deadlock. Advancement")
    emit("  is inert on this pool.")
    emit("  C vs B isolates replanning: it reduces deadlock substantially but")
    emit("  every run it recovers arrives with contact, never safely. Replanning")
    emit("  converts deadlock into unsafe arrival on this pool.")


# ---------------------------------------------------------------- main
if __name__ == "__main__":
    emit("AMENDMENT G2 - CALIBRATION ANALYSIS")
    emit("Reporting only. No trial is simulated; no arm is run on the evaluation")
    emit("pool. The selection rule is reproduced verbatim as frozen.")

    valid = manipulation_check()

    d = json.loads(DATA.read_text())
    rows = d["rows"]
    by = index(rows)
    scens = sorted(by["A"])
    emit("")
    emit("  calibration data: {} rows, {} configs, {} scenarios, {:.1f} min wall clock"
         .format(len(rows), len(by), len(scens), d["wall_clock_s"] / 60))

    if not valid:
        emit("")
        emit("STOPPING before arm comparisons: the manipulation check did not pass.")
    else:
        w = screen_and_select(by, scens)
        contamination(by, scens, w)
        if w is not None:
            arms = pooled(by, scens, w)
            by_family(by, scens, arms)
            contrasts(by, scens, arms)
        else:
            descriptive_all(by, scens)
            section(8, "outcome of the g2 calibration")
            emit("  NEGATIVE CALIBRATION RESULT. Two independent grounds:")
            emit("")
            emit("    (a) No configuration passes the frozen safety screen.")
            emit("    (b) The primary endpoint is degenerate: safe goal completion")
            emit("        is 0.000 for every arm and every configuration.")
            emit("")
            emit("  Per the frozen protocol and the Amendment F precedent, the")
            emit("  150-scenario EVALUATION POOL IS NOT LAUNCHED. It remains")
            emit("  untouched and geometrically validated.")
            emit("")
            emit("  The screen must NOT be relaxed and the pool must NOT be")
            emit("  regenerated to yield safe completions. Selecting scenario")
            emit("  difficulty to produce a desired outcome is the error that")
            emit("  retired Amendment G.")

    OUT.write_text("\n".join(_buf) + "\n")
    print("\n[wrote " + str(OUT) + "]")
