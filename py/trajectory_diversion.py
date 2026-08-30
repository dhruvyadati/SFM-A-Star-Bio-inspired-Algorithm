"""Does TTC change the ROUTE, or only the dwell along it?

POST-OUTCOME EXPLORATORY, and CONDITIONAL on outcome transition.

Scenarios here are selected BY their transition category. That is the right
sampling for EXPLAINING the confirmed transitions, but it means this analysis
cannot estimate how often diversion occurs in the scenario population. Doing
that would require analysing all 500 pairs, not the 111 selected here.

Equal path length does not imply equal path -- two trajectories can differ
substantially in space while totalling the same distance. The exposure reading
requires that adverse cases follow essentially the same route more slowly; the
diversion alternative requires that the filter steers the robot into a
different conflict.

METRICS. No single one settles the question; they cover different axes.

  SPATIAL   hausdorff        route-shape deviation, order- and timing-blind
  TEMPORAL  max/mean sep     time-aligned distance between the two paths
  DIRECTION max heading diff largest TIME-aligned steering disagreement.
                             CONFOUNDED BY LAG: the braked run sits at a
                             different point of the same route at equal time,
                             so a curving route produces a large difference
                             with no steering change at all. Reported, but it
                             cannot on its own indicate diversion.
            heading @ act    time-aligned difference near first sustained
                             activation, same confound
            head PLACE       heading difference at the same LOCATION, each
                             point of the filtered path matched to its nearest
                             point on the unfiltered path. Lag removed, so
                             this is the metric that separates a genuine
                             steering change from delay along a shared route.
            nn deviation     how far the filtered path sits from the
                             unfiltered route, one-sided
  LOCUS     sep @ first act  time-aligned separation when the filter first fires
            sep @ unsafe     time-aligned separation at the contact step
            offset @ unsafe  distance from the contact point to the NEAREST
                             point on the unfiltered path

INTERPRETATION
  small hausdorff + small aligned sep + small heading change
        -> lingering on essentially the same route (exposure)
  small hausdorff + large aligned sep
        -> similar geometry traversed with different timing
  large hausdorff
        -> genuine local diversion
  large heading deviation near activation
        -> controller coupling, even if total path length is unchanged

Raw per-scenario values are written to trajectory_diversion.json so that any
summary statistic can be recomputed without re-simulating.
"""
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from sfm_explore import Scenario, run_trial

SUB = 10          # subsample to 0.1 s for the Hausdorff point sets
HSTRIDE = 10      # finite-difference stride for heading, 0.1 s
VMIN = 0.05       # heading undefined below this speed (m/s)
ACTWIN = 100      # +/- 1.0 s window around sustained activation
RNG = np.random.default_rng(577)


def category(r):
    if r["outcome"] == "success":
        return "safe" if r["contacts"] == 0 else "unsafe"
    return "deadlock" if r["outcome"] == "deadlock" else "other"


def hausdorff(P, Q):
    """Symmetric Hausdorff distance between two point sets, vectorised.

    Substituted for discrete Frechet on COMPUTATIONAL grounds only: that
    implementation is a Python double loop over a ~1500x1500 matrix per pair
    and did not complete a single scenario in 18 minutes of CPU. The exit
    status of that run says nothing whatever about trajectory behaviour.

    This is NOT a drop-in equivalent for Frechet. Hausdorff measures whether
    the two trajectories occupy similar spatial REGIONS; Frechet additionally
    respects traversal ORDER. Here Hausdorff carries the spatial-deviation
    question alone, and the time-aligned separation and heading metrics carry
    the temporal and directional questions Frechet would have partly covered.
    """
    d = np.hypot(P[:, None, 0] - Q[None, :, 0], P[:, None, 1] - Q[None, :, 1])
    return float(max(d.min(axis=1).max(), d.min(axis=0).max()))


def headings(T):
    """Heading angle and speed from position differences, stride HSTRIDE.

    Differentiating at the raw 0.01 s step is dominated by numerical noise
    when the robot is nearly stopped, which is exactly the regime the filter
    creates. The 0.1 s stride is long enough to give a stable direction and
    short enough to resolve an avoidance turn.
    """
    d = T[:, HSTRIDE:] - T[:, :-HSTRIDE]
    spd = np.hypot(d[0], d[1]) / (HSTRIDE * 0.01)
    return np.arctan2(d[1], d[0]), spd


def ang_diff(a, b):
    """Absolute angular difference wrapped to [0, pi]."""
    return np.abs(np.arctan2(np.sin(a - b), np.cos(a - b)))


def place_aligned(A, B):
    """Compare the two runs at the same PLACE rather than the same time.

    Time-aligned heading is confounded by lag. The braked run is at a
    different point on the SAME route at equal time, so anywhere the route
    curves the headings differ even when the filter has changed no steering
    decision at all. That metric cannot separate diversion from delay.

    Matching each point of B to its nearest point on A removes the lag: it
    asks whether, at a given place, the filtered run was heading the way the
    unfiltered run headed when it passed through. A large difference here is
    a genuine steering change; a large TIME-aligned difference with a small
    place-aligned one is delay along a shared route.

    Returns (median deg, 90th pct deg, mean nn deviation m, max nn deviation m).
    """
    ha, sa = headings(A)
    hb, sb = headings(B)
    Pa, Pb = A[:, :len(ha)], B[:, :len(hb)]
    d = np.hypot(Pb[0][:, None] - Pa[0][None, :], Pb[1][:, None] - Pa[1][None, :])
    nn = np.argmin(d, axis=1)
    dev = d[np.arange(len(nn)), nn]
    ok = (sb[:len(nn)] > VMIN) & (sa[nn] > VMIN)
    if not ok.any():
        return float("nan"), float("nan"), float(dev.mean()), float(dev.max())
    hd = np.degrees(ang_diff(hb[:len(nn)][ok], ha[nn][ok]))
    return (float(np.median(hd)), float(np.percentile(hd, 90)),
            float(dev.mean()), float(dev.max()))


def metrics(S, keep=None):
    Ra = run_trial(S, "hybrid", log=True, ttc=False)
    Rb = run_trial(S, "hybrid", log=True, ttc=True)
    A, B = Ra.traj, Rb.traj
    if keep is not None:
        # Persist the subsampled paths so any later metric can be computed
        # without re-simulating. Every earlier metric change cost a full
        # re-run of the whole group; this removes that cost.
        keep["A"], keep["B"] = A[:, ::SUB].copy(), B[:, ::SUB].copy()
    n = min(A.shape[1], B.shape[1])
    sep = np.hypot(A[0, :n] - B[0, :n], A[1, :n] - B[1, :n])

    ha, sa = headings(A)
    hb, sb = headings(B)
    m = min(len(ha), len(hb))
    # Heading is only defined where BOTH runs are actually moving. Comparing a
    # moving run against a stopped one yields a meaningless angle, and the
    # stopped case is already captured by the separation and stop-time metrics.
    ok = (sa[:m] > VMIN) & (sb[:m] > VMIN)
    hd = ang_diff(ha[:m], hb[:m])
    max_hd = float(np.degrees(hd[ok].max())) if ok.any() else float("nan")

    fa, sact, ue = (Rb.first_activation_step, Rb.sustained_activation_step,
                    Rb.unsafe_event_step)

    hd_act = float("nan")
    if sact > 0:
        lo, hi = max(0, sact - ACTWIN), min(m, sact + ACTWIN)
        w = ok[lo:hi]
        if w.any():
            hd_act = float(np.degrees(hd[lo:hi][w].max()))

    sep_fa = float(sep[fa - 1]) if 0 < fa <= n else float("nan")
    sep_ue = float(sep[ue - 1]) if 0 < ue <= n else float("nan")

    off = float("nan")
    if 0 < ue <= B.shape[1]:
        p = B[:, ue - 1]
        off = float(np.min(np.hypot(A[0] - p[0], A[1] - p[1])))

    pa_med, pa_p90, nn_mean, nn_max = place_aligned(A, B)

    return dict(
        max_sep=float(sep.max()), mean_sep=float(sep.mean()),
        head_place_med=pa_med, head_place_p90=pa_p90,
        nn_dev_mean=nn_mean, nn_dev_max=nn_max,
        hausdorff=hausdorff(A[:, ::SUB].T, B[:, ::SUB].T),
        max_head=max_hd, head_at_act=hd_act,
        sep_at_act=sep_fa, sep_at_unsafe=sep_ue, offset_at_unsafe=off,
        path_a=float(Ra.path_length), path_b=float(Rb.path_length),
        steps_a=int(Ra.steps), steps_b=int(Rb.steps),
    )


def iqr(v):
    return float(np.percentile(v, 25)), float(np.percentile(v, 75))


def boot_mean_ci(v, n=4000):
    idx = RNG.integers(0, len(v), size=(n, len(v)))
    m = v[idx].mean(axis=1)
    return float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))


def boot_diff_ci(x, y, n=4000):
    """Two-sample bootstrap for mean(x) - mean(y).

    The groups are different scenarios, so this is unpaired: each group is
    resampled independently. It is not the paired interval used for the
    confirmatory contrasts and carries no confirmatory status.
    """
    ix = RNG.integers(0, len(x), size=(n, len(x)))
    iy = RNG.integers(0, len(y), size=(n, len(y)))
    d = x[ix].mean(axis=1) - y[iy].mean(axis=1)
    return float(np.mean(d)), float(np.percentile(d, 2.5)), float(np.percentile(d, 97.5))


res = json.load(open("v2/runs/v2/confirm_factorial.json"))
by, cond = defaultdict(dict), {}
for r in res:
    by[r["scenario"]][r["arm"]] = r
    cond[r["scenario"]] = r["condition"]
complete = [s for s in sorted(by) if len(by[s]) == 4]


def pick(a, b):
    return [s for s in complete if category(by[s]["hybrid"]) == a
            and category(by[s]["hybrid-ttc"]) == b]


adverse = pick("safe", "unsafe")
# Comparison groups matched on SIZE only, drawn deterministically by even
# spacing over the sorted scenario list. This is not covariate matching: the
# groups are not balanced on condition or crowd density, so between-group
# differences are descriptive.
def match(pool):
    return list(np.array(pool)[np.linspace(0, len(pool) - 1, len(adverse)).astype(int)])


groups = {
    "safe -> unsafe": adverse,
    "safe -> safe": match(pick("safe", "safe")),
    "unsafe -> safe": match(pick("unsafe", "safe")),
}

raw, paths = {}, {}
for gname, ss in groups.items():
    rows = []
    for i, s in enumerate(ss, 1):
        S = Scenario.load(f"v2/runs/v2/export_confirm/{s}.json")
        keep = {}
        m = metrics(S, keep)
        paths[f"{gname}|{s}|A"], paths[f"{gname}|{s}|B"] = keep["A"], keep["B"]
        m["scenario"] = s
        m["condition"] = cond[s]
        rows.append(m)
        print(f"  [{i:>2}/{len(ss)}] {gname:<16} {s:<28} "
              f"maxSep {m['max_sep']:6.2f}  meanSep {m['mean_sep']:5.2f}  "
              f"haus {m['hausdorff']:6.2f}  headTime {m['max_head']:5.1f}d  "
              f"headPlace {m['head_place_med']:5.1f}d  nnDev {m['nn_dev_mean']:5.2f}",
              flush=True)
    raw[gname] = rows

Path("v2/runs/v2/trajectory_diversion.json").write_text(json.dumps(raw, indent=1))
np.savez_compressed("v2/runs/v2/trajectory_diversion_paths.npz", **paths)

A = {g: {k: np.array([r[k] for r in rows], dtype=float) for k in rows[0]
         if k not in ("scenario", "condition")} for g, rows in raw.items()}


def col(v):
    v = v[~np.isnan(v)]
    if not len(v):
        return "        n/a       "
    lo, hi = iqr(v)
    return f"{np.median(v):6.2f} [{lo:5.2f},{hi:5.2f}]"


print("\n" + "=" * 86)
print("PAIRED TRAJECTORY DEVIATION (hybrid vs hybrid-ttc, same scenario)")
print("POST-OUTCOME EXPLORATORY, CONDITIONAL ON TRANSITION -- median [IQR]")
print("=" * 86)
print(f"{'group':<16} {'n':>3} {'Hausdorff m':>21} {'maxSep m':>21} {'meanSep m':>21}")
print("-" * 86)
for g in groups:
    print(f"{g:<16} {len(A[g]['max_sep']):>3} {col(A[g]['hausdorff']):>21} "
          f"{col(A[g]['max_sep']):>21} {col(A[g]['mean_sep']):>21}")

print(f"\n{'group':<16} {'TIME-aligned head':>21} {'head@act deg':>21} {'sep@act m':>21}")
print("-" * 86)
for g in groups:
    print(f"{g:<16} {col(A[g]['max_head']):>21} {col(A[g]['head_at_act']):>21} "
          f"{col(A[g]['sep_at_act']):>21}")

print("\nPLACE-aligned: same location on the route, lag removed. This is the")
print("metric that separates genuine steering change from delay.")
print(f"{'group':<16} {'head med deg':>21} {'head p90 deg':>21} {'nn deviation m':>21}")
print("-" * 86)
for g in groups:
    print(f"{g:<16} {col(A[g]['head_place_med']):>21} {col(A[g]['head_place_p90']):>21} "
          f"{col(A[g]['nn_dev_mean']):>21}")

print("\n" + "=" * 86)
print("safe->unsafe MINUS safe->safe   (unpaired two-sample bootstrap, descriptive)")
print("=" * 86)
for k, lab in (("hausdorff", "Hausdorff m"), ("max_sep", "max sep m"),
               ("mean_sep", "mean sep m"), ("max_head", "max heading deg"),
               ("head_at_act", "heading @ act deg"), ("sep_at_act", "sep @ act m"),
               ("head_place_med", "head PLACE med deg"), ("head_place_p90", "head PLACE p90 deg"),
               ("nn_dev_mean", "nn deviation m")):
    x, y = A["safe -> unsafe"][k], A["safe -> safe"][k]
    x, y = x[~np.isnan(x)], y[~np.isnan(y)]
    if not len(x) or not len(y):
        continue
    d, lo, hi = boot_diff_ci(x, y)
    flag = "" if lo <= 0 <= hi else "  *"
    print(f"  {lab:<20} {d:>+8.2f}  [{lo:+7.2f},{hi:+7.2f}]{flag}")
print("  * interval excludes zero")

print("\n" + "=" * 86)
print("Where the contact happens, safe->unsafe group")
print("=" * 86)
for k, lab in (("sep_at_unsafe", "time-aligned separation at the contact step"),
               ("offset_at_unsafe", "distance from contact to NEAREST point on unfiltered path")):
    v = A["safe -> unsafe"][k]
    v = v[~np.isnan(v)]
    lo, hi = iqr(v)
    print(f"  {lab}\n    n={len(v)}  median {np.median(v):.2f}  IQR [{lo:.2f},{hi:.2f}]  "
          f"max {v.max():.2f} m")

print("\nA small offset means the contact happened essentially ON the original")
print("route, supporting exposure. A large offset means the filter steered the")
print("robot somewhere the unfiltered run never went.")
