"""Conjunctive verdict for exposure versus diversion.

WRITTEN BEFORE THE TRAJECTORY OUTPUTS EXISTED. The thresholds and the
conjunction below are fixed here so they cannot be chosen to fit the numbers.

The conclusion must NOT be reduced to one metric. Exposure is supported only
if the adverse (safe -> unsafe) group shows ALL FIVE of these together, each
against the matched safe -> safe group:

  1. higher activation                     the filter fired more
  2. more stopping                         it produced more stationary time
  3. longer duration                       the run spent longer in the crowd
  4. small route offset                    it stayed on essentially the same route
  5. small place-aligned heading deviation it steered essentially the same way

Any single one of these is weak. (1)-(3) alone are equally consistent with the
filter steering the robot into a slower, different route; (4)-(5) alone say
only that the route is shared, not that dwell increased. The conjunction is
what distinguishes lingering on the same path from being diverted onto another.

PRESPECIFIED THRESHOLDS, with the reasoning that fixed them:

  small route offset   median nearest-route deviation < 0.35 m, the robot's own
                       body radius. A deviation smaller than the robot's radius
                       cannot constitute taking a different route: the two paths
                       overlap within one body width.
  small heading        median place-aligned heading difference < 15 deg. The
                       planner's 8-connected graph turns in 45 deg increments,
                       so a deviation under a third of one increment is well
                       inside "the same steering decision".
  local excursion      Hausdorff >= 1.0 m WHILE median nearest-route deviation
                       < 0.35 m. Hausdorff is a maximum, so this is the exact
                       signature of a brief local departure from a shared route
                       rather than general diversion, and is reported as such.

DIRECTIONAL COMPONENTS (1)-(3) are judged by whether the unpaired bootstrap
interval for (adverse - stable) excludes zero in the predicted direction. The
groups are different scenarios, so this is descriptive, not confirmatory.
"""
from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path

import numpy as np

CONF = Path("v2/runs/v2/confirm_factorial.json")
RAW = Path("v2/runs/v2/trajectory_diversion.json")
NPZ = Path("v2/runs/v2/trajectory_diversion_paths.npz")
RNG = np.random.default_rng(4241)

R_BODY = 0.35        # m, robot radius -> "same route" bound
HEAD_TOL = 15.0      # deg, < 1/3 of one 45 deg graph increment
EXCURSION = 1.0      # m, Hausdorff at or above this with a small median offset
DT = 0.1
VMIN = 0.05


def category(r):
    if r["outcome"] == "success":
        return "safe" if r["contacts"] == 0 else "unsafe"
    return "deadlock" if r["outcome"] == "deadlock" else "other"


def boot_diff(x, y, n=4000):
    """Unpaired two-sample bootstrap for mean(x) - mean(y)."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    x, y = x[~np.isnan(x)], y[~np.isnan(y)]
    if len(x) < 2 or len(y) < 2:
        return float("nan"), float("nan"), float("nan")
    ix = RNG.integers(0, len(x), size=(n, len(x)))
    iy = RNG.integers(0, len(y), size=(n, len(y)))
    d = x[ix].mean(axis=1) - y[iy].mean(axis=1)
    return (float(x.mean() - y.mean()),
            float(np.percentile(d, 2.5)), float(np.percentile(d, 97.5)))


def headings(T):
    d = T[:, 1:] - T[:, :-1]
    return np.arctan2(d[1], d[0]), np.hypot(d[0], d[1]) / DT


def ang_diff(a, b):
    return np.abs(np.arctan2(np.sin(a - b), np.cos(a - b)))


def arclen(T):
    return np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(T[0]), np.diff(T[1])))])


# --- mediator quantities, paired within scenario --------------------------
conf = defaultdict(dict)
for r in json.load(open(CONF)):
    if not r.get("failed"):
        conf[r["scenario"]][r["arm"]] = r

raw = json.load(open(RAW))
paths = np.load(NPZ)


def med_route_offset(g, s):
    """Median nearest-route deviation, recomputed from the stored paths.

    The stored per-scenario record keeps the MEAN; the median is what a single
    local excursion cannot move, so it is what the threshold is applied to.
    """
    A, B = paths[f"{g}|{s}|A"], paths[f"{g}|{s}|B"]
    d = np.hypot(B[0][:, None] - A[0][None, :], B[1][:, None] - A[1][None, :])
    return float(np.median(d.min(axis=1)))


def arc_heading(g, s):
    """Place-aligned heading by ARC LENGTH: monotone, so a looping route
    cannot pair the wrong segment."""
    A, B = paths[f"{g}|{s}|A"], paths[f"{g}|{s}|B"]
    ha, sa = headings(A)
    hb, sb = headings(B)
    la, lb = arclen(A)[:len(ha)], arclen(B)[:len(hb)]
    j = np.clip(np.searchsorted(la, lb), 0, len(la) - 1)
    ok = (sb[:len(j)] > VMIN) & (sa[j] > VMIN)
    if not ok.any():
        return float("nan")
    return float(np.median(np.degrees(ang_diff(hb[:len(j)][ok], ha[j][ok]))))


G_ADV, G_STA = "safe -> unsafe", "safe -> safe"
metrics = {}
for g in raw:
    rows = raw[g]
    m = defaultdict(list)
    for r in rows:
        s = r["scenario"]
        b = conf[s]["hybrid-ttc"]
        a = conf[s]["hybrid"]
        m["activation"].append(b["ttc_active_frac"])
        m["d_stop"].append(b["stop_time"] - a["stop_time"])
        m["d_dur"].append((b["steps"] - a["steps"]) * 0.01)
        m["offset_med"].append(med_route_offset(g, s))
        m["head_place"].append(r["head_place_med"])
        m["head_arc"].append(arc_heading(g, s))
        m["hausdorff"].append(r["hausdorff"])
    metrics[g] = {k: np.array(v, float) for k, v in m.items()}

print("=" * 88)
print("EXPOSURE vs DIVERSION — conjunctive verdict (thresholds frozen in advance)")
print("POST-OUTCOME EXPLORATORY, conditional on transition class")
print("=" * 88)

adv, sta = metrics[G_ADV], metrics[G_STA]
print(f"\nadverse n={len(adv['activation'])}   stable n={len(sta['activation'])}")

print("\nDIRECTIONAL COMPONENTS — adverse minus stable, unpaired bootstrap")
print(f"  {'component':<34} {'adverse':>9} {'stable':>9} {'difference':>26}  holds")
print("  " + "-" * 84)
directional = [("activation", "1. higher activation", +1),
               ("d_stop", "2. more stopping (s)", +1),
               ("d_dur", "3. longer duration (s)", +1)]
holds = {}
for key, lab, sign in directional:
    d, lo, hi = boot_diff(adv[key], sta[key])
    ok = (lo > 0) if sign > 0 else (hi < 0)
    holds[lab] = ok
    print(f"  {lab:<34} {np.nanmean(adv[key]):>9.3f} {np.nanmean(sta[key]):>9.3f} "
          f"{d:>+9.3f} [{lo:+.3f},{hi:+.3f}]  {'YES' if ok else 'no'}")

print("\nABSOLUTE COMPONENTS — adverse group against the frozen thresholds")
print(f"  {'component':<34} {'median':>9} {'p90':>9} {'threshold':>12}  holds")
print("  " + "-" * 84)
absolute = [("offset_med", "4. route offset (m)", R_BODY, "< body radius"),
            ("head_place", "5a. heading, nearest-pt (deg)", HEAD_TOL, "< 15 deg"),
            ("head_arc", "5b. heading, arc-length (deg)", HEAD_TOL, "< 15 deg")]
for key, lab, thr, txt in absolute:
    v = adv[key][~np.isnan(adv[key])]
    med = float(np.median(v)) if len(v) else float("nan")
    p90 = float(np.percentile(v, 90)) if len(v) else float("nan")
    ok = med < thr
    holds[lab] = ok
    print(f"  {lab:<34} {med:>9.3f} {p90:>9.3f} {txt:>12}  {'YES' if ok else 'no'}")

print("\nHAUSDORFF — a maximum, so read only beside the median offset")
h, o = adv["hausdorff"], adv["offset_med"]
exc = int(np.sum((h >= EXCURSION) & (o < R_BODY)))
div = int(np.sum(o >= R_BODY))
print(f"  median {np.median(h):.3f} m   p90 {np.percentile(h, 90):.3f} m   max {h.max():.3f} m")
print(f"  local excursions (Hausdorff >= {EXCURSION} m but median offset < {R_BODY} m): "
      f"{exc}/{len(h)}")
print(f"  sustained route change (median offset >= {R_BODY} m):                     "
      f"{div}/{len(h)}")

print("\n" + "=" * 88)
n_hold = sum(holds.values())
for k, v in holds.items():
    print(f"  {'HOLDS' if v else 'FAILS':<6} {k}")
print(f"\n  {n_hold}/{len(holds)} components hold")
if n_hold == len(holds):
    print("\n  All components hold together: the adverse transitions are consistent")
    print("  with PROLONGED EXPOSURE on essentially the same route, not diversion.")
    if exc:
        print(f"  {exc} scenarios show a LOCAL EXCURSION (large Hausdorff, small median")
        print("  offset). Report those as local departures, NOT as general diversion.")
else:
    print("\n  The conjunction does NOT hold. Exposure is not supported as stated;")
    print("  report which components failed rather than the headline reading.")
print("\n  Conditional on the sampled transition classes. Says nothing about how")
print("  often either pattern occurs across all 500 scenarios.")
