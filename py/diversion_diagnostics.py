"""Robustness diagnostics for the trajectory-diversion analysis.

Runs entirely off the persisted paths, so it costs seconds and never
re-simulates. Four questions, each raised because the headline metrics could
be wrong in a way the headline numbers would not reveal.

1. HAUSDORFF IS A MAXIMUM, so one local excursion can dominate it while the
   two routes are otherwise identical. Reported here as a distribution beside
   the MEDIAN nearest-route deviation, which a single excursion cannot move.
   Hausdorff is never presented as the sole diversion metric.

2. UNEQUAL DURATION. The braked run lasts longer, and the time-aligned metrics
   truncate to the shorter run. That silently discards the filtered run's
   tail -- exactly where a late contact would occur. Quantified, not assumed
   negligible.

3. SELF-INTERSECTING ROUTES. Place-aligned matching takes the nearest point on
   the other path. If a route loops or passes close to itself, that can pair
   the wrong segment: two points adjacent in space but far apart in traversal.
   Measured directly (does the matched index run backwards?) and cross-checked
   against an alternative matching that cannot make this error.

4. ARC-LENGTH MATCHING as that cross-check. Matching by cumulative distance
   travelled is monotone by construction and therefore immune to
   self-intersection, at the cost of assuming the two routes have comparable
   length. Nearest-point matching makes the opposite trade. Agreement between
   them is evidence neither artefact is driving the result.

The stored paths are subsampled every 10th step, i.e. 0.1 s apart, which is
exactly the heading stride used in the main analysis, so headings computed
here with a stride of 1 sample reproduce it rather than approximate it.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

RAW = Path("v2/runs/v2/trajectory_diversion.json")
NPZ = Path("v2/runs/v2/trajectory_diversion_paths.npz")
VMIN = 0.05
DT = 0.1          # seconds between stored samples
BACKTRACK = 10    # samples (1.0 s) of reverse matching treated as a real inversion


def headings(T):
    d = T[:, 1:] - T[:, :-1]
    return np.arctan2(d[1], d[0]), np.hypot(d[0], d[1]) / DT


def ang_diff(a, b):
    return np.abs(np.arctan2(np.sin(a - b), np.cos(a - b)))


def arclen(T):
    d = np.hypot(np.diff(T[0]), np.diff(T[1]))
    return np.concatenate([[0.0], np.cumsum(d)])


def summarise(name, v, unit=""):
    v = np.asarray(v, float)
    v = v[~np.isnan(v)]
    if not len(v):
        print(f"  {name:<34} n/a")
        return
    q = np.percentile(v, [25, 50, 75, 90])
    print(f"  {name:<34} n={len(v):>3}  median {q[1]:7.3f}  "
          f"IQR [{q[0]:6.3f},{q[2]:6.3f}]  p90 {q[3]:7.3f}  max {v.max():7.3f} {unit}")


raw = json.load(open(RAW))
paths = np.load(NPZ)
groups = list(raw)

print("=" * 88)
print("TRAJECTORY-DIVERSION DIAGNOSTICS — robustness of the headline metrics")
print("=" * 88)

for g in groups:
    rows = raw[g]
    print(f"\n{'#' * 88}\n## {g}   (n={len(rows)})\n{'#' * 88}")

    haus, nn_med, nn_mean, pa_near, pa_arc = [], [], [], [], []
    inv_frac, backmax, selfprox = [], [], []
    dur_a, dur_b, trunc_frac = [], [], []
    agree = []

    for r in rows:
        s = r["scenario"]
        A, B = paths[f"{g}|{s}|A"], paths[f"{g}|{s}|B"]
        haus.append(r["hausdorff"])
        nn_mean.append(r["nn_dev_mean"])

        # --- 2. duration -------------------------------------------------
        na, nb = A.shape[1], B.shape[1]
        dur_a.append(na * DT)
        dur_b.append(nb * DT)
        trunc_frac.append(abs(nb - na) / max(na, nb))

        ha, sa = headings(A)
        hb, sb = headings(B)

        # --- nearest-point matching (as used in the main analysis) --------
        Pa, Pb = A[:, :len(ha)], B[:, :len(hb)]
        d = np.hypot(Pb[0][:, None] - Pa[0][None, :], Pb[1][:, None] - Pa[1][None, :])
        nn = np.argmin(d, axis=1)
        dev = d[np.arange(len(nn)), nn]
        nn_med.append(float(np.median(dev)))

        ok = (sb[:len(nn)] > VMIN) & (sa[nn] > VMIN)
        hd_near = np.degrees(ang_diff(hb[:len(nn)][ok], ha[nn][ok])) if ok.any() else np.array([])
        pa_near.append(float(np.median(hd_near)) if len(hd_near) else np.nan)

        # --- 3. does the matched index run backwards? --------------------
        # A monotone match means B traverses A's route in the same order. Large
        # reverse jumps mean nearest-point matching has paired a later part of
        # B with an earlier part of A -- the self-intersection failure mode.
        step = np.diff(nn.astype(float))
        inv_frac.append(float(np.mean(step < -BACKTRACK)))
        backmax.append(float(-step.min() * DT) if len(step) else 0.0)

        # How close does A come to itself at points far apart in traversal?
        # Small values mean nearest-point matching is genuinely ambiguous.
        idx = np.arange(Pa.shape[1])
        far = np.abs(idx[:, None] - idx[None, :]) > 50      # > 5 s apart
        daa = np.hypot(Pa[0][:, None] - Pa[0][None, :], Pa[1][:, None] - Pa[1][None, :])
        selfprox.append(float(daa[far].min()) if far.any() else np.nan)

        # --- 4. arc-length matching, monotone by construction -------------
        la, lb = arclen(A)[:len(ha)], arclen(B)[:len(hb)]
        j = np.clip(np.searchsorted(la, lb), 0, len(la) - 1)
        ok2 = (sb[:len(j)] > VMIN) & (sa[j] > VMIN)
        hd_arc = np.degrees(ang_diff(hb[:len(j)][ok2], ha[j][ok2])) if ok2.any() else np.array([])
        pa_arc.append(float(np.median(hd_arc)) if len(hd_arc) else np.nan)

        if len(hd_near) and len(hd_arc):
            agree.append(abs(np.median(hd_near) - np.median(hd_arc)))

    print("\n  1. SPATIAL DEVIATION — Hausdorff is a max; the median is not")
    summarise("Hausdorff (max-based)", haus, "m")
    summarise("nearest-route deviation, median", nn_med, "m")
    summarise("nearest-route deviation, mean", nn_mean, "m")
    h, m = np.array(haus, float), np.array(nn_med, float)
    ok = m > 1e-9
    if ok.any():
        print(f"  ratio Hausdorff / median deviation: median "
              f"{np.median(h[ok] / m[ok]):.1f}x  max {np.max(h[ok] / m[ok]):.1f}x")
        print("  A large ratio means Hausdorff is set by a brief local excursion")
        print("  rather than a sustained route change.")

    print("\n  2. DURATION AND TRUNCATION")
    summarise("unfiltered duration", dur_a, "s")
    summarise("filtered duration", dur_b, "s")
    summarise("fraction of longer run truncated", trunc_frac)
    n_late = sum(1 for r in rows
                 if np.isnan(r.get("sep_at_unsafe", np.nan))
                 and not np.isnan(r.get("offset_at_unsafe", np.nan)))
    print(f"  contacts occurring AFTER the unfiltered run ended: {n_late}/{len(rows)}")
    print("  (time-aligned separation is undefined for those; the offset-to-route")
    print("   metric still covers them, which is why both are reported)")

    print("\n  3. SELF-INTERSECTION RISK IN NEAREST-POINT MATCHING")
    summarise("fraction of matches running backwards", inv_frac)
    summarise("largest reverse jump", backmax, "s")
    summarise("route self-proximity (>5 s apart)", selfprox, "m")
    print("  Self-proximity far above the ~0.1 m deviation scale means points")
    print("  adjacent in space are also adjacent in traversal, so matching is")
    print("  unambiguous. Values near the deviation scale would make it fragile.")

    print("\n  4. PLACE-ALIGNED HEADING UNDER TWO INDEPENDENT MATCHINGS")
    summarise("nearest-point matching", pa_near, "deg")
    summarise("arc-length matching (monotone)", pa_arc, "deg")
    summarise("per-scenario disagreement", agree, "deg")
    print("  Arc-length matching cannot pair the wrong segment of a looping")
    print("  route. Agreement between the two is evidence that neither the")
    print("  self-intersection artefact nor the length assumption drives the result.")

print("\n" + "=" * 88)
print("These diagnostics test the METRICS, not the mechanism. They say whether")
print("the reported numbers mean what they appear to mean; the exposure-versus-")
print("diversion reading rests on the main analysis, and only within the")
print("outcome-selected transition classes sampled here.")
