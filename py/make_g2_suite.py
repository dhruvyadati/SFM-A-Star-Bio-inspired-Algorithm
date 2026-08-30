"""Amendment G2: path-invalidation challenge suite.

Run: python3 py/make_g2_suite.py

A TARGETED PATH-INVALIDATION CHALLENGE STUDY, not a representative navigation
benchmark.

The retired Amendment G tried to create recovery conditions by making geometry
harder and hoping the baseline would fail; it produced 0 deadlocks in 75
scenarios. G2 instead invalidates the route BY CONSTRUCTION: a static map
change at a frozen time blocks the remaining route, while a materially
different valid alternative is verified to exist.

CONDITION 9 IS LOAD-BEARING: no scenario is accepted or rejected on the basis
of whether arm A eventually deadlocks. Using "arm A must fail" as a filter
would rebuild the exact error that retired G. Geometry establishes
invalidation; outcomes are never consulted during generation.
"""
from __future__ import annotations

import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from planner import GridPlanner, prune_collinear
from sfm_explore import Params

OUT = Path("v2/runs/v2")
SEED_BASE = {"calib": 4_000_000_000, "eval": 4_100_000_000}   # disjoint of all prior
PER_FAMILY = {"calib": 10, "eval": 30}
MAX_ATTEMPT = 600
FAMILIES = ["corridor_closed", "waypoint_enclosed", "wall_displaces",
            "passage_switch", "pocket_formed"]
N_PED = 48
MIN_HAUSDORFF = 3.0          # "materially different" alternate route
TRIGGER_FRAC = (0.25, 0.45)  # where along the route the change fires


def rect(cx, cy, w, h):
    return np.array([[cx - w / 2, cy - h / 2], [cx + w / 2, cy - h / 2],
                     [cx + w / 2, cy + h / 2], [cx - w / 2, cy + h / 2]])


def seg_hits_poly(p, q, verts):
    a, b = verts, np.roll(verts, -1, axis=0)
    x1, y1, x2, y2 = p[0], p[1], q[0], q[1]
    x3, y3, x4, y4 = a[:, 0], a[:, 1], b[:, 0], b[:, 1]
    d = (x2 - x1) * (y4 - y3) - (y2 - y1) * (x4 - x3)
    with np.errstate(divide="ignore", invalid="ignore"):
        ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / d
        ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / d
    return bool(np.any((np.abs(d) > 1e-15) & (ua >= 0) & (ua <= 1)
                       & (ub >= 0) & (ub <= 1)))


def path_hits(path, polys):
    for i in range(path.shape[1] - 1):
        for v in polys:
            if seg_hits_poly(path[:, i], path[:, i + 1], v):
                return True
    return False


def hausdorff(P, Q):
    d = np.hypot(P[0][:, None] - Q[0][None, :], P[1][:, None] - Q[1][None, :])
    return float(max(d.min(axis=1).max(), d.min(axis=0).max()))


def densify(p, step=0.5):
    out = []
    for i in range(p.shape[1] - 1):
        a, b = p[:, i], p[:, i + 1]
        n = max(2, int(np.hypot(*(b - a)) / step) + 1)
        out.append(a[:, None] + np.linspace(0, 1, n)[None, :] * (b - a)[:, None])
    return np.hstack(out) if out else p


def point_in_poly(pt, v):
    x, y = pt
    inside, n, j = False, len(v), len(v) - 1
    for i in range(n):
        if (v[i, 1] > y) != (v[j, 1] > y):
            xi = (v[j, 0] - v[i, 0]) * (y - v[i, 1]) / (v[j, 1] - v[i, 1]) + v[i, 0]
            if x < xi:
                inside = not inside
        j = i
    return inside


def dist_to_poly(pt, v):
    x, y = pt
    a, b = v, np.roll(v, -1, axis=0)
    e = b - a
    L2 = np.maximum((e ** 2).sum(1), 1e-18)
    t = np.clip(((x - a[:, 0]) * e[:, 0] + (y - a[:, 1]) * e[:, 1]) / L2, 0, 1)
    c = a + t[:, None] * e
    return float(np.hypot(x - c[:, 0], y - c[:, 1]).min())


def base_map(family, rng, P):
    """Static obstacles present from t=0."""
    if family == "passage_switch":
        cx = float(rng.uniform(24, 30))
        return [rect(cx, 14.0, 2.0, 20.0), rect(cx, 36.0, 2.0, 20.0)]
    if family == "pocket_formed":
        # The base wall must STRADDLE the start-goal diagonal, or A* returns a
        # near-straight path that prunes below four waypoints and no trigger
        # index can be placed. The first version put it off-route and was
        # rejected 600/600 as path_too_short.
        c = float(rng.uniform(20, 30))
        return [rect(c, c + float(rng.uniform(-2, 2)),
                     float(rng.uniform(10, 16)), 2.0)]
    n = int(rng.integers(2, 5))
    obs = []
    for _ in range(n):
        for _try in range(40):
            r = rect(float(rng.uniform(8, 42)), float(rng.uniform(8, 42)),
                     float(rng.uniform(4, 8)), float(rng.uniform(4, 8)))
            if all(not (r[:, 0].max() >= o[:, 0].min() and o[:, 0].max() >= r[:, 0].min()
                        and r[:, 1].max() >= o[:, 1].min() and o[:, 1].max() >= r[:, 1].min())
                   for o in obs):
                obs.append(r)
                break
    return obs


def change_obstacle(family, rng, path, k, P):
    """A new obstacle placed to invalidate the route AT OR AFTER index k."""
    q = path[:, min(k + 1, path.shape[1] - 1)]
    nxt = path[:, min(k + 2, path.shape[1] - 1)]
    d = nxt - q
    n = float(np.hypot(*d)) or 1.0
    tx, ty = d[0] / n, d[1] / n
    px, py = -ty, tx                       # perpendicular to the route

    if family == "waypoint_enclosed":
        s = float(rng.uniform(3.0, 4.5))
        return [rect(q[0], q[1], s, s)]
    if family == "wall_displaces":
        L = float(rng.uniform(12, 18))
        c = q + np.array([tx, ty]) * 1.0
        return [np.array([[c[0] - px * L / 2 - tx, c[1] - py * L / 2 - ty],
                          [c[0] + px * L / 2 - tx, c[1] + py * L / 2 - ty],
                          [c[0] + px * L / 2 + tx, c[1] + py * L / 2 + ty],
                          [c[0] - px * L / 2 + tx, c[1] - py * L / 2 + ty]])]
    # corridor_closed, passage_switch, pocket_formed: a plug across the route
    L = float(rng.uniform(6, 10))
    c = q + np.array([tx, ty]) * float(rng.uniform(0.5, 2.0))
    return [np.array([[c[0] - px * L / 2 - tx, c[1] - py * L / 2 - ty],
                      [c[0] + px * L / 2 - tx, c[1] + py * L / 2 - ty],
                      [c[0] + px * L / 2 + tx, c[1] + py * L / 2 + ty],
                      [c[0] - px * L / 2 + tx, c[1] - py * L / 2 + ty]])]


def build(family, seed, P):
    rng = np.random.default_rng(seed)
    obs = base_map(family, rng, P)

    for i in range(len(obs)):                                       # cond 5
        for j in range(i + 1, len(obs)):
            a, b = obs[i], obs[j]
            if (a[:, 0].max() >= b[:, 0].min() and b[:, 0].max() >= a[:, 0].min()
                    and a[:, 1].max() >= b[:, 1].min() and b[:, 1].max() >= a[:, 1].min()):
                return None, "obstacle_overlap"
    for pt in (P.start, P.goal):
        for o in obs:
            if point_in_poly(pt, o) or dist_to_poly(pt, o) < P.rRobot:
                return None, "endpoint_clearance"

    gp = GridPlanner(P.xMax, P.yMax, obs, P.graphMargin)
    path0 = gp.plan(P.start, P.goal)                                # cond 1
    if path0 is None:
        return None, "no_initial_path"
    wp = prune_collinear(path0)
    if wp.shape[1] < 4:
        return None, "path_too_short"

    frac = float(rng.uniform(*TRIGGER_FRAC))
    k = max(1, min(wp.shape[1] - 3, int(frac * wp.shape[1])))
    new_obs = change_obstacle(family, rng, wp, k, P)

    for a in new_obs:                                               # cond 5
        for b in obs:
            if (a[:, 0].max() >= b[:, 0].min() and b[:, 0].max() >= a[:, 0].min()
                    and a[:, 1].max() >= b[:, 1].min() and b[:, 1].max() >= a[:, 1].min()):
                return None, "change_overlaps_base"
        for pt in (P.start, P.goal):
            if point_in_poly(pt, a) or dist_to_poly(pt, a) < P.rRobot:
                return None, "change_hits_endpoint"

    remaining = wp[:, k:]
    if not path_hits(remaining, new_obs):                           # cond 2
        return None, "remaining_path_still_valid"

    gp2 = GridPlanner(P.xMax, P.yMax, obs + new_obs, P.graphMargin)
    alt = gp2.plan(P.start, P.goal)                                 # cond 3
    if alt is None:
        return None, "no_alternate_path"
    alt = prune_collinear(alt)
    if hausdorff(densify(alt), densify(wp)) < MIN_HAUSDORFF:        # cond 4
        return None, "alternate_not_material"

    # cond 7: trigger frozen per scenario, arm-independent. Derived from the
    # nominal traverse time to the trigger point, never from robot state.
    arc = float(np.sum(np.hypot(np.diff(wp[0, :k + 1]), np.diff(wp[1, :k + 1]))))
    t_change = float(arc / max(P.v0Robot, 1e-6))

    all_obs = obs + new_obs
    pos, vel, goal = [], [], []
    tries = 0
    while len(pos) < N_PED and tries < 6000:
        tries += 1
        p = (float(rng.uniform(2, P.xMax - 1)), float(rng.uniform(2, P.yMax - 1)))
        if any(point_in_poly(p, o) or dist_to_poly(p, o) < P.rPed for o in all_obs):
            continue
        pos.append(list(p)); vel.append([0.0, 0.0])
        goal.append([float(rng.uniform(2, P.xMax - 1)),
                     float(rng.uniform(2, P.yMax - 1))])
    if len(pos) < N_PED:
        return None, "pedestrian_placement"

    return dict(
        name=f"{family}_{seed}", condition=family,
        params={k_: v for k_, v in vars(P).items() if not k_.startswith("_")},
        obstacles=[o.tolist() for o in obs],
        mapChange={"t": t_change, "obstacles": [o.tolist() for o in new_obs]},
        robotStart=list(P.start), robotVel=[0.0, 0.0],
        pedPos=pos, pedVel=vel, pedGoal=goal,
        goalPool=[[float(rng.uniform(2, P.xMax - 1)),
                   float(rng.uniform(2, P.yMax - 1))] for _ in range(400)],
        waypoints=wp.T.tolist(),
        # Manipulation-check evidence, recorded per scenario.
        manipulation={"trigger_index": int(k), "trigger_time_s": t_change,
                      "original_valid_before": True,
                      "remaining_invalid_after": True,
                      "alternate_exists": True,
                      "hausdorff_m": float(hausdorff(densify(alt), densify(wp)))},
        matlabOutcome=""), None


def main():
    P = Params()
    manifest = {}
    for pool, base in SEED_BASE.items():
        d = OUT / f"export_g2_{pool}"
        d.mkdir(parents=True, exist_ok=True)
        manifest[pool] = {}
        print(f"\n=== {pool} pool ({PER_FAMILY[pool]} per family) ===")
        for fi, fam in enumerate(FAMILIES):
            acc, rej = [], Counter()
            for k in range(MAX_ATTEMPT):
                if len(acc) >= PER_FAMILY[pool]:
                    break
                seed = base + fi * 10_000_000 + k
                sc, why = build(fam, seed, P)
                if sc is None:
                    rej[why] += 1
                    continue
                (d / f"{sc['name']}.json").write_text(json.dumps(sc))
                acc.append(seed)
            manifest[pool][fam] = dict(accepted=acc, n=len(acc),
                                       rejected=dict(rej),
                                       under_filled=len(acc) < PER_FAMILY[pool])
            flag = "  UNDER-FILLED" if len(acc) < PER_FAMILY[pool] else ""
            print(f"  {fam:<20} accepted {len(acc):>3}  rejected {sum(rej.values()):>4} "
                  f"{dict(rej)}{flag}")
    (OUT / "g2_manifest.json").write_text(json.dumps(manifest, indent=1))
    print(f"\nwrote {OUT}/g2_manifest.json")


if __name__ == "__main__":
    main()
