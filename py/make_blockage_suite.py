"""Amendment G: hand-authored blockage challenge suite.

Run: python3 py/make_blockage_suite.py

A TARGETED CHALLENGE SET FOR FAILURE-MODE EVALUATION. It is not a random
sample and must never be used to claim population-level prevalence or general
benchmark performance. The ordinary four-condition benchmark carries roughly
2% deadlock without TTC and is therefore too thin to study planner recovery;
these families are constructed to exercise it.

Geometry rules and feasibility checks were fixed in Amendment G BEFORE any
scenario was authored. Rejections are counted by reason and reported per
family; a family that cannot fill its quota is reported under-filled rather
than having its rules relaxed.

Pools use disjoint seed families, both disjoint from confirmation, development
and the Amendment F calibration set (all of which lie below 2^31; F uses
[2^31, 2^31+4e6)).
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
SEED_BASE = {"calib": 3_000_000_000, "eval": 3_100_000_000}   # disjoint of all prior
PER_FAMILY = {"calib": 15, "eval": 40}
MAX_ATTEMPT = 400
FAMILIES = ["narrow_passage", "temporary_obstruction", "concave_pocket",
            "displaced_waypoint", "blocked_waypoint"]
N_PED = 48


def rect(cx, cy, w, h):
    return np.array([[cx - w / 2, cy - h / 2], [cx + w / 2, cy - h / 2],
                     [cx + w / 2, cy + h / 2], [cx - w / 2, cy + h / 2]])


def polys_overlap(a, b):
    """Conservative AABB overlap test; obstacles here are axis-aligned boxes."""
    return not (a[:, 0].max() < b[:, 0].min() or b[:, 0].max() < a[:, 0].min()
                or a[:, 1].max() < b[:, 1].min() or b[:, 1].max() < a[:, 1].min())


def point_in_poly(p, v):
    x, y = p
    inside = False
    n = len(v)
    j = n - 1
    for i in range(n):
        if (v[i, 1] > y) != (v[j, 1] > y):
            xi = (v[j, 0] - v[i, 0]) * (y - v[i, 1]) / (v[j, 1] - v[i, 1]) + v[i, 0]
            if x < xi:
                inside = not inside
        j = i
    return inside


def dist_to_poly(p, v):
    x, y = p
    a, b = v, np.roll(v, -1, axis=0)
    e = b - a
    L2 = np.maximum((e ** 2).sum(1), 1e-18)
    t = np.clip(((x - a[:, 0]) * e[:, 0] + (y - a[:, 1]) * e[:, 1]) / L2, 0, 1)
    c = a + t[:, None] * e
    return float(np.hypot(x - c[:, 0], y - c[:, 1]).min())


# ---------------------------------------------------------------------------
# Geometry families. Each returns (obstacles, blocking_peds) where
# blocking_peds is a list of (position, hold_seconds) for stationary blockers.


def f_narrow_passage(rng, P):
    """Two walls leaving a gap of width w in [1.2, 2.0] m across the diagonal."""
    w = float(rng.uniform(1.2, 2.0))
    cx = float(rng.uniform(22, 30))
    cy = cx + float(rng.uniform(-3, 3))
    ang = rng.uniform(0, 1) < 0.5
    span = 14.0
    if ang:
        obs = [rect(cx, cy - (w / 2 + span / 2), 2.0, span),
               rect(cx, cy + (w / 2 + span / 2), 2.0, span)]
    else:
        obs = [rect(cx - (w / 2 + span / 2), cy, span, 2.0),
               rect(cx + (w / 2 + span / 2), cy, span, 2.0)]
    return obs, []


def f_temporary_obstruction(rng, P):
    """A narrow passage whose gap is held by stationary pedestrians.

    Release is TIME-TRIGGERED from the seed and never conditioned on robot
    state, or the family becomes a hand-tuned rescue case.
    """
    obs, _ = f_narrow_passage(rng, P)
    gx = float(np.mean([o[:, 0].mean() for o in obs]))
    gy = float(np.mean([o[:, 1].mean() for o in obs]))
    hold = float(rng.uniform(8.0, 20.0))
    blockers = [((gx + float(rng.uniform(-0.4, 0.4)),
                  gy + float(rng.uniform(-0.4, 0.4))), hold)
                for _ in range(2)]
    return obs, blockers


def f_concave_pocket(rng, P):
    """U-shaped obstacle opening toward the start, straddling the direct line.

    ONE polygon, not three rectangles. A U built from three boxes requires its
    arms to meet the far wall, which the no-overlap feasibility check
    correctly rejects -- the first attempt failed 400/400 for exactly that
    reason. The check is not weakened; the geometry is expressed properly.
    """
    cx = float(rng.uniform(20, 30))
    cy = cx + float(rng.uniform(-3, 3))
    d = float(rng.uniform(7, 10))
    th = 2.0
    x0, x1 = cx - d / 2, cx + d / 2
    y0, y1 = cy - d / 2, cy + d / 2
    dep = d - th                                   # notch depth
    # Counter-clockwise, opening toward -x (the approach direction).
    u = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1],
                  [x0, y1 - th], [x0 + dep, y1 - th],
                  [x0 + dep, y0 + th], [x0, y0 + th]])
    return [u], []


def f_displaced_waypoint(rng, P):
    """A long wall offset from the route, so crowd pressure can strand the
    robot on the far side with its active waypoint behind the obstacle."""
    cx = float(rng.uniform(18, 26))
    cy = cx + float(rng.uniform(4, 8))
    return [rect(cx, cy, float(rng.uniform(16, 22)), 2.0)], []


def f_blocked_waypoint(rng, P):
    """Stationary pedestrians ON an intermediate waypoint, later one visible.

    The first version placed the obstacle OFF the route, so A* returned a
    straight line pruned to two waypoints -- start and goal. With no
    intermediate waypoint there was nothing to block and nothing to advance
    to, and the family could not test its own mechanism. The distribution
    check caught it: 40/40 scenarios had identical geometry and 2 waypoints.

    The obstacle now straddles the direct line, forcing a detour and creating
    intermediate waypoints. Blockers sit on the route just before the detour.
    """
    frac = float(rng.uniform(0.38, 0.55))
    mx = P.start[0] + frac * (P.goal[0] - P.start[0])
    my = P.start[1] + frac * (P.goal[1] - P.start[1])
    # A wall across the diagonal, offset perpendicular so the path must bend
    # around one end rather than being blocked outright.
    L = float(rng.uniform(14.0, 20.0))
    off = float(rng.uniform(3.0, 6.0))
    obs = [rect(mx + off, my - off, 2.0, L)]
    hold = float(rng.uniform(10.0, 20.0))
    # Blockers slightly BEFORE the wall, on the approach the robot must take.
    bx, by = mx - 2.5, my - 2.5
    blockers = [((bx + float(rng.uniform(-0.7, 0.7)),
                  by + float(rng.uniform(-0.7, 0.7))), hold) for _ in range(4)]
    return obs, blockers


BUILDERS = dict(zip(FAMILIES, [f_narrow_passage, f_temporary_obstruction,
                               f_concave_pocket, f_displaced_waypoint,
                               f_blocked_waypoint]))


def build(family, seed, P):
    """Return a scenario dict, or (None, reason) if it fails a feasibility check."""
    rng = np.random.default_rng(seed)
    obs, blockers = BUILDERS[family](rng, P)

    for i in range(len(obs)):                                    # check 2
        for j in range(i + 1, len(obs)):
            if polys_overlap(obs[i], obs[j]):
                return None, "obstacle_overlap"
    for pt in (P.start, P.goal):                                 # check 3
        for o in obs:
            if point_in_poly(pt, o) or dist_to_poly(pt, o) < P.rRobot:
                return None, "endpoint_clearance"

    gp = GridPlanner(P.xMax, P.yMax, obs, P.graphMargin)          # check 1
    path = gp.plan(P.start, P.goal)
    if path is None:
        return None, "no_astar_path"
    wp = prune_collinear(path)

    # FAMILY VALIDITY, not a new feasibility check. The frozen geometry rule for
    # `blocked_waypoint` is "pedestrians on the ACTIVE waypoint while a LATER
    # waypoint retains line of sight". A path pruned to two waypoints has only
    # start and goal: no active intermediate waypoint to block, and nothing to
    # advance to. Such a draw does not instantiate the family, so it is
    # rejected and counted rather than admitted as a degenerate member.
    if family == "blocked_waypoint" and wp.shape[1] < 3:
        return None, "no_intermediate_waypoint"

    pos, vel, goal = [], [], []
    for p, _h in blockers:
        pos.append(list(p)); vel.append([0.0, 0.0]); goal.append(list(p))
    tries = 0
    while len(pos) < N_PED and tries < 4000:                      # check 4
        tries += 1
        p = (float(rng.uniform(2, P.xMax - 1)), float(rng.uniform(2, P.yMax - 1)))
        if any(point_in_poly(p, o) or dist_to_poly(p, o) < P.rPed for o in obs):
            continue
        pos.append(list(p)); vel.append([0.0, 0.0])
        goal.append([float(rng.uniform(2, P.xMax - 1)),
                     float(rng.uniform(2, P.yMax - 1))])
    if len(pos) < N_PED:
        return None, "pedestrian_placement"

    return dict(
        name=f"{family}_{seed}", condition=family,
        params={k: v for k, v in vars(P).items() if not k.startswith("_")},
        obstacles=[o.tolist() for o in obs],
        robotStart=list(P.start), robotVel=[0.0, 0.0],
        pedPos=pos, pedVel=vel, pedGoal=goal,
        goalPool=[[float(rng.uniform(2, P.xMax - 1)),
                   float(rng.uniform(2, P.yMax - 1))] for _ in range(400)],
        waypoints=wp.T.tolist(),
        # Stationary blockers: index and hold duration. Release is time-based.
        blockerHold=[[i, h] for i, (_p, h) in enumerate(blockers)],
        matlabOutcome=""), None


def main():
    P = Params(numPedestrians=N_PED) if "numPedestrians" in vars(Params()) else Params()
    manifest = {}
    for pool, base in SEED_BASE.items():
        d = OUT / f"export_gblock_{pool}"
        d.mkdir(parents=True, exist_ok=True)
        manifest[pool] = {}
        print(f"\n=== {pool} pool ({PER_FAMILY[pool]} per family) ===")
        for fi, fam in enumerate(FAMILIES):
            acc, rej = [], Counter()
            for k in range(MAX_ATTEMPT):
                if len(acc) >= PER_FAMILY[pool]:
                    break
                seed = base + fi * 1_000_000 + k
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
            print(f"  {fam:<24} accepted {len(acc):>3}  rejected {sum(rej.values()):>4} "
                  f"{dict(rej)}{flag}")
    (OUT / "gblock_manifest.json").write_text(json.dumps(manifest, indent=1))
    print(f"\nwrote {OUT}/gblock_manifest.json")


if __name__ == "__main__":
    main()
