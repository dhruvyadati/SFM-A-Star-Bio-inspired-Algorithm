"""Amendment G arms B and C: route recovery over a fixed A* plan.

Arm A is the absence of this module. Arm B is `RouteRecovery(replan=False)`.
Arm C is `RouteRecovery(replan=True)` -- **the same object with one extra
capability**, so the B--C contrast isolates replanning and nothing else.

STATIC GEOMETRY ONLY. Held pedestrians are never planner obstacles. Arm C
answers "can the planner recover when the robot is displaced or trapped by
static geometry?", not "can it route around people?". Two families exercise
replanning proper; two exercise waiting and advancement; `narrow_passage`
checks that recovery does not destabilise a constrained but valid route.

Every threshold is frozen in Amendment G. Only `G_STALL` and `G_PROG` are
calibrated; the rest are engineering choices recorded as such, because
Amendment F showed that assuming a parameter is sensible is how
non-discriminating parameters enter a study.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class RecoveryParams:
    stall_s: float = 3.0        # G_STALL   calibrated
    progress_m: float = 0.30    # G_PROG    calibrated
    clearance_m: float = 0.50   # G_CLEAR   robot radius 0.35 + TTC clearance 0.15
    max_skip: int = 3           # G_MAXSKIP bounds corner-cutting
    cooldown_s: float = 3.0     # G_COOLDOWN anti-chatter
    max_adv: int = 3            # G_MAXADV  beyond this the route is likely wrong
    max_plan: int = 2           # G_MAXPLAN from the 5a sketch, as a choice


def _seg_clear(p, q, obstacles, clearance):
    """Does segment p->q stay `clearance` from every obstacle along its length?

    Sampled at 0.1 m. A vertex-only test would miss a segment that clips an
    edge midway, which is exactly the corner-cutting the rule exists to stop.
    """
    d = float(np.hypot(*(q - p)))
    n = max(2, int(d / 0.1) + 1)
    ts = np.linspace(0.0, 1.0, n)
    pts = p[:, None] + ts[None, :] * (q - p)[:, None]
    for verts in obstacles:
        a = verts
        b = np.roll(verts, -1, axis=0)
        e = b - a
        L2 = np.maximum((e ** 2).sum(1), 1e-18)
        for k in range(pts.shape[1]):
            x, y = pts[0, k], pts[1, k]
            t = np.clip(((x - a[:, 0]) * e[:, 0] + (y - a[:, 1]) * e[:, 1]) / L2, 0, 1)
            c = a + t[:, None] * e
            if float(np.hypot(x - c[:, 0], y - c[:, 1]).min()) < clearance:
                return False
    return True


class RouteRecovery:
    """Progress-triggered waypoint advancement, optionally with replanning."""

    def __init__(self, waypoints, obstacles, P, cfg: RecoveryParams,
                 replan: bool, planner=None):
        self.wp = np.asarray(waypoints, float)
        self.obstacles = obstacles
        self.P, self.cfg = P, cfg
        self.replan_enabled = bool(replan)
        self.planner = planner

        n = cfg.stall_s / P.dt
        assert abs(n - round(n)) < 1e-9, "stall window must be a whole number of steps"
        self.win = int(round(n)) + 1          # N samples span N-1 intervals

        self.hist = []
        self.cool_until = -1.0
        self.n_adv = 0
        self.n_plan = 0
        self.n_plan_failed = 0
        self.adv_steps = []
        self.plan_steps = []

    # -- remaining arc length along the CURRENT path from waypoint index wi --
    def _remaining(self, x, wi):
        if wi >= self.wp.shape[1]:
            return 0.0
        d = float(np.hypot(*(self.wp[:, wi] - x)))
        seg = self.wp[:, wi:]
        if seg.shape[1] > 1:
            d += float(np.sum(np.hypot(np.diff(seg[0]), np.diff(seg[1]))))
        return d

    def update(self, step, x, wi):
        """Returns (new_waypoints_or_None, new_index_or_None).

        None means no intervention this step. The caller keeps its own state;
        this object never mutates the simulation.
        """
        cfg, dt = self.cfg, self.P.dt
        t_now = step * dt
        self.hist.append(self._remaining(x, wi))
        if len(self.hist) > self.win:
            self.hist.pop(0)
        if len(self.hist) < self.win or t_now < self.cool_until:
            return None, None

        # Progress test on the CURRENT path.
        if (self.hist[0] - self.hist[-1]) >= cfg.progress_m:
            return None, None

        # --- arm B: advancement, tried FIRST in both arms -----------------
        if self.n_adv < cfg.max_adv:
            hi = min(self.wp.shape[1], wi + 1 + cfg.max_skip)
            for j in range(wi + 1, hi):          # earliest feasible, not furthest
                if _seg_clear(x, self.wp[:, j], self.obstacles, cfg.clearance_m):
                    self.n_adv += 1
                    self.adv_steps.append(step)
                    self.cool_until = t_now + cfg.cooldown_s
                    self.hist.clear()
                    return None, j

        # --- arm C only: replan, when no candidate was feasible ------------
        if self.replan_enabled and self.planner is not None and self.n_plan < cfg.max_plan:
            new = self.planner.plan(x, self.P.goal)
            self.n_plan += 1
            self.plan_steps.append(step)
            self.cool_until = t_now + cfg.cooldown_s
            self.hist.clear()
            if new is None or new.shape[1] < 2:
                self.n_plan_failed += 1
                return None, None                 # keep existing path
            from planner import prune_collinear
            self.wp = prune_collinear(new)
            return self.wp, 0                     # active index reset to 0
        return None, None
