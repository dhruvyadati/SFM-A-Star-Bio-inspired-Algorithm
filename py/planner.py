"""A* on the pruned 8-connected grid: Python port of GraphFast.m / AStarFast.m.

Amendment G arm C replans mid-run, which requires the planner inside the Python
engine. Until now waypoints have always arrived pre-computed in the scenario
JSON, so no Python planner existed.

WHY AN EXACT PORT MATTERS. If arm C replanned with a different implementation
from the one that produced arms A and B's initial paths, any B--C difference
could be the port rather than replanning. `Test_PlannerParity` therefore
regenerates waypoints for archived scenarios and requires an EXACT match
against the MATLAB-produced paths already in those files. A port that merely
"looks right" would silently confound the experiment.

Details preserved deliberately, because each changes the resulting path:

  * node pruning uses `dmin <= margin`, not `<`;
  * the Lipschitz skip fires when `dmin(u) <= L or dmin(v) <= L`, matching the
    MATLAB condition exactly;
  * open-set selection breaks ties by LOWEST NODE INDEX among equal f-scores,
    which MATLAB obtains from `min` over `find(openSet)`;
  * nodes are indexed column-major (MATLAB `reshape`), so index arithmetic
    follows Fortran order rather than C order.
"""
from __future__ import annotations

import heapq

import numpy as np

SQRT2 = float(np.sqrt(2.0))
DIRS = [(-1, 0, 1.0), (-1, 1, SQRT2), (0, 1, 1.0), (1, 1, SQRT2),
        (1, 0, 1.0), (1, -1, SQRT2), (0, -1, 1.0), (-1, -1, SQRT2)]


def _poly_dist_grid(xs, ys, verts):
    """Unsigned distance from every grid point to one polygon boundary.

    Mirrors p_poly_dist's magnitude. Sign is irrelevant here: the caller
    compares against a positive margin, and interior points are blocked either
    way.
    """
    vx, vy = verts[:, 0], verts[:, 1]
    ax, ay = vx[:, None], vy[:, None]
    bx, by = np.roll(vx, -1)[:, None], np.roll(vy, -1)[:, None]
    ex, ey = bx - ax, by - ay
    seg2 = np.where(ex ** 2 + ey ** 2 < 1e-18, 1.0, ex ** 2 + ey ** 2)
    px, py = xs.ravel()[None, :], ys.ravel()[None, :]
    t = np.clip(((px - ax) * ex + (py - ay) * ey) / seg2, 0.0, 1.0)
    d = np.hypot(px - (ax + t * ex), py - (ay + t * ey))
    return d.min(axis=0).reshape(xs.shape)


def _seg_hits_poly(x1, y1, x2, y2, verts):
    """Does segment (1)->(2) intersect the polygon boundary? polyxpoly analogue."""
    vx, vy = verts[:, 0], verts[:, 1]
    x3, y3 = vx, vy
    x4, y4 = np.roll(vx, -1), np.roll(vy, -1)
    d = (x2 - x1) * (y4 - y3) - (y2 - y1) * (x4 - x3)
    with np.errstate(divide="ignore", invalid="ignore"):
        ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / d
        ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / d
    ok = (np.abs(d) > 1e-15) & (ua >= 0) & (ua <= 1) & (ub >= 0) & (ub <= 1)
    return bool(np.any(ok))


class GridPlanner:
    """Pruned 8-connected grid with A*. Build once per map, plan many times."""

    def __init__(self, x_max, y_max, obstacles, margin):
        self.x_max, self.y_max = int(x_max), int(y_max)
        self.obstacles = [np.asarray(o, float) for o in obstacles]
        self.margin = float(margin)

        xs, ys = np.meshgrid(np.arange(1, self.x_max + 1),
                             np.arange(1, self.y_max + 1))
        dmin = np.full((self.y_max, self.x_max), np.inf)
        for verts in self.obstacles:
            dmin = np.minimum(dmin, _poly_dist_grid(xs, ys, verts))
        self.dmin = dmin
        # `<=`, matching MATLAB. Using `<` would admit a ring of nodes exactly
        # at the margin and change every path that runs near an obstacle.
        self.blocked = dmin <= self.margin

        self.adj = self._build_edges()

    def _build_edges(self):
        adj = {}
        blocked, dmin = self.blocked, self.dmin
        for y in range(1, self.y_max + 1):
            for x in range(1, self.x_max + 1):
                if blocked[y - 1, x - 1]:
                    continue
                u = self._nid(x, y)
                for dy, dx, L in DIRS:
                    nx, ny = x + dx, y + dy
                    if not (1 <= nx <= self.x_max and 1 <= ny <= self.y_max):
                        continue
                    if blocked[ny - 1, nx - 1]:
                        continue
                    v = self._nid(nx, ny)
                    if u >= v:                      # dedupe the undirected pair
                        continue
                    if dmin[y - 1, x - 1] <= L or dmin[ny - 1, nx - 1] <= L:
                        if any(_seg_hits_poly(x, y, nx, ny, o)
                               for o in self.obstacles):
                            continue
                    adj.setdefault(u, []).append((v, L))
                    adj.setdefault(v, []).append((u, L))
        return adj

    def _nid(self, x, y):
        """Column-major node id, matching MATLAB reshape(1:xMax*yMax, yMax, xMax)."""
        return (x - 1) * self.y_max + y

    def _xy(self, nid):
        q, r = divmod(nid - 1, self.y_max)
        return q + 1, r + 1

    def plan(self, start_xy, goal_xy):
        """A* from start to goal, both 1-based (x, y). Returns (2, n) or None."""
        sx, sy = int(round(start_xy[0])), int(round(start_xy[1]))
        gx, gy = int(round(goal_xy[0])), int(round(goal_xy[1]))
        if self.blocked[sy - 1, sx - 1] or self.blocked[gy - 1, gx - 1]:
            return None
        start, goal = self._nid(sx, sy), self._nid(gx, gy)

        g = {start: 0.0}
        came = {}
        closed = set()
        # Tie-break on node index reproduces MATLAB's min-over-find(openSet).
        pq = [(float(np.hypot(sx - gx, sy - gy)), start, start)]
        while pq:
            f, _, cur = heapq.heappop(pq)
            if cur in closed:
                continue
            if cur == goal:
                nodes, n = [cur], cur
                while n != start:
                    n = came[n]
                    nodes.append(n)
                nodes.reverse()
                return np.array([self._xy(n) for n in nodes], float).T
            closed.add(cur)
            for v, w in self.adj.get(cur, ()):
                if v in closed:
                    continue
                t = g[cur] + w
                if t < g.get(v, np.inf):
                    g[v] = t
                    came[v] = cur
                    vx, vy = self._xy(v)
                    heapq.heappush(pq, (t + float(np.hypot(vx - gx, vy - gy)), v, v))
        return None


def prune_collinear(path):
    """Drop interior points that lie on the segment joining their neighbours."""
    if path is None or path.shape[1] < 3:
        return path
    keep = [0]
    for i in range(1, path.shape[1] - 1):
        a, b, c = path[:, keep[-1]], path[:, i], path[:, i + 1]
        if abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) > 1e-9:
            keep.append(i)
    keep.append(path.shape[1] - 1)
    return path[:, keep]
