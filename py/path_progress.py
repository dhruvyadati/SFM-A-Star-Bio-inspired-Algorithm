"""Arc-length progress along a FIXED A* polyline.

Amendment F measures progress as reduction in remaining arc length:

    delta_s = s_remaining(t - W) - s_remaining(t)

Straight-line distance to the current target is NOT a substitute. Rounding an
obstacle, the robot makes genuine arc-length progress while straight-line
distance to the next waypoint stagnates or grows -- so a straight-line detector
would fire recovery precisely in the cluttered geometry the controller is
supposed to handle carefully.

TWO FAILURE MODES, PULLED APART.

The naive fix for one causes the other:

  * SELF-INTERSECTION CAPTURE. Nearest-segment projection on a route that
    doubles back can match a far-away segment that happens to be spatially
    close, so `s_remaining` jumps discontinuously.
  * LOST BACKWARD MOTION. Forbidding earlier segments outright stops the jump,
    but then a robot pushed back across a passed waypoint has its projection
    clamped at that waypoint and `s_remaining` SATURATES. Real backward travel
    is silently discarded, which is exactly the state the recovery trigger
    needs to see.

A monotone index solves the first and creates the second. This implementation
instead bounds candidates by PLAUSIBILITY: a projection is admissible only if
the implied change in arc coordinate is consistent with how far the robot
actually moved this step. Genuine backward travel is admitted, because the
displacement supports it; a self-intersection jump is rejected, because it
would require a large arc change from a small displacement.

Consequences, all deliberate:

  * passing a waypoint is continuous -- adjacent segments share that vertex;
  * moving back across a passed waypoint increases `s_remaining` by the full
    backward displacement, so progress goes NEGATIVE and is not clamped;
  * a self-approaching path cannot capture the projection.

F contains no replanning, so the polyline is fixed for the whole trial. If
PROTOCOL 5a is ever implemented, progress must reset onto the new path under a
separately defined rule; that complexity is deliberately absent here.
"""
from __future__ import annotations

import numpy as np


class PathProgress:
    """Remaining arc length to the end of a fixed polyline.

    `waypoints` is (2, n). Call `remaining(x)` once per step with the robot
    position.
    """

    def __init__(self, waypoints: np.ndarray, lookahead: int = 3,
                 lookback: int = 3, jump_tol: float = 0.05):
        self.W = np.asarray(waypoints, float)
        if self.W.ndim != 2 or self.W.shape[0] != 2 or self.W.shape[1] < 2:
            raise ValueError("waypoints must be (2, n>=2)")
        d = np.hypot(np.diff(self.W[0]), np.diff(self.W[1]))
        self.seg_len = d
        self.cum = np.concatenate([[0.0], np.cumsum(d)])
        self.total = float(self.cum[-1])
        self.n_seg = len(d)
        self.seg = 0                  # search hint, NOT a hard floor
        self.lookahead = lookahead
        self.lookback = lookback
        # Slack on the plausibility bound, absorbing projection geometry and
        # the fact that arc distance can slightly exceed chord displacement.
        self.jump_tol = float(jump_tol)
        self.last_travelled = None
        self.last_x = None

    def _project(self, x, i):
        """Projection of x onto segment i. Returns (t in [0,1], distance)."""
        a = self.W[:, i]
        b = self.W[:, i + 1]
        e = b - a
        L2 = float(e[0] ** 2 + e[1] ** 2)
        if L2 < 1e-18:
            return 0.0, float(np.hypot(*(x - a)))
        t = float(np.clip(((x[0] - a[0]) * e[0] + (x[1] - a[1]) * e[1]) / L2, 0.0, 1.0))
        c = a + t * e
        return t, float(np.hypot(*(x - c)))

    def remaining(self, x) -> float:
        """Remaining arc length from the robot to the end of the polyline.

        ASSUMES PER-SIMULATION-STEP CALLS. The plausibility bound is
        displacement-based, so at dt = 0.01 s a step is ~0.013 m and the bound
        is tight. Called with a very coarse sampling it may prefer a
        nearer-in-arc projection over the geometrically closest one, because a
        genuine leap and a projection jump become indistinguishable. See
        test_assumes_simulation_rate_sampling.
        """
        x = np.asarray(x, float)
        lo = max(0, self.seg - self.lookback)
        hi = min(self.n_seg, self.seg + self.lookahead + 1)

        cands = []
        for i in range(lo, hi):
            t, dist = self._project(x, i)
            travelled = self.cum[i] + t * self.seg_len[i]
            cands.append((dist, i, travelled))
        cands.sort(key=lambda c: c[0])

        if self.last_travelled is None:
            dist, i, travelled = cands[0]
        else:
            # How far the robot could plausibly have moved along the route this
            # step. Backward motion is allowed; teleporting along the arc is
            # not. This is what separates real reversal from a projection jump.
            disp = float(np.hypot(*(x - self.last_x)))
            budget = disp + self.jump_tol
            ok = [c for c in cands if abs(c[2] - self.last_travelled) <= budget]
            dist, i, travelled = (ok[0] if ok else cands[0])

        self.seg = i
        self.last_travelled = travelled
        self.last_x = x.copy()
        return float(self.total - travelled)
