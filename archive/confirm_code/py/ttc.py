"""Time-to-collision speed filter, Python port of v2/TtcBrake.m.

Independently written from the same specification and cross-validated against
the MATLAB implementation, on the same basis as the SFM engine.

Two mechanisms, both required:

  PREDICTIVE. For each pedestrian, time of closest approach under constant
  velocity, t* = max(0, -(r.u)/|u|^2), and the predicted separation there.
  Braking engages when a pedestrian is both approaching within the horizon and
  predicted to breach the combined radii plus a clearance margin.

  SEPARATION FLOOR. Time-to-collision alone is not sufficient: as relative
  speed falls, t* diverges and the horizon test stops firing, so the filter
  disengages exactly when the robot is creeping. A floor on CURRENT separation
  covers that blind spot. It is gated on approach direction, and the gate ramps
  over [-recedeBand, 0] of closing speed rather than over positive closing
  speed -- ramping the latter way reintroduces the same low-speed hole.

All ramps are smoothstep (C1 at both ends) and the output is slew-rate
limited, because an earlier version using binary gates and a raw min produced
an 11x increase in RMS jerk from activation chatter.
"""
from __future__ import annotations

import numpy as np


def smoothramp(x, lo, hi):
    """0 at or below lo, 1 at or above hi, smoothstep between. C1 at both ends.

    lo/hi may be arrays: the proximity threshold is per-pedestrian, so the
    span must be guarded with np.maximum rather than the builtin max, which
    raises on arrays. The canonical fixtures carry 0 or 1 pedestrian and never
    exercised that path.
    """
    span = np.maximum(np.asarray(hi) - np.asarray(lo), 1e-12)
    t = np.clip((x - lo) / span, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def ttc_brake(p_rob, v_rob, p_ped, v_ped, radii, P, prev_scale=1.0):
    """Return (scale in [0,1], active, t_min)."""
    if p_ped.shape[1] == 0:
        return 1.0, False, np.inf

    r = p_ped - p_rob[:, None]                 # robot -> pedestrian
    u = v_ped - v_rob[:, None]                 # relative velocity
    uu = np.sum(u * u, axis=0)
    ru = np.sum(r * u, axis=0)
    t_star = np.maximum(0.0, -ru / (uu + 1e-12))

    closest = r + t_star * u
    d_star = np.hypot(closest[0], closest[1])
    threshold = P.rRobot + radii + P.ttcClearance

    scale = 1.0
    active = False
    t_min = np.inf

    # --- predictive term ---------------------------------------------
    urgency = smoothramp(t_star, P.ttcStop, P.ttcHorizon)
    band = max(P.ttcClearance, 1e-6)
    # Proximity enters as a smooth WEIGHT, not a gate, so a pedestrian coming
    # into consideration ramps in from no effect.
    prox = 1.0 - smoothramp(d_star, threshold - band, threshold)
    s_pred = 1.0 - prox * (1.0 - urgency)

    # --- current-separation floor -------------------------------------
    dist = np.hypot(r[0], r[1])
    gap_now = dist - (P.rRobot + radii)
    closing = -ru / np.maximum(dist, 1e-12)
    approach = smoothramp(closing, -P.ttcRecedeBand, 0.0)
    s_floor = 1.0 - approach * (1.0 - smoothramp(gap_now, P.ttcHardGap, P.ttcMinGap))

    raw = float(np.clip(min(s_pred.min(), s_floor.min()), 0.0, 1.0))
    active = raw < 0.99
    if active:
        cand = t_star[s_pred < 0.99]
        if cand.size:
            t_min = float(cand.min())

    # --- slew-rate limit ----------------------------------------------
    max_step = P.ttcSlewRate * P.dt
    scale = prev_scale + float(np.clip(raw - prev_scale, -max_step, max_step))
    return float(np.clip(scale, 0.0, 1.0)), active, t_min
