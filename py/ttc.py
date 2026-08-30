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

from dataclasses import dataclass

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


@dataclass(frozen=True)
class TTCBrakeResult:
    """The TTC layer's own decision state — the single source of truth.

    `hard_gap_active` and `urgent_ttc_active` mark interventions the filter
    considers MANDATORY. A recovery layer may relax only a DISCRETIONARY
    intervention, and must take these flags from here rather than recomputing
    an emergency condition at the integration boundary. Recomputing invites
    drift: surface gap versus centre distance, differing threshold equality,
    or a post-update rather than pre-command state, any of which could let
    recovery release a brake this layer deemed compulsory.

    Both flags are computed from the SAME pre-command geometry as `scale`.
    """
    scale: float
    active: bool
    min_ttc: float
    hard_gap_active: bool
    urgent_ttc_active: bool
    # Pre-slew command and whether the rate limiter actually clipped it. Needed
    # to MEASURE limiter activity at a coarse control interval rather than
    # infer it from the maximum possible change.
    raw_scale: float = 1.0
    slew_limited: bool = False


def ttc_brake(p_rob, v_rob, p_ped, v_ped, radii, P, prev_scale=1.0):
    """Return (scale in [0,1], active, t_min).

    Thin view over `ttc_brake_ex`, kept so the frozen confirmatory engine's
    call signature and behaviour are untouched.
    """
    r = ttc_brake_ex(p_rob, v_rob, p_ped, v_ped, radii, P, prev_scale)
    return r.scale, r.active, r.min_ttc


def ttc_brake_ex(p_rob, v_rob, p_ped, v_ped, radii, P, prev_scale=1.0):
    """Authoritative implementation. Returns TTCBrakeResult."""
    if p_ped.shape[1] == 0:
        return TTCBrakeResult(1.0, False, np.inf, False, False, 1.0, False)

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

    # --- mandatory-intervention flags, from the same pre-command state ----
    # Hard gap: separation at or inside the floor the controller must never
    # trade away. Urgent: predicted contact within the full-stop horizon while
    # the pedestrian is inside the proximity threshold.
    # Defined against the SAME weights that produce the command, not against
    # raw geometry. A purely geometric test fires for a RECEDING pedestrian at
    # close range, where `approach` is 0 and the filter correctly commands
    # nothing -- so the flag would not mean "a mandatory intervention is being
    # made", which is precisely what recovery needs it to mean.
    hard_gap_active = bool(np.any((gap_now <= P.ttcHardGap) & (approach > 0.0)))
    urgent_ttc_active = bool(np.any((t_star <= P.ttcStop) & (prox > 0.0)))

    return TTCBrakeResult(float(np.clip(scale, 0.0, 1.0)), active, t_min,
                          hard_gap_active, urgent_ttc_active,
                          float(raw), bool(abs(raw - prev_scale) > max_step + 1e-12))
