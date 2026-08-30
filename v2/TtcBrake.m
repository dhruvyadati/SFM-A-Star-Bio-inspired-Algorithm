function [scale, active, tMin] = TtcBrake(pRob, vRob, pPed, vPed, radii, P, prevScale)
% TtcBrake returns a speed-scale factor in [0,1] that slows the robot when a
% collision is predicted, plus whether the filter engaged and the minimum
% predicted time to closest approach.
%
%   pRob      2x1 robot position        vRob  2x1 robot velocity
%   pPed      2xM pedestrian positions  vPed  2xM pedestrian velocities
%   radii     1xM pedestrian radii
%   P         parameter struct
%   prevScale previous step's scale, for rate limiting (default 1)
%
% This is a FILTER, not a planner. It never chooses a direction; it only
% scales commanded speed. That keeps it a separated, independently ablatable
% layer rather than a second navigation algorithm entangled with the first.
%
% CONTINUITY. An earlier version gated braking on the binary predicates
%   (t* < horizon) AND (d* < threshold)
% and took a raw min of two linear ramps. Whenever d* crossed the threshold
% while t* was already small, the scale jumped from 1 to near 0 in a single
% step. Measured cost was an 11x increase in RMS jerk (12.9 -> 141.6) with
% ~17 activations per trial averaging 0.21 s each, i.e. chatter.
%
% Three changes remove that:
%   1. Proximity enters as a smooth WEIGHT, not a gate, so a pedestrian
%      entering consideration ramps in from no effect.
%   2. All ramps are smoothstep, t^2(3-2t), which is C1 at both ends, so the
%      command has no corner where a linear ramp would.
%   3. The output is slew-rate limited, which bounds the acceleration the
%      filter itself can inject regardless of how fast the geometry changes.
%
% Time-to-collision answers "will current relative motion produce a close
% approach soon?" It does not answer "are we already too close, or drifting
% in slowly?" -- as relative speed falls, t* diverges and the horizon test
% stops firing. The separation floor below covers that blind spot.

if nargin < 7 || isempty(prevScale), prevScale = 1.0; end

scale  = 1.0;
active = false;
tMin   = inf;

M = size(pPed, 2);
if M == 0, return; end

r = pPed - pRob;                       % 2xM, robot -> pedestrian
u = vPed - vRob;                       % 2xM, relative velocity

uu = sum(u.^2, 1);
ru = sum(r .* u, 1);
tStar = max(0, -ru ./ (uu + eps));

closest = r + tStar .* u;
dStar = sqrt(sum(closest.^2, 1));
threshold = P.rRobot + radii + P.ttcClearance;

%% ---- Predictive term -------------------------------------------------
% Urgency ramps smoothly from 1 (at or beyond the horizon) to 0 (at or
% inside the stopping horizon).
urgency = smoothramp(tStar, P.ttcStop, P.ttcHorizon);

% Proximity weight: 1 when the predicted approach is well inside the
% threshold, falling smoothly to 0 as it reaches the threshold. Entering
% consideration therefore costs nothing at the boundary.
band = max(P.ttcClearance, 1e-6);
proxW = 1 - smoothramp(dStar, threshold - band, threshold);

sPred = 1 - proxW .* (1 - urgency);

%% ---- Current-separation floor ----------------------------------------
gapNow = sqrt(sum(r.^2, 1)) - (P.rRobot + radii);
closingSpeed = -ru ./ max(sqrt(sum(r.^2, 1)), eps);   % >0 when closing
approachW = smoothramp(closingSpeed, -P.ttcRecedeBand, 0);
% The band separates RECEDING from SLOWLY APPROACHING, not fast from slow.
% Ramping up over positive closing speed would release the brake exactly when
% the robot creeps forward at low speed -- the same blind spot the separation
% floor exists to cover. Weight is therefore 1 for any non-negative closing
% speed and falls to 0 only once the robot is clearly moving away, which is
% all that is needed to avoid freezing a robot being pushed clear by the
% repulsive force.

sFloor = 1 - approachW .* (1 - smoothramp(gapNow, P.ttcHardGap, P.ttcMinGap));

%% ---- Combine ---------------------------------------------------------
raw = min([sPred, sFloor]);
raw = min(1, max(0, raw));

% Engagement is reported on a meaningful deviation from free speed, so
% negligible ramp values do not inflate the duty-cycle statistic.
active = raw < 0.99;
if active
    cand = tStar(sPred < 0.99);
    if ~isempty(cand), tMin = min(cand); end
end

%% ---- Slew-rate limit -------------------------------------------------
% Bounds the acceleration the filter can inject. Without it, the brake can
% swing across its whole range within one timestep whenever the geometry
% changes quickly, which is what produced the jerk blow-up.
maxStep = P.ttcSlewRate * P.dt;
scale = prevScale + max(-maxStep, min(maxStep, raw - prevScale));
scale = min(1, max(0, scale));
end

% ---------------------------------------------------------------------
function y = smoothramp(x, lo, hi)
% 0 at or below lo, 1 at or above hi, smoothstep in between. C1 at both ends,
% so the command has no corner where a linear ramp would have one.
    t = (x - lo) ./ max(hi - lo, eps);
    t = min(1, max(0, t));
    y = t.^2 .* (3 - 2*t);
end
