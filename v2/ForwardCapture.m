function C = ForwardCapture(S, P, x0, v0, rolloutT)
% ForwardCapture rolls the static local dynamics forward from (x0, v0) and
% returns INTERPRETABLE RAW FEATURES describing capture susceptibility.
%
%   S         scenario (obstacles, goal)
%   P         parameter struct
%   x0, v0    2x1 current position and velocity -- both known to the robot
%   rolloutT  rollout length, seconds
%
% WHY A ROLLOUT, NOT AN EIGENVALUE TEST. Local stability is defined around an
% EQUILIBRIUM, where the state derivative vanishes. A robot travelling at ~v0
% is not at one, since xdot = v =/= 0, so a Jacobian evaluated there says
% nothing about attraction and calling such a state "stable" is a category
% error.
%
% WHY VELOCITY MATTERS AT ALL. The velocity trace itself relaxes with
% tau = 0.5 s, so it is nearly forgotten within a second. But during that
% transient the robot is DISPLACED by of order 1 m, and basin membership is
% positional. Velocity therefore influences the outcome indirectly and
% durably: a sweep over 36 states found median endpoint spread of 1.9 m from
% velocity choice alone, reaching 5.7 m at a 5 s horizon.
%
% WHY RAW FEATURES, NOT ONE COMPOSITE. An earlier single composite conflated
% "no progress because trapped" with "no progress because little time
% elapsed", which made it useless below ~2 s, and separately scored a robot
% decelerating to ARRIVE as maximally trapped. Emitting features separately
% keeps the behaviour of one handcrafted scalar from being mistaken for the
% behaviour of the diagnostic.
%
% Pedestrians are excluded from the rollout, so the reading is "absent further
% pedestrian interference, is this state captured?" Only query-time
% information is used -- map, goal, controller constants, current state -- so
% this remains prospective.
%
% Features returned:
%   .minSpeed, .finalSpeed          m/s
%   .fracSlow                       fraction of rollout below stallSpeed
%   .progressAbs                    goal-distance reduction, m
%   .progressRatio                  progressAbs / (vDes0 * T), dimensionless
%   .pathLength                     distance travelled, m
%   .displacement                   |x(T) - x(0)|, m
%   .tailSpeed                      mean speed over the trailing window
%   .tailProgressRate               mean goal-progress rate over the tail, m/s
%   .speedFalling                   true if speed is decreasing at the end
%   .distToEquilib                  endpoint distance to nearest attracting
%                                   candidate of the static field, m
%   .fracNearEquilib                fraction of rollout within 1 m of one
%   .endGoalDist, .arrived
%
% No composite is returned. Scores are assembled and frozen by the caller on
% development maps only.

nSteps = max(1, round(rolloutT / P.dt));
x = x0(:); v = v0(:);

d0 = norm(x - P.goal(:));
vDes0 = P.v0Robot * min(1, d0 / P.dSlow);

% Trailing window: the last 0.25 s, or a quarter of the rollout if shorter.
tailN = max(1, min(nSteps, round(min(0.25, rolloutT/4) / P.dt)));

F0 = MapField(S, P, 0.5);
hasEq = any(F0.stable(:));
if hasEq
    eqX = F0.X(F0.stable); eqY = F0.Y(F0.stable);
end

speeds = zeros(1, nSteps);
goalD  = zeros(1, nSteps);
nearEq = false(1, nSteps);
pathLen = 0;

for k = 1:nSteps
    dg = P.goal(:) - x;
    dist = norm(dg);
    if dist > eps
        vDes = P.v0Robot * min(1, dist / P.dSlow);
        F = (dg / dist * vDes - v) / P.tau;
    else
        F = -v / P.tau;
    end

    for kk = 1:numel(S.xObsList)
        [dS, xq, yq] = p_poly_dist(x(1), x(2), S.xObsList{kk}, S.yObsList{kk});
        if dS > P.rInfObs, continue; end
        toS = [xq; yq] - x;
        ns = norm(toS);
        if ns < eps, continue; end
        if dS < 0
            nhat = toS / ns;
            F = F + P.aObs * exp(P.rRobot / P.bObs) * nhat ...
                  + P.kBody * (P.rRobot + ns) * nhat;
        else
            nhat = -toS / ns;
            F = F + P.aObs * exp((P.rRobot - dS) / P.bObs) * nhat;
            if dS < P.rRobot
                F = F + P.kBody * (P.rRobot - dS) * nhat;
            end
        end
    end

    v = v + F * P.dt;
    sp = norm(v);
    if sp > P.vMax, v = v / sp * P.vMax; sp = P.vMax; end
    xPrev = x;
    x = x + v * P.dt;

    pathLen = pathLen + norm(x - xPrev);
    speeds(k) = sp;
    goalD(k)  = norm(x - P.goal(:));
    if hasEq
        nearEq(k) = min(hypot(eqX - x(1), eqY - x(2))) <= 1.0;
    end
end

C.minSpeed    = min(speeds);
C.finalSpeed  = speeds(end);
C.fracSlow    = mean(speeds < P.stallSpeed);
C.progressAbs = d0 - goalD(end);
% Dimensionless progress: 1 is nominal, 0 is none, negative is retreat. Uses
% the desired speed AT THE QUERY STATE, so the scale is horizon-appropriate.
C.progressRatio = C.progressAbs / max(vDes0 * rolloutT, eps);
C.pathLength  = pathLen;
C.displacement = norm(x - x0(:));
C.tailSpeed   = mean(speeds(end-tailN+1:end));
if nSteps > tailN
    C.tailProgressRate = (goalD(end-tailN) - goalD(end)) / (tailN * P.dt);
else
    C.tailProgressRate = (d0 - goalD(end)) / (nSteps * P.dt);
end
% Trend over the trailing window rather than a two-sample difference.
if tailN >= 2
    tt = (1:tailN).';
    pf = polyfit(tt, speeds(end-tailN+1:end).', 1);
    C.speedFalling = pf(1) < 0;
    C.speedSlope   = pf(1) / P.dt;      % m/s per second
else
    C.speedFalling = false; C.speedSlope = 0;
end
if hasEq
    C.distToEquilib = min(hypot(eqX - x(1), eqY - x(2)));
else
    C.distToEquilib = inf;
end
C.fracNearEquilib = mean(nearEq);
C.endPos = x;
C.endGoalDist = goalD(end);
% Inside the deceleration zone the robot is arriving, not trapped. Callers
% must exclude these rather than read low speed as capture.
C.arrived = goalD(end) <= max(P.rGoal, P.dSlow);
end
