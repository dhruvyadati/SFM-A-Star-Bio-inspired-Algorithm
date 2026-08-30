function [F, diag] = ComputeForces(x, v, goals, radii, v0, xObsList, yObsList, P)
% ComputeForces evaluates the corrected Social Force Model for every agent.
%
% Inputs
%   x        2 x N  positions (m)
%   v        2 x N  velocities (m/s)
%   goals    2 x N  current goal / subgoal per agent (m)
%   radii    1 x N  body radius per agent (m)
%   v0       1 x N  preferred speed per agent (m/s)
%   xObsList,yObsList  cell arrays of polygon vertex coordinates
%   P        parameter struct from SfmParams
%
% Outputs
%   F        2 x N  net force (acceleration, m/s^2 -- unit mass, see below)
%   diag     struct of per-step diagnostics used by the safety metrics:
%              .minPedGap(i)  smallest surface-to-surface gap to another agent
%              .minObsGap(i)  smallest surface-to-boundary gap to any obstacle
%
% MASS AND UNITS. Every agent is modelled with unit mass, so the quantities
% returned here are accelerations in m/s^2 and F = m*a reduces to F = a. This
% is stated explicitly because the legacy code combined terms
% described as forces and accelerations without specifying a mass. The
% goal term (v_des - v)/tau has units (m/s)/s = m/s^2, and both repulsion
% terms carry strengths declared in m/s^2, so all three are dimensionally
% consistent and may be summed directly.

N = size(x, 2);

%% ---- 1. Goal attraction ---------------------------------------------
% F_G = (v_des * e_hat - v) / tau
%
% e_hat is a UNIT vector. the legacy code used the raw displacement (x_g - x), which made
% the attraction magnitude grow with the square of the distance to the
% subgoal and carried inconsistent units. The desired speed tapers linearly
% to zero inside dSlow so that an agent can actually come to rest at its
% goal instead of orbiting it.
dGoal = goals - x;
distGoal = sqrt(sum(dGoal.^2, 1));
ehat = zeros(2, N);
nz = distGoal > eps;
ehat(:, nz) = dGoal(:, nz) ./ distGoal(nz);

vDes = v0 .* min(1, distGoal ./ P.dSlow);
F = (ehat .* vDes - v) ./ P.tau;

%% ---- 2. Agent repulsion (vectorised) --------------------------------
% F_D = A * exp((r_i + r_j - d_ij) / B) * n_hat_ij
%
% Monotonically DECREASING in d and maximal at contact. the earlier d*exp(-d) form
% peaked at the edge of the influence radius and went to zero at contact,
% which is the opposite of the intended barrier behaviour.
dx = x(1, :).' - x(1, :);          % N x N, entry (i,j) = x_i - x_j
dy = x(2, :).' - x(2, :);
d  = sqrt(dx.^2 + dy.^2);
rSum = radii.' + radii;            % N x N

self = logical(eye(N));
gapMat = d - rSum;
gapMat(self) = inf;
diag.minPedGap = min(gapMat, [], 2).';

% Per-partner gaps for agent 1 (the robot). Contact episodes must be tracked
% by pedestrian IDENTITY: a min-over-all-partners signal merges consecutive
% contacts with different people into one episode and cannot report how many
% distinct pedestrians were touched.
diag.robotGaps = gapMat(1, :);

act = ~self & (d <= P.rInfPed) & (d > eps);
if any(act(:))
    dSafe = d;
    dSafe(~act) = 1;               % avoid divide-by-zero on masked entries

    % Row 1 is the force felt BY the robot, so only that row carries the
    % robot's own repulsion gain. Pedestrian-pedestrian rows keep the nominal
    % value, which is what preserves an identical crowd across arms.
    aMat = P.aPed * ones(N);
    if ~isempty(P.aPedRobot), aMat(1, :) = P.aPedRobot; end

    mag = aMat .* exp((rSum - d) ./ P.bPed);
    overlap = max(0, rSum - d);
    mag = mag + P.kBody * overlap;     % contact term stays symmetric
    mag(~act) = 0;

    ux = dx ./ dSafe;
    uy = dy ./ dSafe;
    F(1, :) = F(1, :) + sum(mag .* ux, 2).';
    F(2, :) = F(2, :) + sum(mag .* uy, 2).';
end

%% ---- 3. Static obstacle repulsion -----------------------------------
% F_O = A_s * exp((r_i - d) / B_s) * n_hat
%
% p_poly_dist returns a SIGNED distance: negative when the point lies inside
% the polygon. the legacy code fed that signed value straight into the cutoff test and the
% direction vector, so a penetrating agent was pushed further inward. Here the
% interior case is detected explicitly and the agent is always driven outward
% along the boundary normal.
%
% A bounding-box prefilter skips the expensive point-to-polygon distance for
% agents that cannot possibly be within the cutoff.
nObs = numel(xObsList);
diag.minObsGap = inf(1, N);
maxR = max(radii);

for k = 1:nObs
    xv = xObsList{k}; yv = yObsList{k};
    pad = P.rInfObs + maxR;
    inBox = x(1, :) >= min(xv) - pad & x(1, :) <= max(xv) + pad & ...
            x(2, :) >= min(yv) - pad & x(2, :) <= max(yv) + pad;
    idx = find(inBox);

    for ii = idx
        [dSigned, xq, yq] = p_poly_dist(x(1, ii), x(2, ii), xv, yv);
        toSurface = [xq; yq] - x(:, ii);
        ns = norm(toSurface);

        if dSigned < 0
            % Inside the polygon: push outward toward the nearest boundary
            % point. the legacy code pushed such an agent deeper in.
            if ns > eps, nhat = toSurface / ns; else, nhat = [1; 0]; end
            F(:, ii) = F(:, ii) + P.aObs * exp(radii(ii) / P.bObs) * nhat ...
                                + P.kBody * (radii(ii) + ns) * nhat;
            diag.minObsGap(ii) = min(diag.minObsGap(ii), dSigned - radii(ii));
            continue;
        end

        gap = dSigned - radii(ii);
        if gap < diag.minObsGap(ii), diag.minObsGap(ii) = gap; end
        if dSigned > P.rInfObs, continue; end

        if ns > eps, nhat = -toSurface / ns; else, nhat = [1; 0]; end
        F(:, ii) = F(:, ii) + P.aObs * exp((radii(ii) - dSigned) / P.bObs) * nhat;

        if dSigned < radii(ii)
            F(:, ii) = F(:, ii) + P.kBody * (radii(ii) - dSigned) * nhat;
        end
    end
end
end
