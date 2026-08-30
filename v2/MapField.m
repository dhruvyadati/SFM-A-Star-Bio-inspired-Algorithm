function F = MapField(S, P, h, forceRecompute)
% MapField returns all STATIC per-map field quantities, memoised by seed and
% grid step. Everything here depends only on the map, the goal, and the
% controller constants -- never on a trajectory, an outcome, or a pedestrian
% snapshot. That property is what makes it usable as a prospective predictor,
% and it is asserted in Test_NoOutcomeLeakage.
%
%   F.X, F.Y        grid
%   F.Fx, F.Fy      static net-force components (goal + obstacles, probe at rest)
%   F.Z             magnitude. Sample it by interpolating COMPONENTS then
%                   taking the norm -- see the note below.
%   F.clearance     signed distance to the nearest obstacle boundary
%   F.gapToRobot    clearance minus robot radius
%   F.gradMag       gradient magnitude of Z, units s^-2
%   F.stable        locally attracting equilibrium candidates, 4x4 Jacobian
%   F.inObs         inside-obstacle mask
%   F.free          usable free-space mask
%   F.seed, F.h     provenance
%
% Memoisation matters: the obstacle term costs ~388k point-to-polygon
% evaluations per map at h = 0.25, and it was previously computed twice per
% case -- once inside the force field and again for the clearance predictor.

persistent CACHE
if isempty(CACHE), CACHE = containers.Map('KeyType','char','ValueType','any'); end
if nargin < 4, forceRecompute = false; end

key = sprintf('%d_%.6f', S.seed, h);
if ~forceRecompute && isKey(CACHE, key)
    F = CACHE(key);
    return;
end

xs = P.xMin:h:P.xMax;
ys = P.yMin:h:P.yMax;
[X, Y] = meshgrid(xs, ys);
[ny, nx] = size(X);
gx = X(:).'; gy = Y(:).';
M = numel(gx);

rRob = P.rRobot;

%% ---- Goal attraction, probe at rest ----------------------------------
% VELOCITY ASSUMPTION -- read before using F.stable as a predictor.
%
% The goal term is evaluated with v = 0: the -v/tau contribution is absent.
% Everything here is therefore F(x, v = 0), and F.stable answers
%
%     "is this position an attracting equilibrium FOR A ROBOT AT REST?"
%
% That is susceptibility to static trapping, NOT whether a moving robot will
% be captured. At a given position one velocity may be drawn into the basin,
% another may pass straight through, and a third may escape.
%
% The consequence is measurable. Scoring the terminal position of a stalled
% robot works well (the robot really is at rest, so the assumption holds),
% while scoring a state five seconds earlier -- when the robot is still moving
% at roughly v0 -- performs at chance. The field is being asked a question it
% does not encode.
%
% The fix is NOT to evaluate this Jacobian at a moving state. Local stability
% is defined around an EQUILIBRIUM, where the state derivative vanishes; at
% v =/= 0 the state is not an equilibrium and eigenvalues there say nothing
% about attraction. Momentum has to be represented by integrating the dynamics
% forward, which is what ForwardCapture does.
%
% TERMINOLOGY. F.stable means "attracting-equilibrium candidate of the frozen
% static field, for a robot at rest". Never describe a moving state as
% "stable" on the basis of a Jacobian evaluated there -- use "capture
% susceptibility" or "forward trapping risk" for that quantity.
dgx = P.goal(1) - gx;
dgy = P.goal(2) - gy;
dist = hypot(dgx, dgy);
nz = dist > eps;
vDes = P.v0Robot * min(1, dist ./ P.dSlow);
Fx = zeros(1, M); Fy = zeros(1, M);
Fx(nz) = vDes(nz) .* dgx(nz) ./ dist(nz) / P.tau;
Fy(nz) = vDes(nz) .* dgy(nz) ./ dist(nz) / P.tau;

%% ---- Obstacles: force AND clearance in one pass ----------------------
clr = inf(1, M);
inObs = false(1, M);
for k = 1:numel(S.xObsList)
    xv = S.xObsList{k}; yv = S.yObsList{k};
    for ii = 1:M
        [dS, xq, yq] = p_poly_dist(gx(ii), gy(ii), xv, yv);
        if dS < clr(ii), clr(ii) = dS; end
        if dS < 0
            inObs(ii) = true;
            continue;
        end
        if dS > P.rInfObs, continue; end
        tx = gx(ii) - xq; ty = gy(ii) - yq;
        nrm = hypot(tx, ty);
        if nrm < eps, continue; end
        mag = P.aObs * exp((rRob - dS) / P.bObs);
        if dS < rRob, mag = mag + P.kBody * (rRob - dS); end
        Fx(ii) = Fx(ii) + mag * tx / nrm;
        Fy(ii) = Fy(ii) + mag * ty / nrm;
    end
end

Fx(inObs) = NaN; Fy(inObs) = NaN;

F.X = X; F.Y = Y; F.xs = xs; F.ys = ys;
F.Fx = reshape(Fx, ny, nx);
F.Fy = reshape(Fy, ny, nx);
F.Z  = hypot(F.Fx, F.Fy);
% Sampling note: interpolate Fx and Fy and then take the norm. Interpolating
% Z directly is invalid near an equilibrium -- the components pass smoothly
% through zero but the magnitude has a V-shaped kink, and averaging the
% magnitudes of two large opposing vectors never approaches zero. Sampling Z
% at a stalled robot overstated the force by roughly three orders of
% magnitude.
F.clearance  = reshape(clr, ny, nx);
F.gapToRobot = F.clearance - rRob;
F.inObs = reshape(inObs, ny, nx);
F.free  = ~F.inObs & isfinite(F.Z);
F.seed = S.seed; F.h = h;

[F.gx, F.gy] = gradient(F.Z, h, h);
F.gradMag = hypot(F.gx, F.gy);

%% ---- Locally attracting equilibrium candidates -----------------------
% TWO conditions, both required. An earlier version applied only the second
% and flagged 89-96% of free space.
%
% 1. THE POINT MUST BE AN EQUILIBRIUM. Stability is defined only where the
%    force vanishes. Evaluating a Jacobian at a point with large ||F|| says
%    nothing about attraction -- the same category error as evaluating
%    equilibrium stability at a moving state. Gated on ||F|| <= eqTol.
%
% 2. THE JACOBIAN MUST BE STRICTLY STABLE, WITH MARGIN. The full 4x4
%    [0 I; A -I/tau] is used because the system is second order and for
%    complex eigenvalues of A the 2x2 position test is not sufficient. But
%    Re(lambda) < 0 alone is degenerate here: in open space the radially
%    inward goal field gives eig(A) = {0, -v0/(tau r)}, and the exact ZERO
%    eigenvalue sits on the boundary. Finite differences return +/-eps
%    instead, so numerical noise decides, and it fell negative ~90% of the
%    time. A margin of stabMargin removes that whole class of false flags.
F.eqTol = 0.05 * P.v0Robot / P.tau;    % 5% of the maximum goal drive
F.stabMargin = 1e-3;                   % required distance from marginality
F.stable = false(ny, nx);
I2 = eye(2);
for j = 2:nx-1
    for i = 2:ny-1
        if F.inObs(i, j), continue; end
        if F.Z(i, j) > F.eqTol, continue; end          % not an equilibrium
        if any(F.inObs([i-1 i+1], j)) || any(F.inObs(i, [j-1 j+1])), continue; end
        A = [ (F.Fx(i, j+1) - F.Fx(i, j-1)) / (2*h), (F.Fx(i+1, j) - F.Fx(i-1, j)) / (2*h);
              (F.Fy(i, j+1) - F.Fy(i, j-1)) / (2*h), (F.Fy(i+1, j) - F.Fy(i-1, j)) / (2*h) ];
        if any(~isfinite(A(:))), continue; end
        if all(real(eig([zeros(2), I2; A, -I2 / P.tau])) < -F.stabMargin)
            F.stable(i, j) = true;
        end
    end
end

CACHE(key) = F;
end
