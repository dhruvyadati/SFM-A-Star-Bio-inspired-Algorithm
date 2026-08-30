function T = TrapField(S, P, h, pedPos, pedVel, cache)
% TrapField computes the net-force magnitude field for a probe agent and
% classifies its low-force regions, using the SAME parameters as the
% navigation experiments.
%
%   S        scenario struct (obstacles, goal)
%   P        parameter struct -- the NAVIGATION parameters, not a softened set
%   h        grid step (m)
%   pedPos   2 x nPed pedestrian positions, or [] for the static field
%   pedVel   2 x nPed pedestrian velocities, or [] (unused; probe is at rest)
%   cache    optional struct from a previous call on the SAME scenario and
%            grid; reuses the obstacle field, which depends only on geometry
%
% Returns T with .X .Y .Z .stable .gradMag .cache
%
% Three defects in the legacy diagnostic are corrected here.
%
% 1. PARAMETERS. The legacy code evaluated the field with a_s = 3000 and
%    r_inf,s = 3.5 while the navigation simulation used a_s = 10000 and
%    r_inf,s = 1, justifying the mismatch as making the field "legible". A
%    diagnostic computed on a different dynamical system cannot explain that
%    system's failures. This takes P directly, so field and simulation cannot
%    diverge.
%
% 2. DYNAMIC AGENTS. The legacy diagnostic omitted moving pedestrians though
%    the system under study is a dynamic crowd. Passing pedPos includes the
%    dynamic-agent term, so the field can be evaluated as it stood at the
%    moment of an observed deadlock.
%
% 3. ATTRACTING EQUILIBRIA. The legacy code declared every point with
%    ||F|| < delta a trap. Small net force also occurs on ridges, saddles and
%    transit regions that a moving agent passes straight through.
%    Classification here uses the FULL 4x4 Jacobian [0 I; A -I/tau], because
%    the system is second order: for complex eigenvalues of A = dF/dx the 2x2
%    position test is necessary but NOT sufficient and yields false positives
%    on underdamped spirals. See Test_StabilityCriterion.
%
%    With moving pedestrians the field is time-varying, so this identifies
%    locally attracting equilibrium CANDIDATES of the frozen instantaneous
%    field. It is a diagnostic, not a stability proof for the full system.
%
% The field is evaluated by direct vectorised formulae rather than by calling
% ComputeForces per grid point: the probe is a single agent, so the N-body
% routine was computing thousands of irrelevant pairwise terms per point.

if nargin < 6, cache = []; end

xs = P.xMin:h:P.xMax;
ys = P.yMin:h:P.yMax;
[T.X, T.Y] = meshgrid(xs, ys);
[ny, nx] = size(T.X);
gx = T.X(:).'; gy = T.Y(:).';
M = numel(gx);

rRob = P.rRobot;
aRob = P.aPed;
if isfield(P, 'aPedRobot') && ~isempty(P.aPedRobot), aRob = P.aPedRobot; end

%% ---- 1. Goal attraction, probe at rest -------------------------------
dgx = P.goal(1) - gx;
dgy = P.goal(2) - gy;
dist = hypot(dgx, dgy);
nz = dist > eps;
vDes = P.v0Robot * min(1, dist ./ P.dSlow);
Fx = zeros(1, M); Fy = zeros(1, M);
Fx(nz) = vDes(nz) .* dgx(nz) ./ dist(nz) / P.tau;
Fy(nz) = vDes(nz) .* dgy(nz) ./ dist(nz) / P.tau;

%% ---- 2. Static obstacle term, cached on geometry ---------------------
if ~isempty(cache) && isequal(cache.h, h) && isequal(cache.seed, S.seed)
    Ox = cache.Ox; Oy = cache.Oy; inObs = cache.inObs;
else
    Ox = zeros(1, M); Oy = zeros(1, M); inObs = false(1, M);
    for k = 1:numel(S.xObsList)
        xv = S.xObsList{k}; yv = S.yObsList{k};
        pad = P.rInfObs + rRob;
        near = find(gx >= min(xv)-pad & gx <= max(xv)+pad & ...
                    gy >= min(yv)-pad & gy <= max(yv)+pad);
        for ii = near
            [dS, xq, yq] = p_poly_dist(gx(ii), gy(ii), xv, yv);
            if dS < 0, inObs(ii) = true; continue; end
            if dS > P.rInfObs, continue; end
            tx = gx(ii) - xq; ty = gy(ii) - yq;
            nrm = hypot(tx, ty);
            if nrm < eps, continue; end
            mag = P.aObs * exp((rRob - dS) / P.bObs);
            if dS < rRob, mag = mag + P.kBody * (rRob - dS); end
            Ox(ii) = Ox(ii) + mag * tx / nrm;
            Oy(ii) = Oy(ii) + mag * ty / nrm;
        end
    end
    cache = struct('h', h, 'seed', S.seed, 'Ox', Ox, 'Oy', Oy, 'inObs', inObs);
end
Fx = Fx + Ox; Fy = Fy + Oy;

%% ---- 3. Dynamic agent term -------------------------------------------
if ~isempty(pedPos)
    rS = rRob + P.rPed;
    for j = 1:size(pedPos, 2)
        ddx = gx - pedPos(1, j);
        ddy = gy - pedPos(2, j);
        dd = hypot(ddx, ddy);
        act = dd <= P.rInfPed & dd > eps;
        if ~any(act), continue; end
        mag = aRob * exp((rS - dd(act)) / P.bPed) ...
            + P.kBody * max(0, rS - dd(act));
        Fx(act) = Fx(act) + mag .* ddx(act) ./ dd(act);
        Fy(act) = Fy(act) + mag .* ddy(act) ./ dd(act);
    end
end

Fx(inObs) = NaN; Fy(inObs) = NaN;

FxG = reshape(Fx, ny, nx);
FyG = reshape(Fy, ny, nx);
T.Fx = FxG; T.Fy = FyG;
T.Z = hypot(FxG, FyG);
% Callers sampling this field at a point MUST interpolate Fx and Fy and then
% take the magnitude. Interpolating Z directly is wrong near an equilibrium:
% the components pass smoothly through zero there, but the magnitude has a
% V-shaped kink, and averaging the magnitudes of two large opposing vectors
% never approaches zero. Sampling Z at a deadlock overestimated the true
% force by roughly three orders of magnitude.
T.inObs = reshape(inObs, ny, nx);
T.h = h; T.xs = xs; T.ys = ys; T.cache = cache;

%% ---- Gradient of the magnitude field ---------------------------------
% Units are m/s^2 per m, i.e. s^-2. The legacy code reported an average
% gradient with no units and no reference scale.
[T.gx, T.gy] = gradient(T.Z, h, h);
T.gradMag = hypot(T.gx, T.gy);

%% ---- Attracting-equilibrium classification ---------------------------
T.stable = false(ny, nx);
I2 = eye(2);
for j = 2:nx-1
    for i = 2:ny-1
        if T.inObs(i, j), continue; end
        if any(T.inObs([i-1 i+1], j)) || any(T.inObs(i, [j-1 j+1])), continue; end
        A = [ (FxG(i, j+1) - FxG(i, j-1)) / (2*h), (FxG(i+1, j) - FxG(i-1, j)) / (2*h);
              (FyG(i, j+1) - FyG(i, j-1)) / (2*h), (FyG(i+1, j) - FyG(i-1, j)) / (2*h) ];
        if any(~isfinite(A(:))), continue; end
        if all(real(eig([zeros(2), I2; A, -I2 / P.tau])) < 0)
            T.stable(i, j) = true;
        end
    end
end
T.attractingCandidate = T.stable;
end
