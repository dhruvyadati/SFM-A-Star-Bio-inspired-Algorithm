% Run_TrapDiscrimination.m
% Can the pre-run static force field DISCRIMINATE deadlock locations from
% places a robot slowed down and escaped?
%
% Capture rate alone measures only sensitivity, P(flagged | deadlock). A field
% that flags half the workspace captures half the deadlocks by chance, and a
% field that merely tracks obstacle clutter will flag anywhere a robot ever
% slows. Discrimination needs negatives.
%
%   POSITIVE : terminal position of a deadlocked trial.
%   NEGATIVE : minimum-speed position of a SUCCESSFUL trial, at least 3 m from
%              the goal -- the closest that robot came to stalling without
%              doing so. This is the hard negative; using arbitrary points
%              from successful trajectories would make the task trivial.
%
% Positives and negatives come from DIFFERENT map seeds, so there is no
% spatial leakage: the same trap basin cannot appear on both sides.
%
% Map-to-map variation is removed by scoring each point as the PERCENTILE of
% its force magnitude within its own map's free workspace. Absolute force
% values are not comparable across layouts.
%
% Every predictor is scored the same way, so the force field competes with
% geometry on equal terms. If obstacle clearance discriminates as well, the
% force field is a transformed clearance map and adds nothing.
%
% Usage:  cd to the repository root, then  run('v2/Run_TrapDiscrimination.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

if ~exist('gridStep','var');  gridStep  = 0.25; end
if ~exist('maxEach','var');   maxEach   = 25;   end
if ~exist('batchFile','var'); batchFile = fullfile('runs','v2','safety_gate.mat'); end
if ~exist('armName','var');   armName   = 'sfm'; end

outDir = fullfile('runs','v2'); if ~exist(outDir,'dir'); mkdir(outDir); end

L = load(batchFile);
T0 = L.T; P = L.P;
sub  = T0(strcmp(T0.method, armName), :);
posR = sub(strcmp(sub.outcome, 'deadlock'), :);
negR = sub(strcmp(sub.outcome, 'success'),  :);

nPos = min(maxEach, height(posR));
nNeg = min(maxEach, height(negR));
fprintf('=== Trap discrimination (arm: %s, grid %.2f m) ===\n', armName, gridStep);
fprintf('positives (deadlock) %d,  negatives (near-stall, escaped) %d\n\n', nPos, nNeg);
if nPos < 4 || nNeg < 4
    error('Run_TrapDiscrimination:tooFew', ...
          'Need >= 4 of each; have %d positive, %d negative.', nPos, nNeg);
end

% randomScore and distFromStart are NUISANCE baselines, not candidates.
% randomScore must land near AUC 0.5 -- if it does not, the evaluation
% pipeline itself is broken. distFromStart must be weak -- if it is strong,
% the groups are separated by trajectory PHASE rather than trap structure,
% which is exactly the artifact that made the first attempt look good.
preds = {'staticForce','staticAttract','obsClearance','goalDist', ...
         'randomScore','distFromStart'};
nP = numel(preds);
pctPos = nan(nPos, nP);
pctNeg = nan(nNeg, nP);
nuisPos = nan(nPos, 4);   % [elapsed_s, dist_from_start, dist_to_goal, |F| abs]
nuisNeg = nan(nNeg, 4);

for c = 1:nPos
    S = MakeScenario(posR.seed(c), P);
    R = RunTrial(S, armName, struct('logTrajectory', true));
    if ~strcmp(R.outcome, 'deadlock'), continue; end
    pctPos(c, :) = scorePoint(S, R.finalRobotPos, P, gridStep, nP);
    nuisPos(c, :) = [R.simSeconds, norm(R.finalRobotPos - P.start(:)), ...
                     norm(R.finalRobotPos - P.goal(:)), absForce(S, P, R.finalRobotPos)];
    fprintf('  POS %2d seed=%-11d staticForce pct = %.4f\n', c, posR.seed(c), pctPos(c,1));
end
for c = 1:nNeg
    S = MakeScenario(negR.seed(c), P);
    R = RunTrial(S, armName, struct('logTrajectory', true));
    if ~strcmp(R.outcome, 'success') || ~isfinite(R.minSpeed), continue; end
    pctNeg(c, :) = scorePoint(S, R.minSpeedPos, P, gridStep, nP);
    nuisNeg(c, :) = [R.minSpeedStep * P.dt, norm(R.minSpeedPos - P.start(:)), ...
                     norm(R.minSpeedPos - P.goal(:)), absForce(S, P, R.minSpeedPos)];
    fprintf('  NEG %2d seed=%-11d staticForce pct = %.4f  (min speed %.3f m/s)\n', ...
            c, negR.seed(c), pctNeg(c,1), R.minSpeed);
end

%% ---- Discrimination --------------------------------------------------
fprintf('\n============================================================\n');
fprintf('DISCRIMINATION  (lower percentile = flagged as hazardous)\n');
fprintf('============================================================\n');
fprintf('%-15s %10s %10s %9s %9s %9s\n', 'predictor', 'pos median', ...
        'neg median', 'Cliff d', 'p', 'AUC');
for pI = 1:nP
    a = pctPos(isfinite(pctPos(:,pI)), pI);
    b = pctNeg(isfinite(pctNeg(:,pI)), pI);
    if numel(a) < 3 || numel(b) < 3, continue; end
    [pv, ~, cliff] = StatsLib.rankSum(a, b);
    auc = (1 - cliff) / 2;   % P(positive scores lower than negative)
    fprintf('%-15s %10.4f %10.4f %9.3f %9.4f %9.3f\n', ...
            preds{pI}, median(a), median(b), cliff, pv, auc);
end

%% ---- Sampling sanity: are the groups separated by a nuisance? --------
fprintf('\n--- Sampling sanity (nuisance variables) ---\n');
nn = {'elapsed s','dist from start','dist to goal','ABS |F| m/s^2'};
fprintf('%-18s %12s %12s %10s\n', 'variable', 'pos median', 'neg median', 'Cliff d');
for k = 1:4
    a = nuisPos(isfinite(nuisPos(:,k)), k);
    b = nuisNeg(isfinite(nuisNeg(:,k)), k);
    if numel(a) < 3 || numel(b) < 3, continue; end
    [~, ~, cd] = StatsLib.rankSum(a, b);
    flag = '';
    if abs(cd) > 0.8 && k < 4, flag = '   <-- groups nearly separated by this'; end
    fprintf('%-18s %12.3f %12.3f %10.3f%s\n', nn{k}, median(a), median(b), cd, flag);
end

save(fullfile(outDir,'trap_discrimination.mat'), 'pctPos','pctNeg','preds', ...
     'nuisPos','nuisNeg','P');

fprintf('\nAUC 0.5 = no discrimination. Positives should score LOWER.\n');
fprintf('If obsClearance matches staticForce, the field is a transformed\n');
fprintf('clearance map and adds no information beyond geometry.\n');

    function pct = scorePoint(S, pt, P, gridStep, nP)
        % Percentile of each predictor at pt within this map's free workspace.
        FS = TrapField(S, P, gridStep, [], []);        % STATIC, pre-run
        free = ~FS.inObs & isfinite(FS.Z);
        pct = nan(1, nP);

        % Component-wise interpolation: the magnitude field has a kink at an
        % equilibrium and interpolating it directly is invalid there.
        fx = interp2(FS.X, FS.Y, FS.Fx, pt(1), pt(2), 'linear', NaN);
        fy = interp2(FS.X, FS.Y, FS.Fy, pt(1), pt(2), 'linear', NaN);
        zv = hypot(fx, fy);
        vals = FS.Z(free);
        pct(1) = mean(vals <= zv);

        st = interp2(FS.X, FS.Y, double(FS.stable), pt(1), pt(2), 'nearest', 0);
        A = FS.Z; A(~FS.stable) = inf;
        if st >= 0.5, pct(2) = mean(A(free) <= zv); else, pct(2) = 1.0; end

        C = inf(size(FS.X));
        for k = 1:numel(S.xObsList)
            xv = S.xObsList{k}; yv = S.yObsList{k};
            for ii = 1:numel(FS.X)
                if FS.inObs(ii), continue; end
                C(ii) = min(C(ii), p_poly_dist(FS.X(ii), FS.Y(ii), xv, yv));
            end
        end
        cv = interp2(FS.X, FS.Y, C, pt(1), pt(2), 'linear', NaN);
        pct(3) = mean(C(free) <= cv);

        G = hypot(FS.X - P.goal(1), FS.Y - P.goal(2));
        gv = hypot(pt(1) - P.goal(1), pt(2) - P.goal(2));
        pct(4) = mean(G(free) >= gv);   % far from goal = suspicious

        % Random field, seeded per map: deterministic but carries no
        % information. Expected AUC 0.5; anything else means the evaluation
        % pipeline is broken.
        rs = RandStream('twister', 'Seed', double(mod(S.seed, 2^31-1)));
        Rf = rand(rs, size(FS.X));
        rv = interp2(FS.X, FS.Y, Rf, pt(1), pt(2), 'linear', 0.5);
        pct(5) = mean(Rf(free) <= rv);

        % Distance from start: pure trajectory-phase nuisance.
        DS = hypot(FS.X - P.start(1), FS.Y - P.start(2));
        dv = hypot(pt(1) - P.start(1), pt(2) - P.start(2));
        pct(6) = mean(DS(free) >= dv);
    end


    function f = absForce(S, P, pt)
    % Exact net force magnitude at a point, static map only. Reported in
    % physical units alongside the within-map percentile: a 1st-percentile
    % force can still be physically large in one layout and near zero in
    % another, and the percentile alone hides that.
        F = ComputeForces(pt(:), [0;0], P.goal(:), P.rRobot, P.v0Robot, ...
                          S.xObsList, S.yObsList, P);
        f = norm(F);
    end
