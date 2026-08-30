% Run_HorizonPrediction.m
% Prospective task, per PROTOCOL section 10a:
%
%   Given a trajectory state, will a deadlock BEGIN within the next H seconds?
%
% This replaces endpoint association ("is the final stall position in a
% flagged region"), which is nearly circular -- a stalled robot must sit at a
% force equilibrium by Newton's second law.
%
%   POSITIVE : robot state at exactly (onset - H). Onset is the first step of
%              the stall window, not the confirmation step 5 s later, so the
%              lead time really is H.
%   NEGATIVE : minimum-speed state of a SUCCESSFUL trial, gated past the
%              acceleration phase and >= 3 m from start and goal. The closest
%              that robot came to stalling without doing so.
%
% Negatives are MATCHED to positives on elapsed time by greedy nearest
% assignment, because if deadlocks happen late and negatives early then
% trajectory phase does the work instead of trap structure.
%
% Predictors are scored ONLY from the static map field, which depends on map,
% goal and controller constants -- never on a trajectory, outcome or
% pedestrian snapshot.
%
% Prespecified family: staticForce, staticAttract, obsClearance.
% randomScore and distFromStart are pipeline controls, NOT hypotheses.
%
% Usage:  cd to the repository root, then  run('v2/Run_HorizonPrediction.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

if ~exist('H','var');        H = 5.0;   end          % primary horizon, s
if ~exist('gridStep','var'); gridStep = 0.25; end
if ~exist('maxPairs','var'); maxPairs = 40;  end

outDir = fullfile('runs','v2'); if ~exist(outDir,'dir'); mkdir(outDir); end
L = load(fullfile(outDir,'deadlock_batch.mat'));
T = L.T; rows = L.rows;

isDead = strcmp(T.outcome, 'deadlock') & isfinite(T.deadlockOnsetStep);
isSucc = strcmp(T.outcome, 'success')  & isfinite(T.minSpeed);
posIdx = find(isDead);
negIdx = find(isSucc);

% Positive is only usable if the trajectory extends H seconds before onset.
P0 = SfmParams();
hSteps = round(H / P0.dt);
posIdx = posIdx(T.deadlockOnsetStep(posIdx) > hSteps);

fprintf('=== Horizon prediction, H = %.1f s ===\n', H);
fprintf('candidate positives %d, candidate negatives %d\n', numel(posIdx), numel(negIdx));

%% ---- Match negatives to positives on elapsed time --------------------
posTime = (T.deadlockOnsetStep(posIdx) - hSteps) * P0.dt;
negTime = T.minSpeedStep(negIdx) * P0.dt;

nPair = min([maxPairs, numel(posIdx), numel(negIdx)]);
usedNeg = false(numel(negIdx), 1);
pairs = zeros(nPair, 2);
[~, order] = sort(posTime);          % deterministic, no RNG
for k = 1:nPair
    pi_ = posIdx(order(k));
    pt = (T.deadlockOnsetStep(pi_) - hSteps) * P0.dt;
    d = abs(negTime - pt); d(usedNeg) = inf;
    [~, j] = min(d);
    if ~isfinite(d(j)), break; end
    usedNeg(j) = true;
    pairs(k, :) = [pi_, negIdx(j)];
end
pairs = pairs(pairs(:,1) > 0, :);
nPair = size(pairs, 1);
fprintf('matched pairs: %d\n\n', nPair);
if nPair < 8
    error('Run_HorizonPrediction:tooFew', 'Only %d pairs.', nPair);
end

preds = {'staticForce','staticAttract','obsClearance','randomScore','distFromStart'};
nP = numel(preds);
scorePos = nan(nPair, nP);  scoreNeg = nan(nPair, nP);
nuisPos  = nan(nPair, 4);   nuisNeg  = nan(nPair, 4);
condPos  = cell(nPair, 1);
leadTime = nan(nPair, 1);

for k = 1:nPair
    % ---- positive ----
    rp = rows(pairs(k,1));
    Pp = SfmParams('numObstacles', rp.condObs, 'maxArea', rp.condArea, ...
                   'numPedestrians', rp.condPed, 'v0Ped', rp.condSpeed);
    Sp = MakeScenario(rp.seed, Pp);
    Rp = RunTrial(Sp, 'sfm', struct('logTrajectory', true));
    if ~strcmp(Rp.outcome,'deadlock') || ~isfinite(Rp.deadlockOnsetStep), continue; end
    stepP = Rp.deadlockOnsetStep - hSteps;
    if stepP < 1 || stepP > size(Rp.traj, 2), continue; end
    ptP = Rp.traj(:, stepP);
    scorePos(k,:) = scoreStatic(Sp, Pp, gridStep, ptP, nP);
    nuisPos(k,:)  = [stepP*Pp.dt, norm(ptP - Pp.start(:)), norm(ptP - Pp.goal(:)), ...
                     absF(Sp, Pp, ptP)];
    condPos{k} = rp.condition;
    leadTime(k) = H;

    % ---- negative ----
    rn = rows(pairs(k,2));
    Pn = SfmParams('numObstacles', rn.condObs, 'maxArea', rn.condArea, ...
                   'numPedestrians', rn.condPed, 'v0Ped', rn.condSpeed);
    Sn = MakeScenario(rn.seed, Pn);
    Rn = RunTrial(Sn, 'sfm', struct('logTrajectory', true));
    if ~strcmp(Rn.outcome,'success') || ~isfinite(Rn.minSpeed), continue; end
    ptN = Rn.minSpeedPos;
    scoreNeg(k,:) = scoreStatic(Sn, Pn, gridStep, ptN, nP);
    nuisNeg(k,:)  = [Rn.minSpeedStep*Pn.dt, norm(ptN - Pn.start(:)), ...
                     norm(ptN - Pn.goal(:)), absF(Sn, Pn, ptN)];

    fprintf('pair %2d  POS %-10s t=%6.1fs pct=%.4f  |  NEG t=%6.1fs pct=%.4f\n', ...
            k, rp.condition, nuisPos(k,1), scorePos(k,1), nuisNeg(k,1), scoreNeg(k,1));
end

%% ---- Discrimination --------------------------------------------------
fprintf('\n============================================================\n');
fprintf('HORIZON PREDICTION, H = %.1f s  (lower score = flagged)\n', H);
fprintf('============================================================\n');
fprintf('%-15s %10s %10s %9s %9s %9s\n', 'predictor', 'pos med', 'neg med', ...
        'Cliff d', 'p raw', 'AUC');
pRaw = nan(1, nP); aucV = nan(1, nP);
for pI = 1:nP
    a = scorePos(isfinite(scorePos(:,pI)), pI);
    b = scoreNeg(isfinite(scoreNeg(:,pI)), pI);
    if numel(a) < 4 || numel(b) < 4, continue; end
    [pv, ~, cd] = StatsLib.rankSum(a, b);
    pRaw(pI) = pv; aucV(pI) = (1 - cd)/2;
    fprintf('%-15s %10.4f %10.4f %9.3f %9.4f %9.3f\n', ...
            preds{pI}, median(a), median(b), cd, pv, aucV(pI));
end

% Holm across the PRESPECIFIED family only (first three). Controls excluded.
fam = 1:3;
adj = StatsLib.holm(pRaw(fam));
fprintf('\nHolm-adjusted across the prespecified family {%s, %s, %s}:\n', ...
        preds{1}, preds{2}, preds{3});
for i = 1:numel(fam)
    fprintf('  %-15s p_adj = %.4f\n', preds{fam(i)}, adj(i));
end
fprintf('Controls (excluded from the family): %s AUC %.3f, %s AUC %.3f\n', ...
        preds{4}, aucV(4), preds{5}, aucV(5));

%% ---- Sampling sanity -------------------------------------------------
fprintf('\n--- Sampling sanity (nuisance variables) ---\n');
nn = {'elapsed s','dist from start','dist to goal','ABS |F| m/s^2'};
for k = 1:4
    a = nuisPos(isfinite(nuisPos(:,k)), k);
    b = nuisNeg(isfinite(nuisNeg(:,k)), k);
    if numel(a) < 4 || numel(b) < 4, continue; end
    [~, ~, cd] = StatsLib.rankSum(a, b);
    flag = ''; if abs(cd) > 0.8 && k < 4, flag = '   <-- nuisance separates groups'; end
    fprintf('  %-16s pos %8.3f  neg %8.3f  Cliff d %6.3f%s\n', nn{k}, ...
            median(a), median(b), cd, flag);
end

save(fullfile(outDir, sprintf('horizon_H%.0f.mat', H)), ...
     'scorePos','scoreNeg','nuisPos','nuisNeg','preds','pairs','H','condPos','aucV','pRaw');
fprintf('\nLead time per positive: %.1f s (by construction)\n', H);
fprintf('Wrote %s\n', fullfile(outDir, sprintf('horizon_H%.0f.mat', H)));

% ---------------------------------------------------------------------
function pct = scoreStatic(S, P, h, pt, nP)
% All predictors from the STATIC map field only: map, goal, controller
% constants. No trajectory, outcome, or pedestrian input.
    F = MapField(S, P, h);
    free = F.free;
    pct = nan(1, nP);

    % Component-wise interpolation, then magnitude. Interpolating the
    % magnitude field is invalid near an equilibrium.
    fx = interp2(F.X, F.Y, F.Fx, pt(1), pt(2), 'linear', NaN);
    fy = interp2(F.X, F.Y, F.Fy, pt(1), pt(2), 'linear', NaN);
    zv = hypot(fx, fy);
    pct(1) = mean(F.Z(free) <= zv);

    st = interp2(F.X, F.Y, double(F.stable), pt(1), pt(2), 'nearest', 0);
    if st >= 0.5
        A = F.Z; A(~F.stable) = inf;
        pct(2) = mean(A(free) <= zv);
    else
        pct(2) = 1.0;
    end

    cv = interp2(F.X, F.Y, F.clearance, pt(1), pt(2), 'linear', NaN);
    pct(3) = mean(F.clearance(free) <= cv);

    rs = RandStream('twister','Seed', double(mod(S.seed, 2^31-1)));
    Rf = rand(rs, size(F.X));
    rv = interp2(F.X, F.Y, Rf, pt(1), pt(2), 'linear', 0.5);
    pct(4) = mean(Rf(free) <= rv);

    DS = hypot(F.X - P.start(1), F.Y - P.start(2));
    dv = hypot(pt(1) - P.start(1), pt(2) - P.start(2));
    pct(5) = mean(DS(free) >= dv);
end

function f = absF(S, P, pt)
    Fv = ComputeForces(pt(:), [0;0], P.goal(:), P.rRobot, P.v0Robot, ...
                       S.xObsList, S.yObsList, P);
    f = norm(Fv);
end
