% Run_EnrichmentNull.m
% Empirical null for "is a pedestrian nearby at deadlock onset more often than
% expected?"
%
% The earlier estimate used a uniform-density Poisson argument: 48 agents over
% 2500 m^2 gives 0.136 expected within 1.5 m, so P(>=1) ~ 12.7%. That assumes
% pedestrians are spatially independent and uniformly distributed. They are
% neither -- they avoid obstacles, they concentrate in navigable corridors, and
% they interact with each other and with the robot. The uniform figure is
% therefore the wrong denominator, and the resulting "5.5x enrichment" cannot
% be trusted in either direction.
%
% This measures the null directly: how often is a pedestrian within 1.5 m at
% MATCHED NON-DEADLOCK states drawn from the same trials? Same crowd, same
% maps, same conditions, same robot -- differing only in that no deadlock is
% beginning.
%
% Usage:  cd to the repository root, then  run('v2/Run_EnrichmentNull.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

if ~exist('maxTrials','var'); maxTrials = 30;  end
if ~exist('nearR','var');     nearR     = 1.5; end   % surface gap, m
if ~exist('bufferS','var');   bufferS   = 5.0; end   % keep clear of onset

outDir = fullfile('runs','v2');
L = load(fullfile(outDir,'deadlock_batch.mat'));
T = L.T; rows = L.rows;

dl = find(strcmp(T.outcome,'deadlock') & isfinite(T.deadlockOnsetStep));
nT = min(maxTrials, numel(dl));
fprintf('=== Empirical null for pedestrian proximity ===\n');
fprintf('deadlock trials used: %d, radius %.2f m surface gap\n\n', nT, nearR);

onsetNear = nan(nT, 1);
nullNear  = nan(nT, 1);
nullCount = nan(nT, 1);
onsetCount = nan(nT, 1);

for k = 1:nT
    r = rows(dl(k));
    P = SfmParams('numObstacles', r.condObs, 'maxArea', r.condArea, ...
                  'numPedestrians', r.condPed, 'v0Ped', r.condSpeed);
    S = MakeScenario(r.seed, P);
    R = RunTrial(S, 'sfm', struct('logTrajectory', true, 'logProximity', true));
    if ~strcmp(R.outcome,'deadlock'), continue; end

    onsetNear(k)  = double(r.onset.nPedNear >= 1);
    onsetCount(k) = r.onset.nPedNear;

    % Null states: same trial, but well before onset so the robot is neither
    % in nor entering the basin. Sampled every 0.5 s over the eligible span.
    bufSteps = round(bufferS / P.dt);
    lastOK = R.deadlockOnsetStep - bufSteps;
    if lastOK < 200, continue; end
    stepsToCheck = 100:50:lastOK;      % skip the first second of acceleration

    stepsToCheck = stepsToCheck(stepsToCheck <= numel(R.proxSeries));
    prox = R.proxSeries(stepsToCheck);
    nullNear(k)  = mean(prox >= 1);
    nullCount(k) = mean(prox);

    fprintf('trial %2d %-10s onset near=%d  null P(near)=%.3f  null mean count=%.3f\n', ...
            k, r.condition, r.onset.nPedNear, nullNear(k), nullCount(k));
end

ok = isfinite(onsetNear) & isfinite(nullNear);
fprintf('\n============================================================\n');
fprintf('ENRICHMENT vs EMPIRICAL NULL (n = %d trials)\n', sum(ok));
fprintf('============================================================\n');
pOnset = mean(onsetNear(ok));
pNull  = mean(nullNear(ok));
fprintf('  P(pedestrian within %.2f m) at ONSET        : %.3f\n', nearR, pOnset);
fprintf('  P(pedestrian within %.2f m) at MATCHED null : %.3f\n', nearR, pNull);
fprintf('  empirical enrichment                        : %.2fx\n', pOnset / max(pNull, eps));
fprintf('  (uniform-density estimate was 0.127 -> 5.5x, which assumed\n');
fprintf('   independent uniform pedestrians and is not the right null)\n');

% Paired test: each trial contributes its own null, so this is matched.
[pv, zv, eff] = StatsLib.signedRank(onsetNear(ok), nullNear(ok));
fprintf('\n  paired signed-rank: z = %.3f, p = %.4g, rank-biserial = %.3f\n', zv, pv, eff);
fprintf('  mean count at onset %.3f vs null %.3f\n', ...
        mean(onsetCount(ok)), mean(nullCount(ok)));

save(fullfile(outDir,'enrichment_null.mat'), 'onsetNear','nullNear', ...
     'onsetCount','nullCount','nearR','bufferS');
