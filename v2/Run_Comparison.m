% Run_Comparison.m
% Head-to-head comparison of hybrid A*+SFM against SFM-only, under the
% corrected force model and the corrected evaluation protocol.
%
% The hybrid runs here with NO blockage recovery, so this measures the strict
% one-shot variant.
%
% What changed relative to the legacy model, and why:
%   1. Repulsion uses unit direction vectors and decays monotonically with
%      distance, so it is maximal at contact instead of vanishing there.
%   2. Goal attraction uses a unit direction vector, so its magnitude no
%      longer grows with the square of the distance to the subgoal.
%   3. vMin is removed; agents can decelerate to rest.
%   4. Integration is genuinely semi-implicit Euler.
%   5. Obstacle penetration is corrected outward instead of reinforced.
%   6. The two arms run in SEPARATE simulations from a shared seed, so they
%      no longer exert forces on each other.
%   7. Pedestrians resample a goal on arrival instead of holding one goal.
%   8. Success requires sustained dwell inside the goal radius, not a single
%      latched instant.
%   9. A*-infeasible maps are counted and reported rather than silently
%      scored as a double failure.
%
% Usage:  cd to the repository root, then  run('v2/Run_Comparison.m')

addpath(fileparts(mfilename('fullpath')));      % v2/
addpath(fileparts(fileparts(mfilename('fullpath'))));  % repo root

if ~exist('numTrials','var');   numTrials   = 100;      end
if ~exist('MASTER_SEED','var'); MASTER_SEED = 20260728; end

outDir = fullfile('runs', 'v2');
if ~exist(outDir, 'dir'); mkdir(outDir); end

P = SfmParams();

rng(MASTER_SEED, 'twister');
seeds = randi([1, 2^31-1], numTrials, 1);

methods = {'hybrid', 'sfm'};
rows = struct([]);
nInfeasible = 0;

fprintf('=== Comparison: corrected model, separated arms, %d trials ===\n', numTrials);
tAll = tic;

for k = 1:numTrials
    S = MakeScenario(seeds(k), P);

    if ~S.feasible
        nInfeasible = nInfeasible + 1;
        fprintf('trial %3d/%d seed=%-11d A* INFEASIBLE (excluded)\n', ...
                k, numTrials, seeds(k));
        continue;
    end

    for m = 1:numel(methods)
        R = RunTrial(S, methods{m});
        R.trial = k;
        if isempty(rows), rows = R; else, rows(end+1) = R; end %#ok<SAGROW>
    end

    h = rows(end-1); s = rows(end);
    fprintf(['trial %3d/%d seed=%-11d | hybrid %-8s len=%6.1f col=%d ' ...
             '| sfm %-8s len=%6.1f col=%d\n'], ...
        k, numTrials, seeds(k), h.outcome, h.pathLength, ...
        h.pedCollisions + h.obsCollisions, ...
        s.outcome, s.pathLength, s.pedCollisions + s.obsCollisions);
end

fprintf('\nTotal wall-clock: %.1f s\n', toc(tAll));

%% ---- Summary ---------------------------------------------------------
T = struct2table(rows);
writetable(T, fullfile(outDir, 'comparison_per_trial.csv'));
save(fullfile(outDir, 'comparison.mat'), 'T', 'P', 'seeds', 'MASTER_SEED', 'nInfeasible');

fprintf('\n============================================================\n');
fprintf('SUMMARY  (corrected model, separated simulations)\n');
fprintf('============================================================\n');
fprintf('Trials attempted      : %d\n', numTrials);
fprintf('A*-infeasible maps    : %d (%.1f%%, excluded from both arms)\n', ...
        nInfeasible, 100 * nInfeasible / numTrials);
fprintf('Trials evaluated      : %d\n\n', numTrials - nInfeasible);

for m = 1:numel(methods)
    sel = strcmp(T.method, methods{m});
    sub = T(sel, :);
    ok  = sub.success == 1;

    fprintf('--- %s (n = %d) ---\n', upper(methods{m}), height(sub));
    fprintf('  success rate        : %5.1f%%\n', 100 * mean(sub.success));
    fprintf('  deadlock            : %5.1f%%\n', 100 * mean(strcmp(sub.outcome,'deadlock')));
    fprintf('  timeout             : %5.1f%%\n', 100 * mean(strcmp(sub.outcome,'timeout')));
    fprintf('  path length (succ)  : %6.1f m  (n=%d)\n', mean(sub.pathLength(ok)), sum(ok));
    fprintf('  final dist (ALL)    : %6.1f m\n', mean(sub.finalDistToGoal));
    fprintf('  ped collisions/trial: %6.2f\n', mean(sub.pedCollisions));
    fprintf('  obs collisions/trial: %6.2f\n', mean(sub.obsCollisions));
    fprintf('  min ped gap         : %6.2f m\n', min(sub.minPedGap));
    fprintf('  min obs gap         : %6.2f m\n', min(sub.minObsGap));
    fprintf('  personal-zone time  : %6.2f s\n', mean(sub.personalTime));
    fprintf('  mean |accel|        : %6.2f m/s^2\n', mean(sub.meanAccel));
    fprintf('  mean |jerk|         : %6.2f m/s^3\n', mean(sub.meanJerk));
    fprintf('  A* plan time        : %6.3f s\n', mean(sub.astarSeconds));
    fprintf('  SFM step time       : %6.2f ms  (%.0f Hz)\n\n', ...
            1000 * mean(sub.meanStepSeconds), mean(sub.controlHz));
end

%% ---- Paired comparison ----------------------------------------------
% Both arms ran on the SAME scenarios, so the samples are paired. The legacy
% analysis used an unpaired rank-sum test, which throws that pairing away.
hyb = T(strcmp(T.method,'hybrid'), :);
sfm = T(strcmp(T.method,'sfm'),    :);

bothOK = hyb.success == 1 & sfm.success == 1;
fprintf('--- Paired comparison ---\n');
fprintf('Trials where BOTH arms succeeded: %d\n', sum(bothOK));

if sum(bothOK) >= 6
    d = sfm.pathLength(bothOK) - hyb.pathLength(bothOK);
    [pSign, zSign, effSign] = StatsLib.signedRank( ...
        hyb.pathLength(bothOK), sfm.pathLength(bothOK));
    fprintf('  mean paired path-length saving: %.1f m\n', mean(d));
    fprintf('  Wilcoxon SIGNED-RANK (paired) : z = %.3f, p = %.6g\n', zSign, pSign);
    fprintf('  effect size (rank-biserial)   : %.3f\n', effSign);
else
    fprintf('  too few jointly-successful trials for a paired test\n');
end

% Distance-to-goal over ALL trials, not just successes. The legacy analysis
% compared path length on successful runs only, while the two arms had very
% different success counts, which is a selection bias. This uses every
% evaluated trial.
[pAll, zAll, effAll] = StatsLib.signedRank(hyb.finalDistToGoal, sfm.finalDistToGoal);
fprintf('\n  Final distance-to-goal, ALL trials (no selection bias):\n');
fprintf('    hybrid %.2f m vs sfm %.2f m\n', ...
        mean(hyb.finalDistToGoal), mean(sfm.finalDistToGoal));
fprintf('    signed-rank z = %.3f, p = %.6g, effect = %.3f\n', zAll, pAll, effAll);

% Exact McNemar test on the paired success/failure table.
b = sum(hyb.success == 1 & sfm.success == 0);
c = sum(hyb.success == 0 & sfm.success == 1);
fprintf('\n  McNemar discordant pairs: hybrid-only=%d, sfm-only=%d\n', b, c);
[pMc, orMc] = StatsLib.mcnemarExact(b, c);
fprintf('  McNemar exact p = %.6g, odds ratio = %.2f\n', pMc, orMc);

% Wilson score intervals on each success rate.
nH = height(hyb); nS = height(sfm);
[loH, hiH] = StatsLib.wilsonCI(sum(hyb.success), nH);
[loS, hiS] = StatsLib.wilsonCI(sum(sfm.success), nS);
fprintf('\n  Success rate, Wilson 95%% CI:\n');
fprintf('    hybrid %.1f%% [%.1f, %.1f]\n', 100*mean(hyb.success), 100*loH, 100*hiH);
fprintf('    sfm    %.1f%% [%.1f, %.1f]\n', 100*mean(sfm.success), 100*loS, 100*hiS);

fprintf('\nWrote %s\n', fullfile(outDir, 'comparison_per_trial.csv'));
fprintf('============================================================\n');
