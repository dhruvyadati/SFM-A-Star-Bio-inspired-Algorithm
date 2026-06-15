% CSVTotalDistanceVsNumberTrials.m
% Collects total path distances over N trials for A*+SFM and SFM-only.
% Saves seeds for reproducibility and runs a Wilcoxon rank-sum test
% (non-parametric, suitable since only successful trials are included).

numberOfTrials = 50;

dist_astar_all = [];   % distances for A*+SFM (successful trials only)
dist_sfm_all   = [];   % distances for SFM-only (successful trials only)

% Distinct, saved, reproducible seed per trial (see Success_Rate.m note:
% never use `s = rng('shuffle'); rng(s.Seed)` — it resets every trial to seed 0).
rng('shuffle');
trial_seeds = randi([1, 2^31-1], numberOfTrials, 1);

% Per-trial records for the committed seed log (runs/)
per_trial_dist_astar = zeros(numberOfTrials, 1);
per_trial_dist_sfm   = zeros(numberOfTrials, 1);
per_trial_astar_ok   = zeros(numberOfTrials, 1);
per_trial_sfm_ok     = zeros(numberOfTrials, 1);

T = table();

for i = 1:numberOfTrials
    rng(trial_seeds(i), 'twister');   % reproducible, distinct per-trial seed

    [dist_astarsfm, dist_sfm, SFM_A_success, SFM_success] = ...
        GetTotalDistanceVsNumberTrialsVariables;

    % Only record distance for successful runs
    if SFM_A_success == 1
        dist_astar_all(end+1) = dist_astarsfm; %#ok<SAGROW>
    end
    if SFM_success == 1
        dist_sfm_all(end+1) = dist_sfm; %#ok<SAGROW>
    end

    % Keep the full per-trial record for the seed log
    per_trial_dist_astar(i) = dist_astarsfm;
    per_trial_dist_sfm(i)   = dist_sfm;
    per_trial_astar_ok(i)   = SFM_A_success;
    per_trial_sfm_ok(i)     = SFM_success;

    % Store 0 for failed runs in the table (for record keeping)
    newRow = table( ...
        dist_sfm   * SFM_success,   ...
        dist_astarsfm * SFM_A_success, ...
        'VariableNames', {'SFM', 'AStarSFM'});
    T = [T; newRow]; %#ok<AGROW>
end

% ── Summary ──────────────────────────────────────────────────────────
fprintf('\n=== Path Distance Results (successful runs only) ===\n');
fprintf('A*+SFM:   n=%d, mean=%.1f, std=%.1f\n', ...
    numel(dist_astar_all), mean(dist_astar_all), std(dist_astar_all));
fprintf('SFM-only: n=%d, mean=%.1f, std=%.1f\n', ...
    numel(dist_sfm_all),   mean(dist_sfm_all),   std(dist_sfm_all));

% ── Wilcoxon Rank-Sum Test (non-parametric, two-tailed) ───────────────
% H0: path lengths drawn from same distribution.
% Implemented directly via the Mann-Whitney U normal approximation (with
% tie and continuity correction) so no Statistics Toolbox is required.
[p_ranksum, z_ranksum] = wilcoxonRankSum(dist_astar_all, dist_sfm_all);
stats_ranksum.zval = z_ranksum;   % kept for downstream/StatisticalTests.m
fprintf('\n=== Wilcoxon Rank-Sum Test ===\n');
fprintf('z-statistic: %.4f\n', stats_ranksum.zval);
fprintf('p-value:     %.6f\n', p_ranksum);
if p_ranksum < 0.05
    fprintf('Result: STATISTICALLY SIGNIFICANT (p < 0.05)\n');
else
    fprintf('Result: not significant at alpha = 0.05\n');
end

% ── Save results ──────────────────────────────────────────────────────
% Bulky outputs (NOT tracked in git)
writetable(T, 'PathLengthData.csv');
save('path_distance_results.mat', ...
    'dist_astar_all', 'dist_sfm_all', 'trial_seeds', ...
    'p_ranksum', 'stats_ranksum');

% Committed per-trial seed log (runs/) — the reproducibility record.
% rng(seed) regenerates the exact obstacles and pedestrians for that trial.
if ~exist('runs', 'dir'); mkdir('runs'); end
seedLog = table((1:numberOfTrials)', trial_seeds, ...
    per_trial_astar_ok, per_trial_dist_astar, ...
    per_trial_sfm_ok,   per_trial_dist_sfm, ...
    'VariableNames', {'trial', 'seed', ...
        'AStarSFM_success', 'AStarSFM_dist', ...
        'SFM_success', 'SFM_dist'});
writetable(seedLog, fullfile('runs', 'path_distance_seeds.csv'));

fprintf('\nData saved to PathLengthData.csv, path_distance_results.mat, and runs/path_distance_seeds.csv\n');
fprintf('To reproduce trial i exactly: rng(trial_seeds(i))\n');

% ── Plot ─────────────────────────────────────────────────────────────
figure; hold on;
means = [mean(dist_sfm_all), mean(dist_astar_all)];
sems  = [std(dist_sfm_all)/sqrt(numel(dist_sfm_all)), ...
         std(dist_astar_all)/sqrt(numel(dist_astar_all))];
bar(["SFM Only", "A* + SFM"], means);
errorbar([1 2], means, 1.96*sems, 'o', 'LineStyle', 'none');
ylabel('Average Total Distance');
title('Average Total Distance for A* + SFM vs SFM-Only Model');

% ── Local function: Wilcoxon rank-sum (Mann-Whitney U) ────────────────
function [p, z] = wilcoxonRankSum(x, y)
% Two-sided Wilcoxon rank-sum test via the normal approximation.
% Matches MATLAB's ranksum(...) 'approximate' method (tie-corrected
% variance + continuity correction) without needing the Statistics Toolbox.
    x = x(:); y = y(:);
    n1 = numel(x); n2 = numel(y); N = n1 + n2;

    % Average ranks of the pooled sample (handles ties)
    pooled = [x; y];
    [sorted, ord] = sort(pooled);
    r = zeros(N, 1);
    tieTerm = 0;          % sum over tie groups of (t^3 - t)
    i = 1;
    while i <= N
        j = i;
        while j < N && sorted(j+1) == sorted(i)
            j = j + 1;
        end
        r(i:j) = (i + j) / 2;          % average rank for this tie group
        t = j - i + 1;
        tieTerm = tieTerm + (t^3 - t);
        i = j + 1;
    end
    ranks(ord) = r;                    % map ranks back to original order

    W    = sum(ranks(1:n1));           % rank sum of the first sample (x)
    muW  = n1 * (N + 1) / 2;
    varW = (n1 * n2 / 12) * ((N + 1) - tieTerm / (N * (N - 1)));

    % Continuity-corrected z and two-sided p-value (base-MATLAB erfc)
    z = (W - muW - 0.5 * sign(W - muW)) / sqrt(varW);
    p = erfc(abs(z) / sqrt(2));
end
