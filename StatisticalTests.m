% StatisticalTests.m
% Loads saved results and reports all statistical tests in a readable summary.
% Run Success_Rate.m and CSVTotalDistanceVsNumberTrials.m first to generate
% the .mat files this script reads.

fprintf('============================================================\n');
fprintf('STATISTICAL SIGNIFICANCE REPORT\n');
fprintf('============================================================\n\n');

% ── Success Rate Z-Test ───────────────────────────────────────────────
if exist('success_rate_results.mat', 'file')
    sr = load('success_rate_results.mat');
    fprintf('--- Success Rate (N = %d trials) ---\n', sr.numTrials);
    fprintf('  SFM-only:  %.1f%%\n', sr.p_sfm   * 100);
    fprintf('  A*+SFM:    %.1f%%\n', sr.p_astar * 100);
    fprintf('  Two-proportion z-test:\n');
    fprintf('    z = %.4f,  p = %.6f\n', sr.z_stat, sr.p_value);
    if sr.p_value < 0.001
        fprintf('    *** p < 0.001 (highly significant)\n');
    elseif sr.p_value < 0.01
        fprintf('    ** p < 0.01\n');
    elseif sr.p_value < 0.05
        fprintf('    * p < 0.05\n');
    else
        fprintf('    n.s. (p >= 0.05)\n');
    end
else
    fprintf('success_rate_results.mat not found. Run Success_Rate.m first.\n');
end

fprintf('\n');

% ── Path Distance Rank-Sum Test ───────────────────────────────────────
if exist('path_distance_results.mat', 'file')
    pd = load('path_distance_results.mat');
    n_a = numel(pd.dist_astar_all);
    n_s = numel(pd.dist_sfm_all);
    fprintf('--- Path Distance (successful runs) ---\n');
    fprintf('  A*+SFM:   n=%d, mean=%.1f (SD=%.1f)\n', ...
        n_a, mean(pd.dist_astar_all), std(pd.dist_astar_all));
    fprintf('  SFM-only: n=%d, mean=%.1f (SD=%.1f)\n', ...
        n_s, mean(pd.dist_sfm_all),   std(pd.dist_sfm_all));
    fprintf('  Reduction: %.1f%%\n', ...
        (mean(pd.dist_sfm_all) - mean(pd.dist_astar_all)) / mean(pd.dist_sfm_all) * 100);
    fprintf('  Wilcoxon rank-sum test:\n');
    fprintf('    z = %.4f,  p = %.6f\n', ...
        pd.stats_ranksum.zval, pd.p_ranksum);
    if pd.p_ranksum < 0.001
        fprintf('    *** p < 0.001 (highly significant)\n');
    elseif pd.p_ranksum < 0.01
        fprintf('    ** p < 0.01\n');
    elseif pd.p_ranksum < 0.05
        fprintf('    * p < 0.05\n');
    else
        fprintf('    n.s. (p >= 0.05)\n');
    end
else
    fprintf('path_distance_results.mat not found. Run CSVTotalDistanceVsNumberTrials.m first.\n');
end

fprintf('\n');

% ── Reproducibility Note ─────────────────────────────────────────────
fprintf('--- Reproducibility ---\n');
if exist('success_rate_results.mat', 'file')
    sr = load('success_rate_results.mat');
    fprintf('  %d seeds saved for success-rate trials.\n', numel(sr.trial_seeds));
    fprintf('  To rerun trial i: rng(trial_seeds(i)), then Success()\n');
end
if exist('path_distance_results.mat', 'file')
    pd = load('path_distance_results.mat');
    fprintf('  %d seeds saved for distance trials.\n', numel(pd.trial_seeds));
end
fprintf('============================================================\n');
