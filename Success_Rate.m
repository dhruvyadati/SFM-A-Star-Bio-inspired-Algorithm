% Success_Rate.m
% Runs N trials comparing A*+SFM vs SFM-only success rates.
% Saves the random seed for each trial so results are exactly reproducible.
% Reports a two-proportion z-test for statistical significance.

numTrials = 100;

Success_Counts = zeros(numTrials, 2);  % col 1 = SFM-only, col 2 = A*+SFM

% Draw one distinct seed per trial from a single entropy-seeded master stream.
% Saving these makes each trial exactly reproducible via rng(trial_seeds(i)).
% NOTE: do NOT use `s = rng('shuffle'); rng(s.Seed)` — rng() returns the
% PREVIOUS settings, whose Seed stays 0, which would make every trial identical.
rng('shuffle');
trial_seeds = randi([1, 2^31-1], numTrials, 1);

for i = 1:numTrials
    rng(trial_seeds(i), 'twister');   % reproducible, distinct per-trial seed

    [sfm_success, astar_success] = Success();
    Success_Counts(i, 1) = sfm_success;    % SFM-only
    Success_Counts(i, 2) = astar_success;  % A*+SFM
end

% ── Summary ──────────────────────────────────────────────────────────
n_sfm   = sum(Success_Counts(:,1));
n_astar = sum(Success_Counts(:,2));
p_sfm   = n_sfm   / numTrials;
p_astar = n_astar / numTrials;

fprintf('\n=== Success Rate Results ===\n');
fprintf('SFM-only:  %d / %d = %.1f%%\n', n_sfm,   numTrials, p_sfm*100);
fprintf('A*+SFM:    %d / %d = %.1f%%\n', n_astar, numTrials, p_astar*100);

% ── 95% Confidence Intervals ─────────────────────────────────────────
SE = [sqrt(p_sfm*(1-p_sfm)/numTrials), sqrt(p_astar*(1-p_astar)/numTrials)];
ME = 1.96 * SE;
fprintf('\n95%% CI SFM-only:  [%.3f, %.3f]\n', p_sfm  -ME(1), p_sfm  +ME(1));
fprintf('95%% CI A*+SFM:    [%.3f, %.3f]\n', p_astar-ME(2), p_astar+ME(2));

% ── Two-Proportion Z-Test (H0: p_sfm == p_astar, two-tailed) ─────────
p_pool  = (n_sfm + n_astar) / (2 * numTrials);
se_pool = sqrt(p_pool * (1 - p_pool) * (1/numTrials + 1/numTrials));
z_stat  = (p_astar - p_sfm) / se_pool;
% Two-tailed p-value from the standard normal using base-MATLAB erfc
% (avoids the Statistics Toolbox normcdf): 2*(1-Phi(|z|)) = erfc(|z|/sqrt(2)).
p_value = erfc(abs(z_stat) / sqrt(2));

fprintf('\n=== Two-Proportion Z-Test ===\n');
fprintf('Z-statistic: %.4f\n', z_stat);
fprintf('p-value:     %.6f\n', p_value);
if p_value < 0.05
    fprintf('Result: STATISTICALLY SIGNIFICANT (p < 0.05)\n');
else
    fprintf('Result: not significant at alpha = 0.05\n');
end

% ── Save seeds and results ────────────────────────────────────────────
% Full results (bulky) go to a .mat that is NOT tracked in git.
save('success_rate_results.mat', ...
    'Success_Counts', 'trial_seeds', 'numTrials', ...
    'p_sfm', 'p_astar', 'z_stat', 'p_value');

% Small, human-readable per-trial seed log goes to runs/ and IS tracked in
% git — this is the reproducibility record. rng(seed) regenerates the exact
% obstacles and pedestrians for that trial.
if ~exist('runs', 'dir'); mkdir('runs'); end
seedLog = table((1:numTrials)', trial_seeds, ...
    Success_Counts(:,1), Success_Counts(:,2), ...
    'VariableNames', {'trial', 'seed', 'SFM_success', 'AStarSFM_success'});
writetable(seedLog, fullfile('runs', 'success_rate_seeds.csv'));

fprintf('\nSeeds saved to success_rate_results.mat and runs/success_rate_seeds.csv\n');
fprintf('To reproduce trial i exactly: rng(trial_seeds(i)), then call Success()\n');

% ── Plot ─────────────────────────────────────────────────────────────
figure; hold on;
bar(["SFM", "SFM + A*"], [p_sfm, p_astar]);
h = errorbar([1 2], [p_sfm, p_astar], ME, 'o');
h.LineStyle = 'none';
ylabel('Percentage of Trials That Reached the Goal');
title('Success Rate of SFM Only vs. A* + SFM');
