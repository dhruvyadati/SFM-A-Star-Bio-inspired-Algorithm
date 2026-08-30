% Run_Interaction.m
% Matched-scenario interaction contrasts on the safety-gate factorial.
%
% Estimand, per matched scenario:
%     delta = [p(G,T) - p(G,0)] - [p(0,T) - p(0,0)]
% so scenario difficulty cancels. Bootstrap CI over scenarios and a sign-flip
% permutation p-value; no asymptotic assumptions.
%
% Development-set sizes cannot support a confirmatory claim. This quantifies
% direction and magnitude to decide what to test at proper n.
%
% Usage:  cd to the repository root, then  run('v2/Run_Interaction.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

L = load(fullfile('runs','v2','safety_gate.mat'));
T = L.T;
% Local helper: pull one arm's column, ordered by scenario so rows align.
arm = @(label, field) getfield(sortrows(T(strcmp(T.method,label),:), 'trial'), field);

fprintf('=== Matched interaction contrasts (n = %d scenarios) ===\n\n', ...
        numel(arm('sfm','safeSuccess')));

pairs = {
  'guidance x TTC filter',      'hybrid-ttc',   'hybrid',       'sfm-ttc',   'sfm'
  'guidance x robot gain',      'hybrid-aPed5', 'hybrid',       'sfm-aPed5', 'sfm'
  'gain x TTC (under guidance)','hybrid-both',  'hybrid-aPed5', 'hybrid-ttc','hybrid'
};

for k = 1:size(pairs,1)
    name = pairs{k,1};
    a = arm(pairs{k,2}, 'safeSuccess');
    b = arm(pairs{k,3}, 'safeSuccess');
    c = arm(pairs{k,4}, 'safeSuccess');
    d = arm(pairs{k,5}, 'safeSuccess');

    [delta, lo, hi, p] = StatsLib.interaction(a, b, c, d);
    fprintf('%-28s delta = %+0.3f  95%% CI [%+0.3f, %+0.3f]  perm p = %.4f\n', ...
            name, delta, lo, hi, p);
    fprintf('%-28s   simple effects: %+0.3f with, %+0.3f without\n\n', '', ...
            mean(a) - mean(b), mean(c) - mean(d));
end

%% ---- Main effects, matched ------------------------------------------
fprintf('--- Matched main effects on safe success ---\n');
me = {
  'guidance (no safety layer)', 'hybrid',     'sfm'
  'TTC filter (with guidance)', 'hybrid-ttc', 'hybrid'
  'TTC filter (no guidance)',   'sfm-ttc',    'sfm'
  'robot gain (with guidance)', 'hybrid-aPed5','hybrid'
  'gain on top of TTC',         'hybrid-both','hybrid-ttc'
};
for k = 1:size(me,1)
    a = arm(me{k,2}, 'safeSuccess');
    b = arm(me{k,3}, 'safeSuccess');
    dd = a - b;
    [loB, hiB] = StatsLib.bootCI(dd);
    fprintf('%-28s %+0.3f  95%% CI [%+0.3f, %+0.3f]\n', me{k,1}, mean(dd), loB, hiB);
end

%% ---- Deadlock, the cost side ----------------------------------------
fprintf('\n--- Deadlock rate (safety bought by refusing to move) ---\n');
for lbl = {'sfm','sfm-ttc','sfm-both','hybrid','hybrid-ttc','hybrid-both'}
    sub = T(strcmp(T.method, lbl{1}), :);
    fprintf('  %-14s %5.0f%%\n', lbl{1}, 100*mean(strcmp(sub.outcome,'deadlock')));
end
