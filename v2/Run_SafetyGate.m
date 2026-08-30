% Run_SafetyGate.m
% Development sweep over the safety arms, on matched seeds.
%
% Question: is collision reduction reachable by social-force calibration
% alone, or is an explicit separation constraint required?
%
% Run as a 2x4 factorial so the planner effect and the safety effect can be
% separated, and their interaction estimated.
%
% Primary outcome is SAFE SUCCESS: goal reached AND no contact at any point.
% The filter duty cycle is reported too: a filter that engages nearly always
% is not an emergency mechanism, it is the controller.
%
% These are DEVELOPMENT seeds. Parameters may be tuned against these results;
% the evaluation run must then use a disjoint seed set.
%
% Usage:  cd to the repository root, then  run('v2/Run_SafetyGate.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

if ~exist('numTrials','var');   numTrials   = 20;       end
if ~exist('DEV_SEED','var');    DEV_SEED    = 19730401; end   % development set

outDir = fullfile('runs','v2'); if ~exist(outDir,'dir'); mkdir(outDir); end

P = SfmParams();
rng(DEV_SEED, 'twister');
seeds = randi([1, 2^31-1], numTrials, 1);

% FULL 2x4 FACTORIAL: planner {sfm, hybrid} x safety {none, aPed5, ttc, both}.
%
% A ladder of hybrid-only safety variants against an unfiltered SFM baseline
% would confound two separate changes. If the final method carries a safety
% filter, its honest baseline is the reactive controller carrying the SAME
% filter -- otherwise "A* helps" and "the safety layer helps" cannot be told
% apart. The factorial also yields the interaction term directly, which is
% what deciding between "one mechanism suffices" and "the two cover
% complementary regimes" actually requires.
arms = {
  struct('planner','sfm',    'aPed',[],  'ttc',false, 'label','sfm')
  struct('planner','sfm',    'aPed',5.0, 'ttc',false, 'label','sfm-aPed5')
  struct('planner','sfm',    'aPed',[],  'ttc',true,  'label','sfm-ttc')
  struct('planner','sfm',    'aPed',5.0, 'ttc',true,  'label','sfm-both')
  struct('planner','hybrid', 'aPed',[],  'ttc',false, 'label','hybrid')
  struct('planner','hybrid', 'aPed',5.0, 'ttc',false, 'label','hybrid-aPed5')
  struct('planner','hybrid', 'aPed',[],  'ttc',true,  'label','hybrid-ttc')
  struct('planner','hybrid', 'aPed',5.0, 'ttc',true,  'label','hybrid-both')
};

rows = struct([]);
nInf = 0;
fprintf('=== Safety gate, %d matched development trials ===\n\n', numTrials);
tAll = tic;

for k = 1:numTrials
    S = MakeScenario(seeds(k), P);
    if ~S.feasible, nInf = nInf + 1; continue; end
    line = sprintf('trial %3d/%d ', k, numTrials);
    for a = 1:numel(arms)
        R = RunTrial(S, arms{a});
        R.trial = k;
        if isempty(rows), rows = R; else, rows(end+1) = R; end %#ok<SAGROW>
        line = [line sprintf('|%s %s/%s', arms{a}.label(1:min(9,end)), ...
                 R.outcome(1:min(4,end)), tf(R.collisionFree,'CF','X'))]; %#ok<AGROW>
    end
    fprintf('%s\n', line);
end

fprintf('\nWall clock: %.1f s   infeasible maps: %d\n\n', toc(tAll), nInf);

T = struct2table(rows);
writetable(T, fullfile(outDir,'safety_gate.csv'));
save(fullfile(outDir,'safety_gate.mat'), 'T','P','seeds','DEV_SEED');

fprintf('%-14s %6s %6s %6s %7s %8s %8s %8s %7s %7s\n', 'arm', ...
        'goal%','safe%','CF%','deadl%','contact_s','maxPen','longest','ttc%','rmsJerk');
fprintf('%s\n', repmat('-', 1, 96));
for a = 1:numel(arms)
    sub = T(strcmp(T.method, arms{a}.label), :);
    if isempty(sub), continue; end
    fprintf('%-14s %6.0f %6.0f %6.0f %7.0f %8.2f %8.4f %8.2f %7.1f %7.2f\n', ...
        arms{a}.label, ...
        100*mean(sub.success), 100*mean(sub.safeSuccess), ...
        100*mean(sub.collisionFree), 100*mean(strcmp(sub.outcome,'deadlock')), ...
        mean(sub.pedContactTime), mean(sub.maxPedPen), mean(sub.longestContact), ...
        100*mean(sub.ttcActiveFrac), mean(sub.rmsJerk));
end

fprintf('\nSecondary:\n');
fprintf('%-14s %8s %8s %9s %8s %9s %9s\n', 'arm', ...
        'time_s','path_m','peds_hit', 'pers%', 'ttc_acts', 'brakeScale');
for a = 1:numel(arms)
    sub = T(strcmp(T.method, arms{a}.label), :);
    if isempty(sub), continue; end
    fprintf('%-14s %8.1f %8.1f %9.2f %8.1f %9.1f %9.3f\n', arms{a}.label, ...
        mean(sub.simSeconds), mean(sub.pathLength), mean(sub.pedsContacted), ...
        100*mean(sub.personalFrac), mean(sub.ttcActivations), mean(sub.meanBrakeScale));
end

fprintf('\nTarget before proceeding to trap validation: safe success >= 85%%.\n');

function s = tf(c,a,b), if c, s=a; else, s=b; end, end
