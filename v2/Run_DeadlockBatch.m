% Run_DeadlockBatch.m
% Multi-condition SFM-only batch, run to accumulate deadlocks for the
% diagnostic study.
%
% Deliberately spread across conditions rather than run to a target failure
% count in one regime. Fifty deadlocks all drawn from a single obstacle
% density would tell us the diagnostic works on that density, and nothing
% about whether it captures a general equilibrium mechanism.
%
% Conditions vary obstacle count, obstacle size, crowd density and pedestrian
% speed. Each condition also becomes a natural transfer split: calibrate on
% some, test on others, so the result cannot be an artefact of one generator
% setting.
%
% Every trial records its deadlock CAUSE (static / pedestrian / mixed /
% controller / other), because the static field represents static structure
% only and its claims must be scoped to the class it covers.
%
% Usage:  cd to the repository root, then  run('v2/Run_DeadlockBatch.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

if ~exist('perCond','var');  perCond  = 15;       end
if ~exist('BATCH_SEED','var'); BATCH_SEED = 20260729; end
if ~exist('armName','var');  armName  = 'sfm';    end

outDir = fullfile('runs','v2'); if ~exist(outDir,'dir'); mkdir(outDir); end

% name, numObstacles, maxArea, numPedestrians, v0Ped
conds = {
  'baseline',   10, 150, 48, 1.34
  'sparse',      6, 150, 48, 1.34
  'cluttered',  14, 150, 48, 1.34
  'bigObs',     10, 250, 48, 1.34
  'lowDens',    10, 150, 24, 1.34
  'highDens',   10, 150, 72, 1.34
  'fastPed',    10, 150, 48, 1.70
};
nC = size(conds, 1);

rng(BATCH_SEED, 'twister');
allSeeds = randi([1, 2^31-1], nC * perCond, 1);

rows = struct([]);
nInf = 0; row = 0;
fprintf('=== Deadlock batch: %d conditions x %d trials = %d ===\n\n', ...
        nC, perCond, nC * perCond);
tAll = tic;

for ci = 1:nC
    P = SfmParams('numObstacles', conds{ci,2}, 'maxArea', conds{ci,3}, ...
                  'numPedestrians', conds{ci,4}, 'v0Ped', conds{ci,5});
    nD = 0;
    for k = 1:perCond
        row = row + 1;
        S = MakeScenario(allSeeds(row), P);
        if ~S.feasible, nInf = nInf + 1; continue; end
        R = RunTrial(S, armName);
        R.condition = conds{ci,1};
        R.condObs = conds{ci,2}; R.condArea = conds{ci,3};
        R.condPed = conds{ci,4}; R.condSpeed = conds{ci,5};
        R.trial = row;
        if isempty(rows), rows = R; else, rows(end+1) = R; end %#ok<SAGROW>
        if strcmp(R.outcome, 'deadlock'), nD = nD + 1; end
    end
    fprintf('%-10s obs=%2d area=%3d ped=%2d v0=%.2f -> %2d/%d deadlocks\n', ...
            conds{ci,1}, conds{ci,2}, conds{ci,3}, conds{ci,4}, conds{ci,5}, nD, perCond);
end

fprintf('\nWall clock: %.1f s   infeasible maps: %d\n', toc(tAll), nInf);

T = struct2table(rmfield_safe(rows, {'onset'}));
writetable(T, fullfile(outDir,'deadlock_batch.csv'));
save(fullfile(outDir,'deadlock_batch.mat'), 'rows', 'T', 'conds', 'BATCH_SEED', 'armName');

%% ---- Summary ---------------------------------------------------------
fprintf('\n============================================================\n');
fprintf('OUTCOMES BY CONDITION\n');
fprintf('============================================================\n');
fprintf('%-10s %8s %8s %8s %8s\n', 'condition', 'success', 'deadlock', 'timeout', 'safe');
for ci = 1:nC
    sub = T(strcmp(T.condition, conds{ci,1}), :);
    if isempty(sub), continue; end
    fprintf('%-10s %7.0f%% %7.0f%% %7.0f%% %7.0f%%\n', conds{ci,1}, ...
        100*mean(strcmp(sub.outcome,'success')), ...
        100*mean(strcmp(sub.outcome,'deadlock')), ...
        100*mean(strcmp(sub.outcome,'timeout')), ...
        100*mean(sub.safeSuccess));
end

fprintf('\n--- Deadlock cause mix (scopes what the static field can claim) ---\n');
dl = T(strcmp(T.outcome,'deadlock'), :);
fprintf('total deadlocks: %d\n', height(dl));
for cause = {'static','pedestrian','mixed','controller','other'}
    n = sum(strcmp(dl.deadlockCause, cause{1}));
    if height(dl) > 0
        fprintf('  %-11s %3d  (%.0f%%)\n', cause{1}, n, 100*n/height(dl));
    end
end

fprintf('\n--- Usable positives for the horizon study ---\n');
fprintf('deadlocks with recorded onset: %d\n', sum(isfinite(dl.deadlockOnsetStep)));
succ = T(strcmp(T.outcome,'success'), :);
fprintf('successes with a usable hard negative: %d\n', sum(isfinite(succ.minSpeed)));
fprintf('\nWrote %s\n', fullfile(outDir,'deadlock_batch.csv'));

% struct2table cannot handle a nested struct field; drop it for the flat CSV
% while keeping the full structs in the .mat for the onset analysis.
function s = rmfield_safe(s, flds)
    for k = 1:numel(flds)
        if isfield(s, flds{k}), s = rmfield(s, flds{k}); end
    end
end
