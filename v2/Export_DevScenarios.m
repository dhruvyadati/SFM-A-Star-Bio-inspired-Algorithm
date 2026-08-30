% Export_DevScenarios.m
% Development scenario set for ORCA tuning. DISJOINT from every seed used in
% the deadlock batch or the safety-gate factorial, so tuning cannot touch data
% that will later carry a confirmatory result.
%
% Spans the conditions the screen has to cover: guided and unguided routes
% (waypoints are exported so both are runnable from one file), a dense crowd,
% a cluttered multi-obstacle case, and large obstacles that force tight
% passages.

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

outDir = fullfile('runs','v2','export_dev');
if ~exist(outDir,'dir'); mkdir(outDir); end
if ~exist('perCond','var'); perCond = 4; end

% DEV_SEED is deliberately unrelated to BATCH_SEED (20260729) and the
% safety-gate development seed (19730401).
%
% NESTED CHECKPOINTS. Each condition draws from its OWN stream seeded by
% base + condition index, rather than all conditions sharing one stream. With
% a shared stream the draws a condition receives depend on perCond, so the
% "first 18" of a 75-per-condition export would differ from an 18-per-condition
% export and the stability table would compare different scenarios rather than
% a growing sample of the same ones.
DEV_BASE = 77712345;

conds = {
  'baseline',  10, 150, 48, 1.34
  'dense',     10, 150, 80, 1.34
  'cluttered', 16, 150, 48, 1.34
  'bigObs',    10, 260, 48, 1.34
};

n = 0;
for ci = 1:size(conds,1)
    P = SfmParams('numObstacles', conds{ci,2}, 'maxArea', conds{ci,3}, ...
                  'numPedestrians', conds{ci,4}, 'v0Ped', conds{ci,5});
    rng(DEV_BASE + ci, 'twister');
    for k = 1:perCond
        s = randi([1, 2^31-1]);
        S = MakeScenario(s, P);
        if ~S.feasible, continue; end
        C.name = sprintf('%s_%d', conds{ci,1}, s);
        C.params = P;
        C.obstacles = cell(1, numel(S.xObsList));
        for j = 1:numel(S.xObsList)
            C.obstacles{j} = [S.xObsList{j}(:), S.yObsList{j}(:)];
        end
        C.robotStart = P.start(:).';
        C.robotVel   = [0 0];
        C.pedPos  = S.pedX0.';
        C.pedVel  = S.pedV0.';
        C.pedGoal = S.pedGoal0.';
        C.pedGoalSeed = S.pedGoalSeed;
        C.goalPool = S.goalPool.';
        C.waypoints = S.path.';
        C.condition = conds{ci,1};
        C.matlabOutcome = '';
        fid = fopen(fullfile(outDir, sprintf('dev_%s.json', C.name)), 'w');
        fwrite(fid, jsonencode(C, 'PrettyPrint', true), 'char');
        fclose(fid);
        n = n + 1;
    end
    fprintf('%-10s obs=%2d area=%3d ped=%2d\n', conds{ci,1}, conds{ci,2}, ...
            conds{ci,3}, conds{ci,4});
end
fprintf('\nwrote %d development scenarios to %s\n', n, outDir);
