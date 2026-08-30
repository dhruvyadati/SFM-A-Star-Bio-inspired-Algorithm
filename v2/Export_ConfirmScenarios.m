% Export_ConfirmScenarios.m
% Generates the 500 CONFIRMATORY scenarios from the frozen seed list.
%
% Seeds come from runs/v2/confirm_seeds.json, whose SHA-256 is recorded in the
% frozen protocol. They are read, never regenerated, so the scenarios cannot
% drift from the hashed set even if this file is edited.
%
% Runs ONCE. See resubmission/CONFIRMATORY_PROTOCOL.md.

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

outDir = fullfile('runs','v2','export_confirm');
if ~exist(outDir,'dir'); mkdir(outDir); end

S0 = jsondecode(fileread(fullfile('runs','v2','confirm_seeds.json')));

conds = {
  'baseline',  10, 150, 48, 1.34
  'dense',     10, 150, 80, 1.34
  'cluttered', 16, 150, 48, 1.34
  'bigObs',    10, 260, 48, 1.34
};

n = 0; nInf = 0;
for ci = 1:size(conds,1)
    name = conds{ci,1};
    P = SfmParams('numObstacles', conds{ci,2}, 'maxArea', conds{ci,3}, ...
                  'numPedestrians', conds{ci,4}, 'v0Ped', conds{ci,5});
    seeds = S0.(name);
    for k = 1:numel(seeds)
        s = seeds(k);
        S = MakeScenario(s, P);
        if ~S.feasible
            nInf = nInf + 1;
            continue;
        end
        C.name = sprintf('%s_%d', name, s);
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
        C.condition = name;
        C.matlabOutcome = '';
        fid = fopen(fullfile(outDir, sprintf('conf_%s.json', C.name)), 'w');
        fwrite(fid, jsonencode(C, 'PrettyPrint', true), 'char');
        fclose(fid);
        n = n + 1;
    end
    fprintf('%-10s %3d scenarios\n', name, numel(seeds));
end
fprintf('\nwrote %d confirmatory scenarios, %d infeasible maps rejected\n', n, nInf);
