% Run_CanonicalTtc.m
% MATLAB reference for the TTC arms on the canonical scenarios, so the Python
% port of the filter can be cross-validated on identical inputs -- the same
% basis on which the SFM engine itself was validated.

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

expDir = fullfile('runs','v2','export');
files = dir(fullfile(expDir, 'canonical_*.json'));
out = struct('name', {}, 'outcome', {}, 'steps', {}, 'pathLength', {}, ...
             'finalDist', {}, 'minPedGap', {}, 'ttcActiveFrac', {});

fprintf('%-12s %-9s %8s %8s %8s %9s %8s\n', 'case','outcome','steps', ...
        'len','finalD','minPed','ttc%');
for k = 1:numel(files)
    C = jsondecode(fileread(fullfile(expDir, files(k).name)));
    S = scenarioFromJson(C);
    arm = struct('planner','sfm','aPed',[],'ttc',true,'label','sfm-ttc');
    opts = struct('robotStart', C.robotStart(:), 'robotVel', C.robotVel(:), ...
                  'logTrajectory', true);
    R = RunTrial(S, arm, opts);
    writematrix(R.traj.', fullfile(expDir, sprintf('mttc_%s.csv', C.name)));
    fprintf('%-12s %-9s %8d %8.2f %8.2f %9.3f %8.1f\n', C.name, R.outcome, ...
            R.steps, R.pathLength, R.finalDistToGoal, R.minPedGap, ...
            100*R.ttcActiveFrac);
    out(end+1) = struct('name', C.name, 'outcome', R.outcome, 'steps', R.steps, ...
        'pathLength', R.pathLength, 'finalDist', R.finalDistToGoal, ...
        'minPedGap', R.minPedGap, 'ttcActiveFrac', R.ttcActiveFrac); %#ok<SAGROW>
end
writetable(struct2table(out), fullfile('runs','v2','canonical_ttc_matlab.csv'));
fprintf('\nWrote runs/v2/canonical_ttc_matlab.csv\n');

function S = scenarioFromJson(C)
    P = C.params;
    if isempty(P.aPedRobot), P.aPedRobot = []; end
    P.numPedestrians = size(to2xN(C.pedPos), 2);
    S.seed = 0; S.P = P; S.feasible = true; S.astarSeconds = 0;
    S.xObsList = {}; S.yObsList = {};
    obs = C.obstacles;
    if ~isempty(obs)
        if iscell(obs)
            for j = 1:numel(obs)
                S.xObsList{end+1} = obs{j}(:,1).'; S.yObsList{end+1} = obs{j}(:,2).';
            end
        elseif ndims(obs) == 3
            for j = 1:size(obs,1)
                o = squeeze(obs(j,:,:));
                S.xObsList{end+1} = o(:,1).'; S.yObsList{end+1} = o(:,2).';
            end
        else
            S.xObsList{end+1} = obs(:,1).'; S.yObsList{end+1} = obs(:,2).';
        end
    end
    S.pedX0 = to2xN(C.pedPos); S.pedV0 = to2xN(C.pedVel);
    S.pedGoal0 = to2xN(C.pedGoal); S.goalPool = to2xN(C.goalPool);
    S.pedGoalSeed = C.pedGoalSeed; S.path = [];
end

function A = to2xN(a)
    if isempty(a), A = zeros(2, 0); return; end
    if size(a, 2) == 2
        A = a.';
    elseif size(a, 2) == 1 && size(a, 1) == 2
        A = a;
    else
        error('shape');
    end
end
