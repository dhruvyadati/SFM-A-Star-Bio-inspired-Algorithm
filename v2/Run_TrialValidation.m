% Run_TrialValidation.m
% MATLAB reference for the REAL trial configuration -- many pedestrians AND
% obstacles simultaneously, across all four factorial arms.
%
% The canonical fixtures carry 0, 1 or 5 pedestrians and at most one obstacle,
% so the combined path that every production run actually exercises has never
% been cross-validated. Previous bugs appeared exactly in such uncovered
% classes, so this is not optional before the 72-scenario result can be
% treated as anything but provisional.

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

expDir = fullfile('runs','v2','export');
files = dir(fullfile(expDir, 'trial_*.json'));
arms = { {'sfm', false}, {'sfm', true}, {'hybrid', false}, {'hybrid', true} };
names = {'sfm','sfm-ttc','hybrid','hybrid-ttc'};

out = struct('scenario',{},'arm',{},'outcome',{},'steps',{},'pathLength',{}, ...
             'finalDist',{},'minPedGap',{},'pedCollisions',{},'ttcActiveFrac',{});

fprintf('%-18s %-11s %-9s %7s %8s %9s %6s\n', 'scenario','arm','outcome', ...
        'steps','len','minPed','col');
for k = 1:numel(files)
    C = jsondecode(fileread(fullfile(expDir, files(k).name)));
    S = scenarioFromJson(C);
    for a = 1:numel(arms)
        arm = struct('planner', arms{a}{1}, 'aPed', [], 'ttc', arms{a}{2}, ...
                     'label', names{a});
        opts = struct('robotStart', C.robotStart(:), 'robotVel', C.robotVel(:));
        R = RunTrial(S, arm, opts);
        fprintf('%-18s %-11s %-9s %7d %8.2f %9.3f %6d\n', C.name, names{a}, ...
                R.outcome, R.steps, R.pathLength, R.minPedGap, R.pedCollisions);
        out(end+1) = struct('scenario', C.name, 'arm', names{a}, ...
            'outcome', R.outcome, 'steps', R.steps, 'pathLength', R.pathLength, ...
            'finalDist', R.finalDistToGoal, 'minPedGap', R.minPedGap, ...
            'pedCollisions', R.pedCollisions, 'ttcActiveFrac', R.ttcActiveFrac); %#ok<SAGROW>
    end
end
writetable(struct2table(out), fullfile('runs','v2','trial_matlab.csv'));
fprintf('\nWrote runs/v2/trial_matlab.csv (%d rows)\n', numel(out));

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
    S.pedGoalSeed = C.pedGoalSeed;
    S.path = to2xN(C.waypoints);
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
