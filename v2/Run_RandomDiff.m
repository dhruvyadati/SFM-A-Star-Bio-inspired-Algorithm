% Run_RandomDiff.m
% MATLAB side of the randomised differential test. Hand fixtures cover the
% edge cases someone thought of; these cover interactions nobody anticipated.
addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

d = fullfile('runs','v2','export_rand');
files = dir(fullfile(d, 'rand_*.json'));
out = struct('scenario',{},'arm',{},'outcome',{},'steps',{},'pathLength',{}, ...
             'finalDist',{},'minPedGap',{},'minObsGap',{});
arms = { {'sfm',false}, {'sfm',true} };
names = {'sfm','sfm-ttc'};

for k = 1:numel(files)
    C = jsondecode(fileread(fullfile(d, files(k).name)));
    S = scenarioFromJson(C);
    for a = 1:2
        arm = struct('planner',arms{a}{1},'aPed',[],'ttc',arms{a}{2},'label',names{a});
        opts = struct('robotStart', C.robotStart(:), 'robotVel', C.robotVel(:));
        R = RunTrial(S, arm, opts);
        out(end+1) = struct('scenario', C.name, 'arm', names{a}, ...
            'outcome', R.outcome, 'steps', R.steps, 'pathLength', R.pathLength, ...
            'finalDist', R.finalDistToGoal, 'minPedGap', R.minPedGap, ...
            'minObsGap', R.minObsGap); %#ok<SAGROW>
    end
    fprintf('%s (%d peds, %d obs)\n', C.name, size(to2xN(C.pedPos),2), ...
            numel(S.xObsList));
end
writetable(struct2table(out), fullfile('runs','v2','random_matlab.csv'));
fprintf('\nWrote %d rows\n', numel(out));

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
    S.pedGoalSeed = 0; S.path = [];
end

function A = to2xN(a)
    if isempty(a), A = zeros(2, 0); return; end
    if size(a,2) == 2, A = a.';
    elseif size(a,2) == 1 && size(a,1) == 2, A = a;
    else, error('shape'); end
end
