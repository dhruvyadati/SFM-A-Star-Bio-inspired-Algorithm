% Run_CanonicalMatlab.m
% Runs the exported canonical scenarios through the MATLAB engine so its
% output can be compared against the Python port on identical inputs.
%
% The JSON is the single source of truth for both engines. Nothing is
% re-derived here -- a re-derivation that differed would be indistinguishable
% from a dynamics discrepancy, which is the whole point of the exchange format.

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

expDir = fullfile('runs','v2','export');
files = dir(fullfile(expDir, 'canonical_*.json'));
out = struct('name', {}, 'outcome', {}, 'steps', {}, 'pathLength', {}, ...
             'finalDist', {}, 'minPedGap', {}, 'minObsGap', {});

fprintf('%-12s %-9s %8s %8s %8s %9s %8s\n', 'case','outcome','steps', ...
        'len','finalD','minPed','minObs');
for k = 1:numel(files)
    C = jsondecode(fileread(fullfile(expDir, files(k).name)));
    S = scenarioFromJson(C);
    opts = struct('robotStart', C.robotStart(:), 'robotVel', C.robotVel(:));
    opts.logTrajectory = true;
    R = RunTrial(S, 'sfm', opts);
    % Trajectories, not just summaries: two engines can agree on outcome while
    % following materially different paths.
    writematrix(R.traj.', fullfile('runs','v2','export', ...
                sprintf('mtraj_%s.csv', C.name)));
    fprintf('%-12s %-9s %8d %8.2f %8.2f %9.3f %8.3f\n', C.name, R.outcome, ...
            R.steps, R.pathLength, R.finalDistToGoal, R.minPedGap, R.minObsGap);
    out(end+1) = struct('name', C.name, 'outcome', R.outcome, 'steps', R.steps, ...
        'pathLength', R.pathLength, 'finalDist', R.finalDistToGoal, ...
        'minPedGap', R.minPedGap, 'minObsGap', R.minObsGap); %#ok<SAGROW>
end

T = struct2table(out);
writetable(T, fullfile('runs','v2','canonical_matlab.csv'));
fprintf('\nWrote %s\n', fullfile('runs','v2','canonical_matlab.csv'));

function S = scenarioFromJson(C)
    P = C.params;
    if isempty(P.aPedRobot), P.aPedRobot = []; end
    P.numPedestrians = size(to2xN(C.pedPos), 2);
    S.seed = 0;
    S.P = P;
    S.feasible = true;
    S.astarSeconds = 0;
    % jsondecode returns a cell array only when the polygons have differing
    % vertex counts; with one obstacle, or several of equal size, it collapses
    % to a numeric array. Both shapes have to be handled.
    S.xObsList = {}; S.yObsList = {};
    obs = C.obstacles;
    if isempty(obs)
        % nothing to do
    elseif iscell(obs)
        for j = 1:numel(obs)
            o = obs{j};
            S.xObsList{end+1} = o(:,1).';
            S.yObsList{end+1} = o(:,2).';
        end
    elseif ndims(obs) == 3
        for j = 1:size(obs, 1)
            o = squeeze(obs(j,:,:));
            S.xObsList{end+1} = o(:,1).';
            S.yObsList{end+1} = o(:,2).';
        end
    else
        S.xObsList{end+1} = obs(:,1).';
        S.yObsList{end+1} = obs(:,2).';
    end
    S.pedX0    = to2xN(C.pedPos);
    S.pedV0    = to2xN(C.pedVel);
    S.pedGoal0 = to2xN(C.pedGoal);
    S.goalPool = to2xN(C.goalPool);
    S.pedGoalSeed = C.pedGoalSeed;
    S.path = [];
end

function A = to2xN(a)
% Normalise a decoded array to 2 x n.
%
% jsondecode returns a COLUMN vector for a flat JSON array, so a single
% [x, y] row arrives as 2x1 rather than 1x2. Transposing unconditionally
% inverts exactly that case, which silently swaps the meaning of the two
% dimensions for any single-agent scenario.
    if isempty(a), A = zeros(2, 0); return; end
    if size(a, 2) == 2
        A = a.';                    % n x 2  ->  2 x n
    elseif size(a, 2) == 1 && size(a, 1) == 2
        A = a;                      % flat [x, y] decoded as a 2x1 column
    else
        error('Run_CanonicalMatlab:shape', ...
              'Cannot interpret a %dx%d array as 2 x n.', size(a,1), size(a,2));
    end
end
