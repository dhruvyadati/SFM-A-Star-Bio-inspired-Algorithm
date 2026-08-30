% Export_Scenarios.m
% Writes canonical scenarios to a language-neutral JSON format so the Python
% port can be validated against MATLAB on identical inputs.
%
% Everything needed to reproduce a trial is exported: obstacle geometry,
% pedestrian initial states and goals, the pedestrian goal-resampling seed,
% the A* waypoint sequence, and the full parameter set. Nothing is left to be
% re-derived on the Python side, because a re-derivation that differs
% silently would be indistinguishable from a dynamics discrepancy.
%
% ARRAY LAYOUT: every array in the file is (n items) x 2, i.e. one row per
% agent/vertex/waypoint with [x, y] columns. MATLAB works in 2 x n
% internally, so the exporter transposes on the way out.
%
% Two scenario families are written:
%   canonical/  small deterministic cases for unit-level agreement
%   trial/      real trial scenarios from the deadlock batch for outcome-level
%               agreement
%
% Usage:  cd to the repository root, then  run('v2/Export_Scenarios.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

outDir = fullfile('runs','v2','export');
if ~exist(outDir,'dir'); mkdir(outDir); end

if ~exist('nTrial','var'); nTrial = 8; end

%% ---- Canonical cases -------------------------------------------------
% Deterministic, hand-specified, no RNG. These isolate the dynamics from the
% scenario generator so a disagreement can be localised.
P = SfmParams();
canon = {};

% 1. Single agent, no obstacles, straight run to goal.
canon{end+1} = mkCase('free_run', P, {}, {}, [5;5], [0;0], zeros(2,0), zeros(2,0), zeros(2,0));

% 2. Single agent decelerating into the goal.
canon{end+1} = mkCase('arrive', P, {}, {}, [46;46], [1.0;1.0], zeros(2,0), zeros(2,0), zeros(2,0));

% 3. Head-on with one stationary pedestrian on the goal line.
canon{end+1} = mkCase('head_on', P, {}, {}, [10;10], [0;0], [20;20], [0;0], [20;20]);

% 4. Crossing pedestrian.
canon{end+1} = mkCase('crossing', P, {}, {}, [10;10], [0.9;0.9], [20;10], [0;1.2], [20;40]);

% 5. Multi-pedestrian crowd. The other canonical cases carry 0 or 1
% pedestrian, so array-valued per-pedestrian quantities were never exercised
% -- a gap that let an array/scalar bug in the TTC filter reach a 288-run
% batch before surfacing.
pedX = [14 18 22 16 20; 16 14 18 22 20];
pedV = [0.3 -0.3 0.2 -0.2 0.0; -0.2 0.3 -0.3 0.2 0.1];
pedG = [30 10 34 12 28; 32 30 10 34 12];
canon{end+1} = mkCase('crowd5', P, {}, {}, [10;10], [0.5;0.5], pedX, pedV, pedG);

% 6. Single square obstacle directly on the path.
sqx = {[20 30 30 20]}; sqy = {[20 20 30 30]};
canon{end+1} = mkCase('obstacle', P, sqx, sqy, [10;10], [0.9;0.9], zeros(2,0), zeros(2,0), zeros(2,0));

% 7. L-corner trap, the geometry used by the capture sanity test.
Lx = {[20 30 30 32 32 20]}; Ly = {[27 27 20 20 30 30]};
canon{end+1} = mkCase('corner', P, Lx, Ly, [28;25], [0;0], zeros(2,0), zeros(2,0), zeros(2,0));

for k = 1:numel(canon)
    writeJson(fullfile(outDir, sprintf('canonical_%s.json', canon{k}.name)), canon{k});
end
fprintf('wrote %d canonical scenarios\n', numel(canon));

%% ---- Real trial scenarios --------------------------------------------
bf = fullfile('runs','v2','deadlock_batch.mat');
if exist(bf, 'file')
    L = load(bf);
    rows = L.rows;
    n = min(nTrial, numel(rows));
    for k = 1:n
        r = rows(k);
        Pk = SfmParams('numObstacles', r.condObs, 'maxArea', r.condArea, ...
                       'numPedestrians', r.condPed, 'v0Ped', r.condSpeed);
        S = MakeScenario(r.seed, Pk);
        if ~S.feasible, continue; end
        C = mkCase(sprintf('seed%d', r.seed), Pk, S.xObsList, S.yObsList, ...
                   Pk.start(:), [0;0], S.pedX0, S.pedV0, S.pedGoal0);
        C.pedGoalSeed = S.pedGoalSeed;
        C.goalPool = S.goalPool.';
        % Transposed to nWay x 2 so EVERY array in the file is
        % (n items) x (2 coords). A mixed orientation is a silent-failure
        % mode for any consumer that assumes one layout.
        C.waypoints = S.path.';
        C.condition = r.condition;
        C.matlabOutcome = r.outcome;
        writeJson(fullfile(outDir, sprintf('trial_%s.json', C.name)), C);
    end
    fprintf('wrote %d trial scenarios\n', n);
else
    fprintf('deadlock_batch.mat not found; skipped trial scenarios\n');
end

fprintf('export dir: %s\n', outDir);

%% ---------------------------------------------------------------------
function C = mkCase(name, P, xo, yo, robX, robV, pedX, pedV, pedG)
    C.name = name;
    C.params = P;
    C.obstacles = cell(1, numel(xo));
    for k = 1:numel(xo)
        C.obstacles{k} = [xo{k}(:), yo{k}(:)];
    end
    C.robotStart = robX(:).';
    C.robotVel   = robV(:).';
    C.pedPos  = pedX.';
    C.pedVel  = pedV.';
    C.pedGoal = pedG.';
    C.pedGoalSeed = 0;
    % Canonical cases get a FIXED cycle of corner goals rather than a sampled
    % pool, so pedestrian behaviour stays deterministic and hand-checkable.
    % An empty pool would abort the moment a pedestrian reached its goal.
    cyc = [12 12; 12 40; 40 40; 40 12];
    C.goalPool = repmat(cyc, 250, 1);
    C.waypoints = [];
    C.condition = 'canonical';
    C.matlabOutcome = '';
end

function writeJson(path, C)
    % jsonencode handles structs, cells and matrices. Written with explicit
    % pretty printing so the files are reviewable in a diff.
    txt = jsonencode(C, 'PrettyPrint', true);
    fid = fopen(path, 'w');
    fwrite(fid, txt, 'char');
    fclose(fid);
end
