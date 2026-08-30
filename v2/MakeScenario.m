function S = MakeScenario(seed, P)
% MakeScenario builds one fully-specified, reproducible trial scenario from a
% single seed: obstacle layout, pedestrian initial states, pedestrian goals,
% and the one-shot A* plan.
%
% The scenario is built ONCE and then handed to each navigation method in a
% SEPARATE simulation. This is the central protocol fix. In the legacy model, the hybrid
% robot and the SFM-only robot occupied the same simulation as agents 1 and 2
% of a 50-agent population, and Force_O_Dyn looped over all pairs, so each
% robot exerted force on the other and on every pedestrian. The two arms were
% therefore not independent, and the claim of "identical conditions" did not
% hold. Running them separately from a shared seed gives genuinely identical
% obstacles, identical crowd initial conditions, and identical pedestrian
% goal sequences, while removing the cross-arm coupling.
%
% Returns S with fields:
%   .seed, .P
%   .xObsList, .yObsList     obstacle polygons
%   .pedX0 (2 x nPed)        pedestrian initial positions
%   .pedV0 (2 x nPed)        pedestrian initial velocities
%   .pedGoalStream           struct with a private RNG substream so that
%                            pedestrian goal RESAMPLING is identical across
%                            arms regardless of how long each arm runs
%   .path (2 x nWay)         pruned A* waypoints, [] if infeasible
%   .feasible                logical
%   .astarSeconds            wall-clock A* planning time

rng(seed, 'twister');
S.seed = seed;
S.P = P;

bounds = [P.xMin, P.xMax, P.yMin, P.yMax];

%% ---- Obstacles -------------------------------------------------------
if isfield(P, 'fixedVertexCount') && ~isempty(P.fixedVertexCount)
    [S.xObsList, S.yObsList] = GenerateEnvironmentNumberVertices( ...
        P.xMax, P.yMax, P.numObstacles, P.maxArea, bounds, P.fixedVertexCount);
else
    [S.xObsList, S.yObsList] = GenerateEnvironmentGraphObject( ...
        P.xMax, P.yMax, P.numObstacles, P.maxArea, bounds);
end

%% ---- Pedestrian initial states and goals -----------------------------
nPed = P.numPedestrians;
S.pedX0    = zeros(2, nPed);
S.pedV0    = zeros(2, nPed);
S.pedGoal0 = zeros(2, nPed);

for i = 1:nPed
    S.pedX0(:, i)    = sampleFree(P, S.xObsList, S.yObsList);
    S.pedV0(:, i)    = randn(2, 1);
    S.pedGoal0(:, i) = sampleFree(P, S.xObsList, S.yObsList);
end

% Pedestrians resample a fresh goal on arrival rather than holding one goal
% for the whole trial. In the legacy model, each pedestrian was given a single goal and,
% because speed was clamped up to vMin = 1 m/s, could never stop on reaching
% it -- so the "crowd" degenerated into 48 agents jittering around fixed
% points. A dedicated substream keeps the resampling sequence identical
% across arms even though the arms consume randomness at different rates.
S.pedGoalSeed = randi([1, 2^31-1]);

% Pre-generate the resampling sequence as an EXPLICIT LIST rather than
% drawing from a stream mid-simulation. Two reasons. First, it makes the
% pedestrian goal sequence a piece of scenario data that can be exported and
% replayed by another implementation, so a cross-language comparison does not
% depend on reproducing MATLAB's MT19937-to-double mapping. Second, it makes
% the sequence identical across arms regardless of how many draws each arm
% happens to consume.
poolStream = RandStream('twister', 'Seed', S.pedGoalSeed);
nPool = 5000;
S.goalPool = zeros(2, nPool);
for gi = 1:nPool
    S.goalPool(:, gi) = sampleFreeStream_(poolStream, P, S.xObsList, S.yObsList);
end

%% ---- One-shot A* plan ------------------------------------------------
% Graph construction and A* search are timed SEPARATELY. Building the pruned
% graph is one-off map preprocessing that a deployed robot would do once for
% a known map; the A* query is the per-navigation planning cost. Conflating
% them badly misstates the planning cost.
tGraph = tic;
[G, nodeIDs, ~, dmin] = GraphFast(P.xMax, P.yMax, S.xObsList, S.yObsList, P.graphMargin);
S.graphSeconds = toc(tGraph);
S.dmin = dmin;

tPlan = tic;
rawPath = AStarFast(G, nodeIDs, P.start, P.goal);
S.astarSeconds = toc(tPlan);

if isempty(rawPath)
    % Recorded, never silently dropped. The legacy code scored
    % infeasible maps as a failure for both arms without reporting how often
    % that happened, which biases the comparison.
    S.feasible = false;
    S.path = [];
    return;
end

S.feasible = true;
S.path = removeColinearNodes(rawPath)';   % 2 x nWay
end

% ---------------------------------------------------------------------
function pt = sampleFree(P, xObsList, yObsList)
% Rejection-sample a point outside every obstacle plus the safety margin.
    for attempt = 1:10000
        xr = (P.xMax - P.xMin) * rand() + P.xMin;
        yr = (P.yMax - P.yMin) * rand() + P.yMin;
        if ~ptInObs(xr, yr, xObsList, yObsList, P.graphMargin)
            pt = [xr; yr];
            return;
        end
    end
    error('MakeScenario:noFreeSpace', ...
          'Could not sample a free point after 10000 attempts.');
end

% ---------------------------------------------------------------------
function pt = sampleFreeStream_(stream, P, xObsList, yObsList)
    for attempt = 1:10000
        xr = (P.xMax - P.xMin) * rand(stream) + P.xMin;
        yr = (P.yMax - P.yMin) * rand(stream) + P.yMin;
        if ~ptInObs(xr, yr, xObsList, yObsList, P.graphMargin)
            pt = [xr; yr];
            return;
        end
    end
    error('MakeScenario:noFreeSpace', 'Could not sample a goal for the pool.');
end
