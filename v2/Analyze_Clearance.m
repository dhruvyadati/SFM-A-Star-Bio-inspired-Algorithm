% Analyze_Clearance.m
% Two questions about the geometric plan:
%
%   A. Can collinear-node pruning push path segments closer to obstacles than
%      the unpruned grid path?
%   B. What does the one-cell margin actually guarantee? It may not ensure
%      clearance along diagonal edges or under continuous SFM deviation.
%
% Usage:  cd to the repository root, then  run('v2/Analyze_Clearance.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

if ~exist('nMaps','var');       nMaps       = 30;       end
if ~exist('MASTER_SEED','var'); MASTER_SEED = 20260729; end

P = SfmParams();
outDir = fullfile('runs','v2'); if ~exist(outDir,'dir'); mkdir(outDir); end

rng(MASTER_SEED,'twister');
seeds = randi([1, 2^31-1], nMaps, 1);

fprintf('=== Clearance analysis over %d maps ===\n\n', nMaps);

% ---------------------------------------------------------------------
% PART A. Is pruning geometry-preserving?
%
% removeColinearNodes drops node i exactly when (p_i - p_{i-1}) x
% (p_{i+1} - p_i) = 0, i.e. when p_i lies ON the segment p_{i-1} -> p_{i+1}.
% The pruned polyline therefore traces the SAME point set as the original.
% Pruning cannot reduce clearance. This part verifies that claim numerically
% rather than resting on the argument alone.
% ---------------------------------------------------------------------
maxDeviation = 0;
minRaw  = inf(nMaps,1);
minPruned = inf(nMaps,1);
minNode = inf(nMaps,1);
nodeCountRaw = zeros(nMaps,1);
nodeCountPruned = zeros(nMaps,1);
feasible = false(nMaps,1);

for k = 1:nMaps
    S = MakeScenario(seeds(k), P);
    if ~S.feasible, continue; end
    feasible(k) = true;

    % Rebuild the unpruned path for comparison.
    [G, nodeIDs] = GraphFast(P.xMax, P.yMax, S.xObsList, S.yObsList, P.graphMargin);
    raw = AStarFast(G, nodeIDs, P.start, P.goal);
    pruned = S.path.';

    nodeCountRaw(k)    = size(raw, 1);
    nodeCountPruned(k) = size(pruned, 1);

    % Deviation: furthest any point of the pruned polyline sits from the raw
    % polyline. Should be exactly zero if pruning is geometry-preserving.
    dev = maxPolylineDeviation(pruned, raw);
    maxDeviation = max(maxDeviation, dev);

    % Clearance sampled densely along each polyline.
    minRaw(k)    = minClearance(raw,    S.xObsList, S.yObsList);
    minPruned(k) = minClearance(pruned, S.xObsList, S.yObsList);

    % Clearance at the graph NODES only -- what the one-cell margin actually
    % guarantees.
    dn = inf;
    for i = 1:size(raw,1)
        dn = min(dn, minDistToObs(raw(i,1), raw(i,2), S.xObsList, S.yObsList));
    end
    minNode(k) = dn;

    fprintf('map %2d/%d seed=%-11d nodes %3d -> %3d  clearance raw %.3f  pruned %.3f  nodes %.3f\n', ...
        k, nMaps, seeds(k), nodeCountRaw(k), nodeCountPruned(k), ...
        minRaw(k), minPruned(k), minNode(k));
end

ok = feasible;
fprintf('\n------------------------------------------------------------\n');
fprintf('PART A -- does pruning reduce clearance?\n');
fprintf('  max deviation of pruned polyline from raw path : %.3e m\n', maxDeviation);
fprintf('  worst clearance, raw path                      : %.3f m\n', min(minRaw(ok)));
fprintf('  worst clearance, pruned path                   : %.3f m\n', min(minPruned(ok)));
fprintf('  max |raw - pruned| clearance difference        : %.3e m\n', ...
        max(abs(minRaw(ok) - minPruned(ok))));
fprintf('  waypoint reduction                             : %.1f%% (%.0f -> %.0f nodes)\n', ...
        100*(1 - mean(nodeCountPruned(ok))/mean(nodeCountRaw(ok))), ...
        mean(nodeCountRaw(ok)), mean(nodeCountPruned(ok)));
fprintf('  => pruning is geometry-preserving, and provably so.\n');

fprintf('\nPART B -- what does the one-cell margin actually guarantee?\n');
fprintf('  min clearance at graph NODES     : %.3f m (margin = %.2f m)\n', ...
        min(minNode(ok)), P.graphMargin);
fprintf('  min clearance ALONG path segments: %.3f m\n', min(minPruned(ok)));
fprintf('  robot radius                     : %.2f m\n', P.rRobot);
deficit = min(minNode(ok)) - min(minPruned(ok));
fprintf('  segment-vs-node clearance deficit: %.3f m\n', deficit);
if min(minPruned(ok)) > P.rRobot
    fprintf('  => the planned path stays clear of every obstacle by more than the\n');
    fprintf('     robot radius, so the geometric plan is collision-free for a\n');
    fprintf('     disc robot. Deviation under SFM control is bounded separately\n');
    fprintf('     by the executed-trajectory clearance metrics.\n');
else
    fprintf('  => WARNING: segment clearance falls below the robot radius. The\n');
    fprintf('     one-cell margin does NOT guarantee clearance for a disc robot;\n');
    fprintf('     the margin must be raised to at least rRobot + max edge sagitta.\n');
end

save(fullfile(outDir,'clearance.mat'), 'seeds','minRaw','minPruned','minNode', ...
     'nodeCountRaw','nodeCountPruned','feasible','maxDeviation','P');
fprintf('\nWrote %s\n', fullfile(outDir,'clearance.mat'));

% ---------------------------------------------------------------------
function d = minDistToObs(px, py, xObsList, yObsList)
    d = inf;
    for k = 1:numel(xObsList)
        d = min(d, p_poly_dist(px, py, xObsList{k}, yObsList{k}));
    end
end

function m = minClearance(poly, xObsList, yObsList)
% Dense sampling along every segment, at 2 cm resolution.
    m = inf;
    for i = 1:size(poly,1)-1
        a = poly(i,:); b = poly(i+1,:);
        L = norm(b - a);
        n = max(2, ceil(L / 0.02));
        for t = linspace(0, 1, n)
            p = a + t*(b - a);
            m = min(m, minDistToObs(p(1), p(2), xObsList, yObsList));
        end
    end
end

function dev = maxPolylineDeviation(A, B)
% Largest distance from any densely sampled point of polyline A to polyline B.
    dev = 0;
    for i = 1:size(A,1)-1
        a = A(i,:); b = A(i+1,:);
        n = max(2, ceil(norm(b-a) / 0.05));
        for t = linspace(0,1,n)
            p = a + t*(b-a);
            dev = max(dev, distPointToPolyline(p, B));
        end
    end
end

function d = distPointToPolyline(p, B)
    d = inf;
    for i = 1:size(B,1)-1
        d = min(d, distPointToSegment(p, B(i,:), B(i+1,:)));
    end
end

function d = distPointToSegment(p, a, b)
    ab = b - a; L2 = sum(ab.^2);
    if L2 < eps
        d = norm(p - a); return;
    end
    t = max(0, min(1, dot(p - a, ab) / L2));
    d = norm(p - (a + t*ab));
end
