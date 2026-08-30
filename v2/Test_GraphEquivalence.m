% Test_GraphEquivalence.m
% Verifies that the optimised GraphFast/AStarFast produce EXACTLY the same
% graph and the same plan as the original Graph/AStar, so the speedups cannot
% be mistaken for a change in behaviour.
%
% Usage:  cd to the repository root, then  run('v2/Test_GraphEquivalence.m')

addpath(fileparts(mfilename('fullpath')));
addpath(fileparts(fileparts(mfilename('fullpath'))));

P = SfmParams();
bounds = [P.xMin, P.xMax, P.yMin, P.yMax];
nCheck = 3;

rng(4242, 'twister');
seeds = randi([1, 2^31-1], nCheck, 1);

fprintf('=== Graph / A* equivalence: optimised vs original ===\n\n');
allOK = true;

for k = 1:nCheck
    rng(seeds(k), 'twister');
    [xo, yo] = GenerateEnvironmentGraphObject(P.xMax, P.yMax, ...
                   P.numObstacles, P.maxArea, bounds);

    tic; [G1, n1] = Graph(P.xMax, P.yMax, xo, yo, P.graphMargin);   t1 = toc;
    n1 = reshape(1:(P.xMax*P.yMax), P.yMax, P.xMax);
    tic; [G2, n2] = GraphFast(P.xMax, P.yMax, xo, yo, P.graphMargin); t2 = toc;

    E1 = sortrows([G1.Edges.EndNodes, G1.Edges.Weight]);
    E2 = sortrows([G2.Edges.EndNodes, G2.Edges.Weight]);
    sameEdges = isequal(size(E1), size(E2)) && max(abs(E1(:) - E2(:))) < 1e-12;

    tic; p1 = AStar(G1, n1, P.start, P.goal);      ta1 = toc;
    tic; p2 = AStarFast(G2, n2, P.start, P.goal);  ta2 = toc;
    samePath = isequal(size(p1), size(p2)) && isequal(p1, p2);

    fprintf('seed %-11d edges %5d vs %5d  %s\n', seeds(k), ...
            size(E1,1), size(E2,1), tf(sameEdges));
    fprintf('             graph build  %7.2f s -> %6.2f s  (%.0fx)\n', ...
            t1, t2, t1/max(t2,eps));
    fprintf('             A* search    %7.2f s -> %6.3f s  (%.0fx)\n', ...
            ta1, ta2, ta1/max(ta2,eps));
    fprintf('             path %d nodes, identical: %s\n\n', ...
            size(p1,1), tf(samePath));

    allOK = allOK && sameEdges && samePath;
end

if allOK
    fprintf('=== ALL EQUIVALENT ===\n');
else
    error('Test_GraphEquivalence:mismatch', 'Optimised path differs from original.');
end

function s = tf(b)
    if b, s = 'MATCH'; else, s = 'DIFFER'; end
end
