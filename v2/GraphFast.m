function [G, nodeIDs, freeNodeIDs, dmin] = GraphFast(xMax, yMax, xObsList, yObsList, margin)
% GraphFast builds the same pruned 8-connected graph as Graph.m, much faster.
%
% Graph.m called ptInObs once per node AND once per candidate neighbour, and
% ptInObs itself loops over every obstacle calling p_poly_dist -- roughly
% 225,000 point-to-polygon distance evaluations per 50x50 map with ten
% obstacles. It then called polyxpoly on every surviving candidate edge.
% Measured cost was 21-26 s per map, which dominated the whole experiment.
%
% Two changes, both exact (the output graph is identical):
%
%   1. The distance from each node to the nearest obstacle, dmin, is computed
%      ONCE and reused for both node pruning and edge screening.
%
%   2. Edge-intersection testing is skipped when it is provably unnecessary.
%      Distance-to-obstacle is 1-Lipschitz along a segment, so for any point p
%      at arc length t <= L from endpoint u, dist(p, obs) >= dmin(u) - L.
%      Hence if dmin(u) > L the entire segment is obstacle-free and polyxpoly
%      can be skipped. Since L <= sqrt(2), only the thin band of nodes within
%      sqrt(2) of an obstacle needs the exact test.
%
% Returns dmin as a fourth output so callers can reuse it (the clearance
% analysis needs it).

nObs = numel(xObsList);
nodeIDs = reshape(1:(xMax*yMax), yMax, xMax);

%% ---- 1. Distance to nearest obstacle, per node ----------------------
dmin = inf(yMax, xMax);
for k = 1:nObs
    xv = xObsList{k}; yv = yObsList{k};
    for y = 1:yMax
        for x = 1:xMax
            d = p_poly_dist(x, y, xv, yv);
            if d < dmin(y, x), dmin(y, x) = d; end
        end
    end
end

blocked = dmin <= margin;

%% ---- 2. Edge construction -------------------------------------------
dirs = [ -1  0 1;  -1  1 sqrt(2);
          0  1 1;   1  1 sqrt(2);
          1  0 1;   1 -1 sqrt(2);
          0 -1 1;  -1 -1 sqrt(2) ];

maxEdges = xMax * yMax * 8;
src = zeros(maxEdges, 1); tgt = zeros(maxEdges, 1); w = zeros(maxEdges, 1);
ne = 0;

for y = 1:yMax
    for x = 1:xMax
        if blocked(y, x), continue; end
        u = nodeIDs(y, x);
        for d = 1:size(dirs, 1)
            ny = y + dirs(d,1);  nx = x + dirs(d,2);
            L  = dirs(d,3);

            if nx < 1 || nx > xMax || ny < 1 || ny > yMax, continue; end
            if blocked(ny, nx), continue; end

            v = nodeIDs(ny, nx);
            if u >= v, continue; end          % dedupe undirected pair

            % Exact skip: segment provably clear of every polygon.
            if dmin(y, x) <= L || dmin(ny, nx) <= L
                if edgeIntersects(x, y, nx, ny), continue; end
            end

            ne = ne + 1;
            src(ne) = u; tgt(ne) = v; w(ne) = L;
        end
    end
end

G = graph(src(1:ne), tgt(1:ne), w(1:ne), xMax*yMax);
freeNodeIDs = find(degree(G) > 0);

    function cross = edgeIntersects(x1, y1, x2, y2)
        cross = false;
        for kk = 1:nObs
            xi = polyxpoly([x1 x2], [y1 y2], xObsList{kk}, yObsList{kk});
            if ~isempty(xi), cross = true; return; end
        end
    end
end
