function [path, gScore] = AStarFast(G, nodeIDs, startXY, goalXY)
% AStarFast is a drop-in replacement for AStar.m with identical output.
%
% The original implementation called findedge(G, current, v) for every
% neighbour of every expanded node. findedge performs a linear scan of the
% edge table, so a single 50x50 plan cost ~21 s of wall clock. Planning cost
% is a headline figure for this system, so that had to come down.
%
% Since planning cost is a headline figure for this system, that had to come
% down before it was worth reporting.
%
% Here the edge list is converted ONCE into a symmetric sparse weight matrix,
% and neighbour lookup becomes a sparse column access. Search order and the
% resulting path are unchanged: same admissible Euclidean heuristic, same
% tie-breaking by lowest index among equal f-scores.

[yMax, xMax] = size(nodeIDs);
n = numnodes(G);

% ---- One-off adjacency construction ---------------------------------
E = G.Edges.EndNodes;
W = G.Edges.Weight;
A = sparse([E(:,1); E(:,2)], [E(:,2); E(:,1)], [W; W], n, n);

start = nodeIDs(startXY(2), startXY(1));
goal  = nodeIDs(goalXY(2),  goalXY(1));

[Ygrid, Xgrid] = ind2sub([yMax xMax], (1:n).');
hAll = hypot(Xgrid - goalXY(1), Ygrid - goalXY(2));

gScore   = inf(n, 1);
fScore   = inf(n, 1);
cameFrom = zeros(n, 1, 'uint32');
openSet  = false(n, 1);
closed   = false(n, 1);

gScore(start) = 0;
fScore(start) = hAll(start);
openSet(start) = true;

path = [];

while any(openSet)
    openIdx = find(openSet);
    [~, minI] = min(fScore(openIdx));
    current = openIdx(minI);

    if current == goal
        pathNodes = reconstruct(cameFrom, start, current);
        if isempty(pathNodes), return; end
        [r, c] = ind2sub([yMax xMax], pathNodes);
        path = [c r];
        return;
    end

    openSet(current) = false;
    closed(current)  = true;

    [nbrs, ~, wts] = find(A(:, current));
    for t = 1:numel(nbrs)
        vtx = nbrs(t);
        if closed(vtx), continue; end
        tentative = gScore(current) + wts(t);
        if tentative < gScore(vtx)
            cameFrom(vtx) = current;
            gScore(vtx)   = tentative;
            fScore(vtx)   = tentative + hAll(vtx);
            openSet(vtx)  = true;
        end
    end
end
end

% ---------------------------------------------------------------------
function pn = reconstruct(cameFrom, start, node)
    pn = node;
    while node ~= start
        node = cameFrom(node);
        if node == 0, pn = []; return; end
        pn(end+1, 1) = node; %#ok<AGROW>
    end
    pn = flip(pn);
end
