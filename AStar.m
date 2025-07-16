function [path, cameFrom, gScore] = AStar(G, nodeIDs, startXY, goalXY)
%   nodeIDs             yMax×xMax lookup from (x,y) to nodeID
%   startXY,goalXY      [x y] in grid coords
%   path                N×2 list of (x,y) from start to goal
%   cameFrom            predecessor map (basically nodeID to nodeID)
%   gScore              final cost for all nodes

    [yMax,xMax] = size(nodeIDs);
    start = nodeIDs(startXY(2), startXY(1));
    goal  = nodeIDs(goalXY(2), goalXY(1));
    n     = numnodes(G);

    % (x,y) positions per node for heuristic
    [Ygrid, Xgrid] = ind2sub([yMax xMax], 1:n);
    Xpos = Xgrid(:); Ypos = Ygrid(:);
    heuristic = @(u) hypot(Xpos(u)-goalXY(1), Ypos(u)-goalXY(2));

    % The data structures for a*
    gScore  = inf(n,1);
    fScore  = inf(n,1);
    cameFrom= zeros(n,1,'uint32');
    openSet = false(n,1);

    gScore(start)=0;
    fScore(start)=heuristic(start);
    openSet(start)=true;

    while any(openSet)
        % pick open node with min fScore
        openIdx = find(openSet);
        [~,minI] = min(fScore(openIdx));
        current = openIdx(minI);

        if current==goal
            % reconstruct
            pathNodes = reconstruct(current);
            [r,c] = ind2sub([yMax xMax], pathNodes);
            path = [c r];  % convert to (x,y)
            return
        end

        openSet(current)=false;
        nbrs = neighbors(G, current);

        for v = nbrs'
            eid = findedge(G, current, v);
            tentative = gScore(current) + G.Edges.Weight(eid);
            if tentative < gScore(v)
                cameFrom(v) = current;
                gScore(v)   = tentative;
                fScore(v)   = tentative + heuristic(v);
                openSet(v)  = true;
            end
        end
    end

    % no path
    path = [];

    function pn = reconstruct(node)
      pn = node;
      while node~= start
        node = cameFrom(node);
        if node==0, pn=[]; return; end
        pn(end+1,1) = node;
      end
      pn = flip(pn);
    end
end
