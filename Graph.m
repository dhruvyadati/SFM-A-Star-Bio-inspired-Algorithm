function [G, nodeIDs] = Graph(xMax, yMax, xObsList, yObsList)
% Builds 8‑way graph over a xMax×yMax grid and 
% removes any nodes/edges that intersect the obstacles

    % helper to test a point-in‑any‑polygon:
    function in = ptInObstacle(x,y)
        in = false;
        for k=1:numel(xObsList)
            if inpolygon(x,y, xObsList{k}, yObsList{k})
                in = true; return
            end
        end
    end

    % helper to test if the straight edge (x1,y1)-(x2,y2)
    % intersects any polygon boundary
    function cross = edgeCrosses(x1,y1,x2,y2)
        cross = false;
        for k=1:numel(xObsList)
            [xi,yi] = polyxpoly([x1 x2], [y1 y2], xObsList{k}, yObsList{k});
            if ~isempty(xi)
                cross = true; return
            end
        end
    end

    % Prepare nodeID lookup
    nodeIDs = reshape(1:(xMax*yMax), yMax, xMax);

    % 8 neighbors + weights
    dirs = [ -1  0 1;  -1  1 sqrt(2);
              0  1 1;   1  1 sqrt(2);
              1  0 1;   1 -1 sqrt(2);
              0 -1 1;  -1 -1 sqrt(2) ];

    src = []; tgt = []; w = [];
    for y=1:yMax
      for x=1:xMax
        % skip nodes inside any obstacle
        if ptInObstacle(x,y), continue, end
        u = nodeIDs(y,x);

        for d=1:size(dirs,1)
          ny = y + dirs(d,1);
          nx = x + dirs(d,2);
          weight = dirs(d,3);
          % bounds check, skip neighbor inside obstacle & 
          % skip edge if it cuts through obstacle
          if nx<1 || nx>xMax || ny<1 || ny>yMax, continue, end
          if ptInObstacle(nx,ny), continue, end

          if edgeCrosses(x, y, nx, ny), continue, end

          v = nodeIDs(ny,nx);
          % add undirected edge once
          if u < v
            src(end+1)=u; tgt(end+1)=v; w(end+1)=weight;
          end
        end
      end
    end

    G = graph(src, tgt, w);


end
