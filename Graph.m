function [G, nodeIDs, freeNodeIDs] = Graph(xMax, yMax, xObsList, yObsList)
% Builds 8‑way graph and removes any nodes/edges on obstacle polygons
%   G               MATLAB graph object
%   nodeIDs         yMax×xMax matrix mapping (y,x)→node ID
%   freeNodeIDs     vector of node IDs that survived (degree>0)

  % Helper func: is point (px,py) inside any obstacle
  function in = ptInObs(px,py)
    in = false;
    for k = 1:numel(xObsList)
      if inpolygon(px, py, xObsList{k}, yObsList{k})
        in = true; return
      end
    end
  end

  % Helper func: does segment (x1,y1)–(x2,y2) intersect any polygon boundary
  function cross = edgeIntersects(x1,y1,x2,y2)
    cross = false;
    for k = 1:numel(xObsList)
      [xi, yi] = polyxpoly([x1 x2], [y1 y2], xObsList{k}, yObsList{k});
      if ~isempty(xi)
        cross = true; return
      end
    end
  end

  % build nodeID lookup
  nodeIDs = reshape(1:(xMax*yMax), yMax, xMax);

  % 8‑way offsets + their movement costs
  dirs = [ -1  0 1;  -1  1 sqrt(2);
            0  1 1;   1  1 sqrt(2);
            1  0 1;   1 -1 sqrt(2);
            0 -1 1;  -1 -1 sqrt(2) ];

  src = []; tgt = []; w = [];
  for y = 1:yMax
    for x = 1:xMax
      if ptInObs(x,y), continue, end       % skip obstacle node
      u = nodeIDs(y,x);
      for d = 1:size(dirs,1)
        ny = y + dirs(d,1);  nx = x + dirs(d,2);
        cost = dirs(d,3);

        if nx<1 || nx>xMax || ny<1 || ny>yMax, continue, end
        if ptInObs(nx,ny),        continue, end  % skip into obstacle
        if edgeIntersects(x,y,nx,ny), continue, end  % skip slicing edge

        v = nodeIDs(ny,nx);
        if u < v
          src(end+1)=u; tgt(end+1)=v; w(end+1)=cost;
        end
      end
    end
  end

  G = graph(src, tgt, w);

  % collect only those nodes that have at least one edge
  freeNodeIDs = find(degree(G) > 0);
end
