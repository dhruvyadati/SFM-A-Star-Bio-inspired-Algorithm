function G = GridToGraph(g)
% Converts a occupancy grid to a graph with 8-way.
% g(y, x): 1 = free, 0 = obstacle
% Returns MATLAB graph object G

    [yMax, xMax] = size(g);
    nodeIDs = reshape(1:numel(g), yMax, xMax);

    edgesSource = [];
    edgesTarget = [];
    weights = [];

    % 8-way neighbor offsets: [dy, dx, weight]
    directions = [
        -1,  0, 1;           % N
        -1,  1, sqrt(2);     % NE
         0,  1, 1;           % E
         1,  1, sqrt(2);     % SE
         1,  0, 1;           % S
         1, -1, sqrt(2);     % SW
         0, -1, 1;           % W
        -1, -1, sqrt(2)      % NW
    ];

    for y = 1:yMax
        for x = 1:xMax
            if g(y,x) == 0
                continue;  % Skip obstacles
            end

            currentNode = nodeIDs(y,x);

            for k = 1:size(directions,1)
                dy = directions(k,1);
                dx = directions(k,2);
                w  = directions(k,3);

                ny = y + dy;
                nx = x + dx;

                if ny >= 1 && ny <= yMax && nx >= 1 && nx <= xMax && g(ny,nx) == 1
                    neighborNode = nodeIDs(ny,nx);

                    edgesSource(end+1) = currentNode;
                    edgesTarget(end+1) = neighborNode;
                    weights(end+1) = w;
                end
            end
        end
    end

    G = graph(edgesSource, edgesTarget, weights);
end
