% Generates the Grid to a Graph

% Set the number of obstacles and maximum area of each obstacle
numObstacles = 5;
maxArea = 20;
xMin = 0;
xMax = 10;
yMin = 0;
yMax = 10;
bounds = [xMin, xMax, yMin, yMax];
marginFromObs = 1; % in lattice points

% Generate occupancy grid and obstacles
[g, xObsList, yObsList] = GenerateEnvironmentWithOccupancy(xMax, yMax, numObstacles, maxArea, bounds);

% Convert grid to graph w/ 8-way connectivity
G = GridToGraph(g);

% Define start and goal coordinates in (x, y)
startXY = [xMin + 1, yMin + 1];  % +1 to be within grid bounds
goalXY  = [xMax, yMax];          % bottom-right

% Convert (x, y) to linear node IDs
startNode = sub2ind(size(g), startXY(2), startXY(1));
goalNode  = sub2ind(size(g), goalXY(2), goalXY(1));

% Use MATLabs' Dijkstra to find shortest path using graph
% Do not need this right now (we will use a*)

[pathNodes, pathLength] = shortestpath(G, startNode, goalNode);


% Convert path node IDs back to (x, y)
[yPath, xPath] = ind2sub(size(g), pathNodes);


figure;
hold on;
axis equal;
grid on;
title('Graph-Based Pathfinding Environment');
xlabel('X');
ylabel('Y');
xlim([xMin xMax]);
%xticks([xMin:1:xMax]);
ylim([yMin yMax]);
%yticks([xMin:1:xMax]);

% Plot obstacles
for i = 1:numObstacles
    fill(xObsList{i}, yObsList{i}, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'k');
end

% Plot start and goal points
plot(startXY(1), startXY(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'r'); % start
plot(goalXY(1), goalXY(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'g');   % goal

% Plot the nodes
for i = 1:xMax
    for j = 1:yMax
        if g(j, i) == 1
            plot(i, j, 'o', 'MarkerSize', 2, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
        end
    end
end

% Plot path if one exists
if ~isempty(pathNodes)
    plot(xPath, yPath, 'k-', 'LineWidth', 2);
else
    disp('No path found between start and goal.');
end


%{
[yMax, xMax] = size(g);
[X, Y] = meshgrid(1:xMax, 1:yMax);
positions = [X(:), Y(:)];
%}

G.Nodes.Name = string(1:numel(g))';  % Optional names
%G.Nodes.Position = positions;       % Store x/y positions if needed

%{
if currentNode < neighborNode
    % Add edge just once
    edgesSource(end+1) = currentNode;
    edgesTarget(end+1) = neighborNode;
    weights(end+1) = w;
end

%}
