% Set # of obstacles and max area of each obstacle
numObstacles = 5;
maxArea = 20;
xMin = 0;
xMax = 10;
yMin = 0;
yMax = 10;
bounds = [xMin, xMax, yMin, yMax];

% Generate obstacle list
[xObsList, yObsList] = GenerateEnvironment(xMax, yMax, numObstacles, maxArea, bounds);

% Make graph
G = Graph(xMax, yMax, xObsList, yObsList);

% Define start and goal coordinates in (x, y)
startXY = [1, 1];
goalXY = [xMax, yMax];

nodeIDs = reshape(1:(xMax * yMax), yMax, xMax);  % Matrix: (row = y, col = x)
% nodeID = nodeIDs(y, x);  % For any (x, y) location

startNode = nodeIDs(startXY(2), startXY(1));  % y, x
goalNode = nodeIDs(goalXY(2), goalXY(1));


%{
% Use MATLabs Dijkstra to find shortest path using graph
% Do not need this right now (we will use a*)
[pathNodes, pathLength] = shortestpath(G, startNode, goalNode);


% Convert path node IDs back to (x, y)
[yPath, xPath] = ind2sub(size(nodeIDs), pathNodes);
%}

figure;
hold on;
axis equal;
grid on;
title('Graph-Based Pathfinding Environment');
xlabel('X');
ylabel('Y');
xlim([xMin xMax]);
xticks([xMin:1:xMax]);    % Comment if grid is > 10x10 (too laggy)
ylim([yMin yMax]);
yticks([xMin:1:xMax]);    % Comment if grid is > 10x10 (too laggy)

% Plot obstacles
for i = 1:numObstacles
    fill(xObsList{i}, yObsList{i}, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'k');
end

% Plot start and goal
plot(startXY(1), startXY(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'r'); % start
plot(goalXY(1), goalXY(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'g');   % goal

nodesWithoutObs = zeros(xMax, yMax);
% Plot the nodes
for i = 1:xMax
    for j = 1:yMax
        for z = 1:numObstacles
            if ~isInsideAnyPolygon(i, j, xObsList, yObsList)
                % If you are exceeding a grid 10x10, comment the plot out -
                % it gets laggy
                plot(i, j, 'o', 'MarkerSize', 2, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
                nodesWithoutObs(i, j) = nodeIDs(i,j);
            end
        end  
    end
end

%{
% Plot path if one exists
if ~isempty(pathNodes)
    plot(xPath, yPath, 'b-', 'LineWidth', 2);
else
    disp('No path');
end
%}