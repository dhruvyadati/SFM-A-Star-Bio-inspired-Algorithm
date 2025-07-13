% Used for visual of environment 

% Set the number of obstacles and maximum area of each obstacle
numObstacles = 20;
maxArea = 20000;

% Set the bounds of plot
xMin = 0
xMax = 1000;
yMin = 0;
yMax = 1000;
bounds = [xMin, xMax, yMin, yMax];

% Get the occupancy grid g and obstacle list
[g, xObsList, yObsList] = GenerateEnvironment(xMax, yMax, numObstacles, maxArea, bounds);

% Create Plot
figure;
hold on;
axis equal;
grid on;
title('Pathfinding Visual Environment');
xlabel('X');
ylabel('Y');
xlim([xMin xMax]);
ylim([yMin yMax]);

% Plot obstacles
for i = 1:numObstacles
    fill(xObsList{i}, yObsList{i}, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'k');
end

% Plot starting point and goal
plot(xMin, yMin, 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k'); % starting
plot(xMax, yMax, 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g'); % goal

