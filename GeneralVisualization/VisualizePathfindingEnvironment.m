% Used for visual of environment 

% Set the number of obstacles and maximum area of each obstacle
numObstacles = 20;        
maxArea = 100;      

% Set the bounds of plot
xMin = 1;                  
xMax = 100;
yMin = 1;            
yMax = 100;
bounds = [xMin, xMax, yMin, yMax];

% Get the occupancy grid g and obstacle list
[g, xObsList, yObsList] = GenerateEnvironment(xMax, yMax, numObstacles, maxArea, bounds);

% call aStar algorithm
startXY = [xMin, yMin];
goalXY = [xMax, yMax];
[path, ~] = AStar(g, startXY, goalXY, true);   

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

% plot a* path
if ~isempty(path)                                              
    plot(path(:,1), path(:,2), 'b-', 'LineWidth', 1.5);    
    %path(:,1) - x-coords of matrix 
    %path(:,2) - y-coords of matrix 
    % path (row, col) 
    %example path 
    %{ 
    path = [
                5 6 
                7 8 
                9 10
    ]
    the x coordinates would be the first column 
    the y coordinates would be the second column 

    %}
else                                                           
    warning('No path was found :(');                        
end                                                            