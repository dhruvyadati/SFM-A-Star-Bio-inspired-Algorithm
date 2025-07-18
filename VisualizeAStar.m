% Used for visual of environment 

% Set the number of obstacles and maximum area of each obstacle
numObstacles = 24;        
maxArea = 200;      

% Set the bounds of plot
xMin = 1;                  
xMax = 50;
yMin = 1;            
yMax = 50;
bounds = [xMin, xMax, yMin, yMax];

% Get the object list
[xObsList, yObsList] = GenerateEnvironmentGraphObject(xMax, yMax, numObstacles, maxArea, bounds);

% Make graph G
[G, nodeIDs, freeNodeIDs] = Graph(xMax, yMax, xObsList, yObsList);

% call aStar algorithm
startXY = [xMin, yMin];
goalXY = [xMax, yMax];

nodeIDs = reshape(1:(xMax * yMax), yMax, xMax);  % Matrix: (row = y, col = x)

startNode = nodeIDs(startXY(2), startXY(1));  % y, x
goalNode = nodeIDs(goalXY(2), goalXY(1));

[path, cameFrom, gScore] = AStar(G, nodeIDs, startXY, goalXY); 

%% Create Plot and Obstacles
 
% Create Plot
figure;
hold on;
axis equal;
grid on;
title('Pathfinding Visual Environment');
xlabel('X'); 
ylabel('Y');
xlim([xMin xMax]); 
%xticks([xMin:1:xMax]);      % Comment if grid is > 10x10 (too laggy)
ylim([yMin yMax]);
%yticks([xMin:1:xMax]);      % Comment if grid is > 10x10 (too laggy)

% Plot obstacles
for i = 1:numObstacles
    fill(xObsList{i}, yObsList{i}, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'k');
end

%% Plotting Avaliable Edges and Nodes

% Compute a position for every possible node:
[Ygrid, Xgrid] = ndgrid(1:yMax, 1:xMax);
allPos = [Xgrid(:), Ygrid(:)];  

% This is in form of [row x col] while freeNodes is node number
freePos = allPos(freeNodeIDs, :);  

% Plot graph edges - Comment if grid is > 10x10 (too laggy)
h = plot(G, ...
    'XData', allPos(:,1), ...
    'YData', allPos(:,2), ...
    'NodeLabel', {}, ...            % no labels
    'LineStyle', '-', ...
    'EdgeColor', [0.7 0.7 0.7], ... % light gray edges
    'Marker', 'none');              % don't draw nodes yet

% Overlay free nodes
scatter(freePos(:,1), freePos(:,2), 5, 'b', 'filled'); % Comment if grid is > 10x10 (too laggy)

% bring everything to front/back
uistack(h, 'bottom');

%% Plotting Starting, Goal, and A-Star Path

% Plot starting point and goal
plot(startXY(1), startXY(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'r'); % start
plot(goalXY(1), goalXY(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'g');   % goal

% plot a* path
if ~isempty(path)                                              
    plot(path(:,1), path(:,2), 'k-', 'LineWidth', 1.5);    
else                                                           
    warning('No path was found :(');                        
end                                                            

%% Save runs


t = datetime('now','TimeZone','America/Los_Angeles','Format','d-MMM-y HH:mm:ss Z');
% Generate a filename using the current date and time
% Change "runs" to the directory you want to save files to
filename = sprintf('runs/variables_%s.mat', t); 
filename2 = sprintf('runs/region_%s.fig', t); 
% Save your data
save(filename);
savefig(filename2);




