% Used for visual of environment 

% Set the maximum area of each obstacle       
maxArea = 150;      

% Set the bounds of plot
xMin = 1;                  
xMax = 50;
yMin = 1;            
yMax = 50;
bounds = [xMin, xMax, yMin, yMax];
margin = 1;

% Define SFM Variables
global goal 
global t_r
global p
global v_max
global a
global b
global r_inf
global v_min
global xObsList
global yObsList
global numObstacles
global a_s
global r_inf_s

t_r = 0.8;
v_max = 5;
v_min = 0.5;
r_inf = 0.5;
r_inf_s = 1.25;
b = 1;
a = 400;
a_s = 100;
numObstacles = 6; 

%% Get Graph Object and Path Nodes

% Get the object list
[xObsList, yObsList] = GenerateEnvironmentGraphObject(xMax, yMax, numObstacles, maxArea, bounds);

% Set random human goals not in obstacles
p = 16;
goal = zeros(2, p);
for i = 1:p
    while true
        xRand = 50 * rand();
        yRand = 50 * rand();
        if ~ptInObs(xRand, yRand, xObsList, yObsList, margin)
            goal(:, i) = [xRand; yRand];
            break;
        end
    end
end

% Make graph G
[G, nodeIDs, freeNodeIDs] = Graph(xMax, yMax, xObsList, yObsList, margin);

% call aStar algorithm
startXY = [xMin, yMin];
goalXY = [xMax, yMax];

nodeIDs = reshape(1:(xMax * yMax), yMax, xMax);  % Matrix: (row = y, col = x)

startNode = nodeIDs(startXY(2), startXY(1));  % y, x
goalNode = nodeIDs(goalXY(2), goalXY(1));

[path, cameFrom, gScore] = AStar(G, nodeIDs, startXY, goalXY); 

path = removeColinearNodes(path);
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
scatter(freePos(:,1), freePos(:,2), 5, [0.7 0.7 0.7], 'filled'); % Comment if grid is > 10x10 (too laggy)

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


%% Plot SFM Nodes

u_0 = zeros(4,p);
u_0 = 50 * rand(size(u_0));

% u_0_1 = [2; 2; 1; 1];
% u_0_2 = [11; 12;  -1; -1 ];
% u_0_3 = [10; 9; 3; -2];
% u_0 = [u_0_1, u_0_2, u_0_3];

t = [];
u = zeros(4,p,1000);
u(:,:,1) = u_0;
dt = 0.01;
i = 1;

axis([1 xMax 1 yMax]);
plot(goal(1,:), goal(2,:), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g');

h = plot(u_0(:, 1), u_0(:,2), 'o');

while any(any(abs(u(1:2,:,i)-goal) > 0.1) ~= 0)
    i = i+1;
    F = Force_G(t,u(:,:,i-1)) + Force_O_Stat(t,u(:,:,i-1)) + Force_O_Dyn(t,u(:,:,i-1));
    
    u(1:2,:, i ) = u(1:2,:, i-1) + u(3:4,:, i-1) * dt + 1/2 * F' * dt^2;
    u(3:4,:, i ) = u(3:4,:, i -1) + F' * dt; 

    if norm(u(3:4,:,i)) > v_max 
        u(3:4,:,i) = u(3:4,:,i) / norm(u(3:4,:,i)) * v_max; 
    end
    if norm(u(3:4,:,i)) < v_min 
        u(3:4,:,i) = u(3:4,:,i) / norm(u(3:4,:,i)) * v_min; 
    end

    if i > 10000
        break
    end
   
    x = squeeze(u(1,:,i));
    y = squeeze(u(2,:,i));
    set(h, 'XData', x, color="red");
    set(h, 'YData', y);
    
    pause(0.01);

end

%% Save runs

%{
t = datetime('now','TimeZone','America/Los_Angeles','Format','d-MMM-y HH:mm:ss Z');
% Generate a filename using the current date and time
% Change "runs" to the directory you want to save files to
filename = sprintf('runs/variables_%s.mat', t); 
filename2 = sprintf('runs/region_%s.fig', t); 
% Save your data
save(filename);
savefig(filename2);
%}



