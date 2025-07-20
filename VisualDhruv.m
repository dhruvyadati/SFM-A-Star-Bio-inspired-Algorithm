function [dist_astarsfm, dist_sfm] = VisualDhruv

% Used for visual of environment 

%rng(2, "twister");

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

t_r = 0.5;
v_max = 4;
v_min = 1;
r_inf = 1;
r_inf_s = 1.25;
b = 1;
a = 800;
a_s = 10000;
numObstacles = 10;

%% Get Graph Object and Path Nodes

% Get the object list
[xObsList, yObsList] = GenerateEnvironmentGraphObject(xMax, yMax, numObstacles, maxArea, bounds);

% Set random human goals not in obstacles
p = 20;
goal = zeros(2, p);
for i = 3:p
    while true
        xRand = (xMax-1) * rand() + 1;
        yRand = (yMax-1) * rand() + 1;
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


%% Run and Plot SFM Nodes

u_0 = zeros(4,p);
% Set random humans not in obstacles
for i = 3:p
    while true
        xRand = (xMax-1) * rand() + 1;
        yRand = (yMax-1) * rand() + 1;
        if ~ptInObs(xRand, yRand, xObsList, yObsList, margin)
            vxRand = randn();   % random velocity x
            vyRand = randn();   % random velocity y
            u_0(:, i) = [xRand; yRand; vxRand; vyRand];
            break;
        end
    end
end

u_0(:,1) = [2,1,1,1];
u_0(:,2) = [1,2,1,1];

% u_0_1 = [2; 2; 1; 1];
% u_0_2 = [11; 12;  -1; -1 ];
% u_0_3 = [10; 9; 3; -2];
% u_0 = [u_0_1, u_0_2, u_0_3];


t = [];
u = zeros(4,p,10000);
u(:,:,1) = u_0;
dt = 0.01;
i = 1;

path = path';

goal(:,1) = path(:,1);
goal(:,2) = [50,50];



plot(goal(1,:), goal(2,:), 'o', 'MarkerSize', 5, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g');
%plot 

h = plot(u_0(1, 3:p), u_0(2,3:p), 'o', 'MarkerSize', 5, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
robot = plot(u_0(1, 1), u_0(2,1), 'o', 'MarkerSize', 5, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
robot_no_SFM = plot(u_0(1, 2), u_0(2,2), 'o', 'MarkerSize', 5, 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'm');
waynum = 1;

while any(any(abs(u(1:2,:,i)-goal) > 0.1) ~= 0)
    i = i+1;
    
    if norm(u(1:2,1,i-1) - goal(:,1)) < 2;
        if waynum >= size(path(1:2,:))
            break
        end
        waynum = waynum + 1;
        goal(:,1) = path(:,waynum);
    end

    
    F = Force_G(t,u(:,:,i-1)) + Force_O_Stat(t,u(:,:,i-1)) + Force_O_Dyn(t,u(:,:,i-1));
    
    u(1:2,:, i ) = u(1:2,:, i-1) + u(3:4,:, i-1) * dt + 1/2 * F' * dt^2;
    u(3:4,:, i ) = u(3:4,:, i -1) + F' * dt; 
    for l = 1:p
        if norm(u(3:4,l,i)) > v_max 
            u(3:4,l,i) = u(3:4,l,i) / norm(u(3:4,l,i)) * v_max; 
        end

        if norm(u(3:4,l,i)) < v_min 
            u(3:4,l,i) = u(3:4,l,i) / norm(u(3:4,l,i)) * v_min; 
        end
    end
    
   
    if i > 10000
        break
    end
   
    x = squeeze(u(1,3:p,i));
    y = squeeze(u(2,3:p,i));
    x_r = squeeze(u(1,1,i));
    y_r = squeeze(u(2,1,i));
    x_r_n = squeeze(u(1,2,i));
    y_r_n = squeeze(u(2,2,i));
    set(h, 'XData', x, color="red");
    set(h, 'YData', y);
    set(robot, 'XData', x_r, color="blue");
    set(robot, 'YData', y_r);
    set(robot_no_SFM, 'XData', x_r_n, color="magenta");
    set(robot_no_SFM, 'YData', y_r_n);
    
    pause(0.01);

end

% Calculate distance traveled by A*+SFM
pos_astarsfm = squeeze(u(1:2,1,1:i));  % [2 x N] positions
diffs_astarsfm = diff(pos_astarsfm, 1, 2);  % [2 x N-1] differences
dist_astarsfm = sum(vecnorm(diffs_astarsfm));  % total distance

% Calculate distance traveled by SFM-only
pos_sfm = squeeze(u(1:2,2,1:i));  % [2 x N] positions
diffs_sfm = diff(pos_sfm, 1, 2);  % [2 x N-1] differences
dist_sfm = sum(vecnorm(diffs_sfm));  % total distance

%% Save runs

t = datetime('now','TimeZone','America/Los_Angeles','Format','d-MMM-y HH:mm:ss Z');
% Generate variables and figures using the current date and time
% Change "runs" to the directory you want to save files to
variables = sprintf('runs/variables_%s.mat', t); 
figures = sprintf('runs/region_%s.fig', t); 

% Save your data
save(variables);
savefig(figures);
