function [SFM_success, SFM_A_success] = SuccessParam(numObstaclesIn, maxAreaIn)
% SuccessParam runs one navigation trial and reports whether the SFM-only
% and the hybrid A*+SFM robot reached the goal. It is identical to Success()
% but exposes the obstacle count and maximum obstacle area as inputs so the
% obstacle-count and area sweeps run the exact same simulation as the
% headline experiment, with one parameter varied.
%
% The random stream is set by the caller (driver) before each call, so this
% function never calls rng itself.

% Maximum area of each obstacle (swept by the area experiment)
maxArea = maxAreaIn;

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
r_inf_s = 1;
b = 1;
a = 800;
a_s = 10000;

% Number of obstacles (swept by the obstacle-count experiment)
numObstacles = numObstaclesIn;

%% Get Graph Object and Path Nodes

% Get the object list
[xObsList, yObsList] = GenerateEnvironmentGraphObject(xMax, yMax, numObstacles, maxArea, bounds);

% Set random human goals not in obstacles
p = 50;
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

% Call aStar algorithm
startXY = [xMin, yMin];
goalXY = [xMax, yMax];

nodeIDs = reshape(1:(xMax * yMax), yMax, xMax);  % Matrix: (row = y, col = x)

startNode = nodeIDs(startXY(2), startXY(1));  % y, x
goalNode = nodeIDs(goalXY(2), goalXY(1));

[path, cameFrom, gScore] = AStar(G, nodeIDs, startXY, goalXY);

if isempty(path)
    warning('A* did not find a path (numObstacles=%d, maxArea=%g). Skipping trial.', numObstacles, maxArea);
    SFM_success = 0;
    SFM_A_success = 0;
    return;
end

path = removeColinearNodes(path);

%% Run SFM Nodes

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

t = [];
u = zeros(4,p,4000);
u(:,:,1) = u_0;
dt = 0.01;
i = 1;

path = path';

goal(:,1) = path(:,1);
goal(:,2) = [50,50];

waynum = 1;

SFM_A_success = 0;
SFM_A_time = 0;
SFM_success = 0;
SFM_time = 0;

while any(any(abs(u(1:2,:,i)-goal) > 0.1) ~= 0)

    i = i+1;

    if SFM_A_success == 0
        if norm(u(1:2,1,i-1) - [50,50]) < 2
                SFM_A_success = 1;
                SFM_A_time = i;
        end
    end

    if SFM_success == 0
        if norm(u(1:2,2,i-1) - [50,50]) < 2
                SFM_success = 1;
                SFM_time = i;
        end
    end

    if SFM_success == 1  && SFM_A_success == 1
        break
    end

    % Code to update waypoints for SFM + A* model
    if norm(u(1:2,1,i-1) - goal(:,1)) < 2;
         waynum = waynum + 1;
        if waynum > size(path(1:2,:))
            waynum = waynum -1;
        end
        goal(:,1) = path(:,waynum);
    end

    % Calculation of Force for every agent
    F = Force_G(t,u(:,:,i-1)) + Force_O_Stat(t,u(:,:,i-1)) + Force_O_Dyn(t,u(:,:,i-1));

    u(1:2,:, i ) = u(1:2,:, i-1) + u(3:4,:, i-1) * dt + 1/2 * F' * dt^2;
    u(3:4,:, i ) = u(3:4,:, i -1) + F' * dt;

    % Keep speeds within bounds
    for l = 1:p
        if norm(u(3:4,l,i)) > v_max
            u(3:4,l,i) = u(3:4,l,i) / norm(u(3:4,l,i)) * v_max;
        end

        if norm(u(3:4,l,i)) < v_min
            u(3:4,l,i) = u(3:4,l,i) / norm(u(3:4,l,i)) * v_min;
        end
    end

    if i > 3000
        break
    end

end

end
