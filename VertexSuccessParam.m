function [SFM_A_success, SFM_success] = VertexSuccessParam(numVertices)
% VertexSuccessParam runs one navigation trial in which every obstacle has
% exactly numVertices vertices, and reports whether the hybrid A*+SFM and the
% SFM-only robot reached the goal. It is identical to
% GetNumberVerticesVsSuccessRate but without the per-call figure/variable
% saving (the sweep runs hundreds of trials) and without an internal rng call
% (the driver sets the random stream before each trial).

% Set the maximum area of each obstacle
maxArea = 150;

% Set the bounds of plot
xMin = 1;
xMax = 50;
yMin = 1;
yMax = 50;
bounds = [xMin, xMax, yMin, yMax];
margin = 1;

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
numObstacles = 10;

%% Get Graph Object and Path Nodes
[xObsList, yObsList] = GenerateEnvironmentNumberVertices(xMax, yMax, numObstacles, maxArea, bounds, numVertices);
numObstacles = numel(xObsList);

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

nodeIDs = reshape(1:(xMax * yMax), yMax, xMax);

startNode = nodeIDs(startXY(2), startXY(1));
goalNode = nodeIDs(goalXY(2), goalXY(1));

[path, cameFrom, gScore] = AStar(G, nodeIDs, startXY, goalXY);

if isempty(path)
    warning('A* did not find a path for vertex count %d. Skipping this trial.', numVertices);
    SFM_A_success = 0;
    SFM_success = 0;
    return;
end

path = removeColinearNodes(path);

%% Run SFM Nodes
u_0 = zeros(4,p);
for i = 3:p
    while true
        xRand = (xMax-1) * rand() + 1;
        yRand = (yMax-1) * rand() + 1;
        if ~ptInObs(xRand, yRand, xObsList, yObsList, margin)
            vxRand = randn();
            vyRand = randn();
            u_0(:, i) = [xRand; yRand; vxRand; vyRand];
            break;
        end
    end
end

u_0(:,1) = [2,1,1,1];
u_0(:,2) = [1,2,1,1];

t = [];
u = zeros(4,p,5000);
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

    if norm(u(1:2,1,i-1) - goal(:,1)) < 2;
         waynum = waynum + 1;
        if waynum > size(path(1:2,:))
            waynum = waynum -1;
        end
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

    if i > 3000
        break
    end

end

end
