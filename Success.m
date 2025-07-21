function [SFM_success, SFM_A_success ] = Success()
    % Used for visual of environment 

%rng(3, "twister");

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
global goal                 % Vector holding all coordinates for goals, first two indicies are for A* + SFM and SFM only respectively
global t_r                  % Reaction time variable for Force Goal
global p                    % Number of participants in the simulation, must be at least 2
global v_max                % Maximum velocity (CHANGES REQUIRE REDOING PARAMETERS DO NOT TOUCH)
global a                    % Parameter defining strength of repulsion from dynamic obstacles (Increasing this will increase overall repulsion)
global b                    % Parameter defining strength of repulsion from dynamic obstacles (Increasing this will decrease overall repulsion)
global r_inf                % Radius of influence for dynamic obstacles
global v_min                % Minimum velocity
global xObsList             % Variable holding x coordinates for every vertex for every obstacle
global yObsList             % Variable holding y coordinates for every vertex for every obstacle
global numObstacles         % Total number of obstacles
global a_s                  % Parameter defining strength of repulsion from static obstacles (KEEP HIGH, LOWERING CAUSES GLITCHING AND TELEPORTATION)
global r_inf_s              % Radius of influence for static obstacles

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

path = removeColinearNodes(path);

%% Create Plot and Obstacles
 
% Create Plot

%xticks([xMin:1:xMax]);      % Comment if grid is > 10x10 (too laggy)

%yticks([xMin:1:xMax]);      % Comment if grid is > 10x10 (too laggy)

% Plot obstacles

%% Plotting Avaliable Edges and Nodes

% Compute a position for every possible node:
[Ygrid, Xgrid] = ndgrid(1:yMax, 1:xMax);
allPos = [Xgrid(:), Ygrid(:)];  

% This is in form of [row x col] while freeNodes is node number
freePos = allPos(freeNodeIDs, :);  

% Plot graph edges - Comment if grid is > 10x10 (too laggy)

% Overlay free nodes


% Bring everything to front/back


%% Plotting Starting, Goal, and A-Star Path

% Plot starting point and goal

% Plot a* path
                                                      


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

% Defining buffer for position and velocity, the time interval and variable
% for time steps

t = [];
u = zeros(4,p,1000);
u(:,:,1) = u_0;
dt = 0.01;
i = 1;

% Restructuring path for manipulation and defining goals for A* model and
% SFM only model


path = path';

goal(:,1) = path(:,1);
goal(:,2) = [50,50];
%plot 


waynum = 1;

SFM_A_success = 0;
SFM_A_time = 0;
SFM_success = 0;
SFM_time = 0;

while any(any(abs(u(1:2,:,i)-goal) > 0.1) ~= 0)

    i = i+1;

    % Code to do success checking and time taken to reach goal, remains
    % zero if goal never reached
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
    
    % Calculation of Force for every agent including both A* model and
    % regular SFM model and humans
    
    F = Force_G(t,u(:,:,i-1)) + Force_O_Stat(t,u(:,:,i-1)) + Force_O_Dyn(t,u(:,:,i-1));
    
    u(1:2,:, i ) = u(1:2,:, i-1) + u(3:4,:, i-1) * dt + 1/2 * F' * dt^2;
    u(3:4,:, i ) = u(3:4,:, i -1) + F' * dt; 

    % Updating speeds to ensure they stay within the defined values

    for l = 1:p
        if norm(u(3:4,l,i)) > v_max 
            u(3:4,l,i) = u(3:4,l,i) / norm(u(3:4,l,i)) * v_max; 
        end

        if norm(u(3:4,l,i)) < v_min 
            u(3:4,l,i) = u(3:4,l,i) / norm(u(3:4,l,i)) * v_min; 
        end
    end
    
    % Change to make the time for the simulation different by timesteps
    if i > 3000
        break
    end


   % Plotting data for visualization of the simulated agents.
    x = squeeze(u(1,3:p,i));
    y = squeeze(u(2,3:p,i));
    x_r = squeeze(u(1,1,i));
    y_r = squeeze(u(2,1,i));
    x_r_n = squeeze(u(1,2,i));
    y_r_n = squeeze(u(2,2,i));
   
    

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


fprintf("SFM + A* success = %d, time taken = %d\n", SFM_A_success, SFM_A_time);
fprintf("SFM success = %d, time taken = %d\n", SFM_success, SFM_time);


end