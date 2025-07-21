function PlotNetForceField_WithRepulsion()
    % Plot net force vectors (goal + static obstacle repulsion)
    % assuming an agent is placed at each free grid point.

    %% Global SFM Setup
    global goal t_r p v_max v_min xObsList yObsList numObstacles a b a_s r_inf r_inf_s

    % Parameters
    xMin = 1; xMax = 50;
    yMin = 1; yMax = 50;
    bounds = [xMin xMax yMin yMax];
    margin = 0;
    maxArea = 150;
    numObstacles = 10;
    goal = [xMax; yMax];

    % SFM constants
    t_r = 0.5;
    v_max = 4;
    v_min = 1;
    r_inf = 1;
    r_inf_s = 1.5;   
    b = 1;
    a = 800;
    a_s = 1000;    
    p = 1;
    
    %% Environment & Grid
    [xObsList, yObsList] = GenerateEnvironmentGraphObject(xMax, yMax, numObstacles, maxArea, bounds);
    [X, Y] = meshgrid(1:0.25:xMax, 1:0.25:yMax);
    U = zeros(size(X));
    V = zeros(size(Y));
    stuckMask = false(size(X));

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

    %% Compute Net Forces at Each Grid Point
    for idx = 1:numel(X)
        x = X(idx);
        y = Y(idx);

        if ptInObs(x, y, xObsList, yObsList, margin)
            U(idx) = NaN;
            V(idx) = NaN;
            continue;
        end

        % Place dummy agent
        u_tmp = zeros(4,1);
        u_tmp(1:2) = [x; y];
        u_tmp(3:4) = [0; 0];

        Fg = Force_G(0, u_tmp);
        Fo = Force_O_Stat(0, u_tmp);
        Fnet = Fg + Fo;

        U(idx) = Fnet(1);
        V(idx) = Fnet(2);

        if norm(Fnet) < 0.3   % threshold for "stuck"
            stuckMask(idx) = true;
        end
    end

    %% Plotting
    figure; hold on; axis equal;
    xlim([xMin xMax]); ylim([yMin yMax]);
    xlabel('X'); ylabel('Y');
    title('Net Force Field with SFM Trap Risk Heatmap');
    grid on;
    
    % Obstacles
    for i = 1:numObstacles
        fill(xObsList{i}, yObsList{i}, 'r', 'FaceAlpha', 0.3, ...
             'EdgeColor', 'k', 'DisplayName', 'Obstacle');
    end
    
    % Vectors
    h_vec = quiver(X, Y, U, V, 1.3, 'b', 'DisplayName', 'Net Force');
    
    % Stuck Points
    [yIdx, xIdx] = find(stuckMask);
    h_stuck = scatter(xIdx, yIdx, 25, 'k', 'filled', 'DisplayName', 'Stuck Point');
    
    % Goal
    h_goal = plot(goal(1), goal(2), 'go', 'MarkerSize', 10, ...
        'LineWidth', 2, 'DisplayName', 'Goal');
    
    legend([h_vec, h_stuck, h_goal], ...
        {'Net Force', 'Stuck Point', 'Goal'}, ...
        'Location', 'northeastoutside');

    % Plot starting point and goal
    plot(startXY(1), startXY(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'r'); % start
    plot(goalXY(1), goalXY(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'g');   % goal
    
    % Plot a* path
    if ~isempty(path)                                              
        plot(path(:,1), path(:,2), 'k-', 'LineWidth', 1.5);    
    else                                                           
        warning('No path was found :(');                        
    end       

end
