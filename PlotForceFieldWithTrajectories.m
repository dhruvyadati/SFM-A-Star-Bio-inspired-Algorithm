function PlotForceFieldWithTrajectories()
    %% === Global Setup ===
    global goal t_r p v_max v_min xObsList yObsList numObstacles a b a_s r_inf r_inf_s

    % Parameters
    xMin = 1; xMax = 50;
    yMin = 1; yMax = 50;
    bounds = [xMin xMax yMin yMax];
    margin = 1;
    maxArea = 150;
    numObstacles = 10;
    goalXY = [xMax, yMax];  % for AStar
    goal = goalXY';         % for SFM

    % SFM constants
    t_r = 0.5;
    v_max = 4; v_min = 1;
    r_inf = 1; r_inf_s = 1.5;
    b = 1; a = 800; a_s = 8000;
    p = 1;

    %% === Generate Environment ===
    [xObsList, yObsList] = GenerateEnvironmentGraphObject(xMax, yMax, numObstacles, maxArea, bounds);

    %% === Compute Force Field ===
    [X, Y] = meshgrid(1:xMax, 1:yMax);
    U = zeros(size(X)); V = zeros(size(Y));
    stuckMask = false(size(X));

    for idx = 1:numel(X)
        x = X(idx); y = Y(idx);

        if ptInObs(x, y, xObsList, yObsList, 0)
            U(idx) = NaN; V(idx) = NaN;
            continue;
        end

        u_tmp = zeros(4,1);
        u_tmp(1:2) = [x; y];
        u_tmp(3:4) = [0; 0];

        Fg = Force_G(0, u_tmp);
        Fo = Force_O_Stat(0, u_tmp);
        Fnet = Fg + Fo;

        U(idx) = Fnet(1);
        V(idx) = Fnet(2);

        if norm(Fnet) < 0.3
            stuckMask(idx) = true;
        end
    end

    %% === Generate Graph + A* Path ===
    [G, nodeIDs, freeNodeIDs] = Graph(xMax, yMax, xObsList, yObsList, margin);

    startXY = [xMin, yMin];  % bottom-left
    goalXY = [xMax, yMax];   % top-right

    nodeIDs = reshape(1:(xMax * yMax), yMax, xMax);  % (row = y, col = x)
    startNode = nodeIDs(startXY(2), startXY(1));
    goalNode  = nodeIDs(goalXY(2), goalXY(1));

    [path, cameFrom, gScore] = AStar(G, nodeIDs, startXY, goalXY);  % path is 2xN

    %% === Simulate SFM-only ===
    start = startXY';  % as column vector
    u0 = [start; 0; 0];
    maxSteps = 3000;
    dt = 0.1;

    u_sfm = zeros(4, maxSteps);
    u_sfm(:,1) = u0;
    for t = 1:maxSteps-1
        u_curr = u_sfm(:,t);
        Fg = Force_G(0, u_curr);
        Fo = Force_O_Stat(0, u_curr);
        acc = Fg + Fo;
        vel_new = u_curr(3:4) + acc * dt;
        pos_new = u_curr(1:2) + vel_new * dt;
        u_sfm(:,t+1) = [pos_new; vel_new];
        if norm(pos_new - goal) < 0.5
            u_sfm = u_sfm(:, 1:t+1);
            break;
        end
    end

    %% === Simulate A* + SFM ===
    u_astar = zeros(4, maxSteps);
    u_astar(:,1) = u0;
    pathIdx = 1;
    numPathPoints = size(path,2);
    waypointThresh = 1.0;

    for t = 1:maxSteps-1
        u_curr = u_astar(:,t);
        currGoal = path(:, pathIdx);
        if norm(u_curr(1:2) - currGoal) < waypointThresh && pathIdx < numPathPoints
            pathIdx = pathIdx + 1;
            currGoal = path(:, pathIdx);
        end
        goal = currGoal;
        Fg = Force_G(0, u_curr);
        Fo = Force_O_Stat(0, u_curr);
        acc = Fg + Fo;
        vel_new = u_curr(3:4) + acc * dt;
        pos_new = u_curr(1:2) + vel_new * dt;
        u_astar(:,t+1) = [pos_new; vel_new];
        if norm(pos_new - path(:,end)) < 0.5
            u_astar = u_astar(:, 1:t+1);
            break;
        end
    end

    %% === Plot Everything ===
    figure; hold on; axis equal;
    xlim([xMin xMax]); ylim([yMin yMax]);
    xlabel('X'); ylabel('Y');
    title('SFM Field with A* and SFM Trajectories');
    grid on;

    for i = 1:numObstacles
        fill(xObsList{i}, yObsList{i}, 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'k');
    end

    quiver(X, Y, U, V, 1.3, 'b');
    [yIdx, xIdx] = find(stuckMask);
    scatter(xIdx, yIdx, 15, 'k', 'filled');

    if ~isempty(path)
        plot(path(1,:), path(2,:), 'c--', 'LineWidth', 2);
    end

    plot(u_sfm(1,:), u_sfm(2,:), 'm-', 'LineWidth', 2);
    plot(u_astar(1,:), u_astar(2,:), 'g-', 'LineWidth', 2);
    plot(start(1), start(2), 'ko', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    legend({'Obstacle','Net Force','Stuck','A* Path','SFM Only','A*+SFM','Start','Goal'}, 'Location', 'northeastoutside');
end
