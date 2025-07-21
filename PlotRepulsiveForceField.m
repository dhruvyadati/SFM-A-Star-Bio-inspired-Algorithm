function PlotRepulsiveForceField()
    % Visualize the repulsive static obstacle forces on a 50x50 grid.
    % Optional: overlay net force (goal + repulsion).
    
    global goal t_r p v_max v_min xObsList yObsList numObstacles a b a_s r_inf r_inf_s

    %% Params
    xMin = 1; xMax = 50;
    yMin = 1; yMax = 50;
    bounds = [xMin xMax yMin yMax];
    margin = 0;
    maxArea = 150;
    numObstacles = 10;
    goal = [xMax; yMax];  % top-right goal

    % SFM constants
    t_r = 0.5; v_max = 4; v_min = 1;
    r_inf = 1; r_inf_s = 1.5;
    b = 1; a = 800; a_s = 10000;
    p = 1;

    %% Environment
    [xObsList, yObsList] = GenerateEnvironmentGraphObject(xMax, yMax, numObstacles, maxArea, bounds);

    % Grid
    [X, Y] = meshgrid(1:0.25:xMax, 1:0.25:yMax);
    U_rep = zeros(size(X));
    V_rep = zeros(size(Y));
    U_net = zeros(size(X));
    V_net = zeros(size(Y));
    stuckMask = false(size(X));

    %% Compute Forces
    for i = 1:numel(X)
        x = X(i); y = Y(i);
        if ptInObs(x, y, xObsList, yObsList, margin)
            U_rep(i) = NaN; V_rep(i) = NaN;
            U_net(i) = NaN; V_net(i) = NaN;
            continue;
        end

        u_tmp = zeros(4,1); u_tmp(1:2) = [x; y];
        Fo = Force_O_Stat(0, u_tmp);
        Fg = Force_G(0, u_tmp);
        Fnet = Fo + Fg;

        U_rep(i) = Fo(1); V_rep(i) = Fo(2);
        U_net(i) = Fnet(1); V_net(i) = Fnet(2);

        if norm(Fnet) < 1e-2
            stuckMask(i) = true;
        end
    end

    %% Plotting
    figure; hold on; axis equal;
    xlim([xMin xMax]); ylim([yMin yMax]);
    xlabel('X'); ylabel('Y');
    title('Repulsive Forces from Obstacles and Stuck Points');
    grid on;

    % Obstacles
    for i = 1:numObstacles
        fill(xObsList{i}, yObsList{i}, 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'k', ...
            'DisplayName', 'Obstacle');
    end

    % Repulsive force vectors (blue)
    h_rep = quiver(X, Y, U_rep, V_rep, 1.5, 'b', 'DisplayName', 'Obstacle Repulsion');

    % Net force vectors (gray)
    h_net = quiver(X, Y, U_net, V_net, 1.5, 'Color', [0.3 0.3 0.3], ...
        'LineStyle', '--', 'DisplayName', 'Net Force');

    % Stuck points
    [yIdx, xIdx] = find(stuckMask);
    h_stuck = scatter(xIdx, yIdx, 20, 'k', 'filled', 'DisplayName', 'Stuck Point');

    % Goal
    h_goal = plot(goal(1), goal(2), 'go', 'MarkerSize', 10, ...
        'LineWidth', 2, 'DisplayName', 'Goal');

    legend([h_rep, h_net, h_stuck, h_goal], ...
        {'Obstacle Repulsion', 'Net Force', 'Stuck Point', 'Goal'}, ...
        'Location', 'northeastoutside');
end
