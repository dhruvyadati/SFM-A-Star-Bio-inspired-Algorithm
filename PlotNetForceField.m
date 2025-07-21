function PlotNetForceField()
    % Plot the vector field of the net force at each free point on the 50x50 grid.

    % Global parameters (same as rest of the project)
    global goal t_r p v_max v_min xObsList yObsList numObstacles a b a_s r_inf r_inf_s

    %% Setup
    xMin = 1; xMax = 50;
    yMin = 1; yMax = 50;
    bounds = [xMin xMax yMin yMax];
    margin = 0;
    maxArea = 150;
    numObstacles = 10;
    goal = [xMax; yMax];  % Assume fixed goal at top-right

    % SFM params
    t_r = 0.5;
    v_max = 4;
    v_min = 1;
    r_inf = 1;
    r_inf_s = 1.5;
    b = 1;
    a = 800;
    a_s = 10000;

    p = 1; % dummy agent (only one agent at a time for force evaluation)

    %% Generate Environment
    [xObsList, yObsList] = GenerateEnvironmentGraphObject(xMax, yMax, numObstacles, maxArea, bounds);

    % Grid of points
    [X, Y] = meshgrid(1:0.25:xMax, 1:0.25:yMax);
    U = zeros(size(X));
    V = zeros(size(Y));
    stuckMask = false(size(X));

    %% Compute Forces at each grid point
    for i = 1:numel(X)
        x = X(i);
        y = Y(i);

        if ptInObs(x, y, xObsList, yObsList, margin)
            U(i) = NaN;
            V(i) = NaN;
            continue;
        end

        % Dummy agent at this location, no velocity
        u_tmp = zeros(4,1);
        u_tmp(1:2) = [x; y];
        u_tmp(3:4) = [0; 0];

        Fg = Force_G(0, u_tmp);         % Goal attraction
        Fo = Force_O_Stat(0, u_tmp);    % Static obstacle repulsion
        F = Fg + Fo;

        U(i) = F(1);
        V(i) = F(2);

        % Local minima (very weak net force)
        if norm(F) < 1e-2
            stuckMask(i) = true;
        end
    end

    %% Plot Vector Field
    figure;
    hold on;
    axis equal;
    xlim([xMin xMax]);
    ylim([yMin yMax]);
    title('Net Force Field (Goal + Static Obstacle Repulsion)');
    xlabel('X'); ylabel('Y');
    grid on;

    % Plot obstacles
    for i = 1:numObstacles
        fill(xObsList{i}, yObsList{i}, 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'k');
    end

    % Plot force vectors
    quiver(X, Y, U, V, 1.5, 'b');

    % Highlight stuck regions (zero-ish net force)
    [yIdx, xIdx] = find(stuckMask);
    scatter(xIdx, yIdx, 20, 'k', 'filled', 'DisplayName', 'Local Minima (Stuck Points)');

    % Plot goal
    plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Goal');

    legend('Obstacle', 'Force Vector', 'Stuck Point', 'Goal', 'Location', 'northeastoutside');

end
