clearvars; close all; clc;

maxArea = 150;
xMin = 1; xMax = 50; yMin = 1; yMax = 50;
bounds = [xMin xMax yMin yMax];
margin = 1;

[xObsList, yObsList] = GenerateEnvironmentGraphObject(xMax, yMax, 14, maxArea, bounds);

global goal t_r p v_max v_min a b r_inf xObsList yObsList numObstacles a_s r_inf_s
t_r  = 0.5; v_max = 20; v_min = 5;
a    = 400; b = 1;  r_inf   = 0.75;
a_s  = 4000; r_inf_s = 1;
numObstacles = 14;

[G, nodeIDs, freeNodeIDs] = Graph(xMax, yMax, xObsList, yObsList, margin);
startXY = [xMin yMin];  goalXY = [xMax yMax];

[path, ~, ~] = AStar(G, nodeIDs, startXY, goalXY);
path  = removeColinearNodes(path).';
K     = size(path, 2);

p    = 100;
goal = zeros(2, p);
for k = 3:p
    while true
        x = (xMax - 1) * rand() + 1;
        y = (yMax - 1) * rand() + 1;
        if ~ptInObs(x, y, xObsList, yObsList, margin)
            goal(:, k) = [x; y];
            break
        end
    end
end
goal(:, 1) = path(:, 1);
goal(:, 2) = goalXY.';

u0 = zeros(4, p);
u0(:, 1) = [2; 2; 1; 0];
u0(:, 2) = [2; 3; 1; 0];
for k = 3:p
    while true
        x = (xMax - 1) * rand() + 1;
        y = (yMax - 1) * rand() + 1;
        if ~ptInObs(x, y, xObsList, yObsList, margin)
            u0(:, k) = [x; y; randn(); randn()];
            break
        end
    end
end
useSFM = true(1, p);

figure; hold on; axis equal; grid on
xlabel('X'); ylabel('Y');
title('Hybrid A*+SFM (blue) vs SFM-only (purple)');
xlim([xMin xMax]); ylim([yMin yMax]);

for k = 1:numObstacles
    fill(xObsList{k}, yObsList{k}, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'k');
end
[Yg, Xg] = ndgrid(1:yMax, 1:xMax);
plot(G, 'XData', Xg(:), 'YData', Yg(:), 'NodeLabel', {}, ...
     'LineStyle', '-', 'EdgeColor', [.85 .85 .85], 'Marker', 'none');
plot(startXY(1), startXY(2), 'or', 'MarkerFaceColor', 'k');
plot(goalXY(1), goalXY(2), 'og', 'MarkerFaceColor', 'k');
plot(path(1, :), path(2, :), 'k-', 'LineWidth', 1.2);

hPed = plot(u0(1, 3:p), u0(2, 3:p), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
hR1  = plot(u0(1, 1),   u0(2, 1),   'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
hR2  = plot(u0(1, 2),   u0(2, 2),   'mo', 'MarkerSize', 6, 'MarkerFaceColor', 'm');

dt = 0.01; Nmax = 15000; i = 1;
u = zeros(4, p, Nmax); u(:, :, 1) = u0;
wayIdx = 1; tol = 0.3;

safetyR = 0.3; comfortR = 0.6;
minSep1 = inf; minSep2 = inf; coll1 = 0; coll2 = 0; intr1 = 0; intr2 = 0;

while i < Nmax
    if norm(u(1:2, 1, i) - goal(:, 1)) < tol && wayIdx < K
        wayIdx  = wayIdx + 1;
        goal(:, 1) = path(:, wayIdx);
    end

    F = Force_G([], u(:, :, i)) + ...
        Force_O_Stat([], u(:, :, i), useSFM) + ...
        Force_O_Dyn([], u(:, :, i), useSFM);

    u(3:4, :, i+1) = u(3:4, :, i) + F' * dt;
    u(1:2, :, i+1) = u(1:2, :, i) + u(3:4, :, i+1) * dt;

    v1 = norm(u(3:4, 1, i+1));
    if v1 > v_max
        u(3:4, 1, i+1) = u(3:4, 1, i+1) / v1 * v_max;
    end

    i = i + 1;

    crowd = squeeze(u(1:2, 3:p, i));
    d1 = vecnorm(crowd - u(1:2, 1, i));
    d2 = vecnorm(crowd - u(1:2, 2, i));
    minSep1 = min(minSep1, min(d1));  minSep2 = min(minSep2, min(d2));
    coll1   = coll1 + sum(d1 < safetyR); coll2 = coll2 + sum(d2 < safetyR);
    intr1   = intr1 + sum(d1 < comfortR); intr2 = intr2 + sum(d2 < comfortR);

    if mod(i, 5) == 0
        set(hPed, 'XData', u(1, 3:p, i), 'YData', u(2, 3:p, i));
        set(hR1,  'XData', u(1, 1, i),   'YData', u(2, 1, i));
        set(hR2,  'XData', u(1, 2, i),   'YData', u(2, 2, i));
        drawnow limitrate
    end

    if norm(u(1:2, 1, i) - goalXY') < tol && norm(u(1:2, 2, i) - goalXY') < tol
        break
    end
end
u = u(:, :, 1:i); N = i;

M1 = Metrics(u(:, 1, :), path, dt);
M2 = Metrics(u(:, 2, :), path, dt);

framesTot = (p - 2) * N;
M1.minSep         = minSep1;
M1.collisionCount = coll1;
M1.intrusionFrac  = intr1 / framesTot;

M2.minSep         = minSep2;
M2.collisionCount = coll2;
M2.intrusionFrac  = intr2 / framesTot;

disp('Hybrid A* + SFM (blue)');   disp(M1)
disp('SFM-only baseline (purple)'); disp(M2)
