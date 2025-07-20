function F = Force_O_Stat(~, u, useSFM)
if nargin < 3
    useSFM = true(1, size(u, 2));
end

global xObsList yObsList numObstacles p a_s b r_inf_s

x = u(1:2,:);
F = zeros(2, p);

for i = 1:p
    if ~useSFM(i), continue, end
    for j = 1:numObstacles
        [d, xc, yc] = p_poly_dist(x(1, i), x(2, i), xObsList{j}, yObsList{j});
        if d > r_inf_s, continue, end
        dv = [xc; yc] - x(:, i);
        F(:, i) = F(:, i) - (a_s * exp(-d) / b) * dv / max(d, 1e-6);
    end
end

F = F.';
end
