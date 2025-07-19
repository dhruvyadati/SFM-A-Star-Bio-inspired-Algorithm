% Helper func: is point (px,py) inside any obstacle
% Also takes in the margin and checks points within that margin
function near = ptInObs(px,py, xObsList, yObsList, margin)
    near = false;
    for k = 1:numel(xObsList)
        [d, ~, ~] = p_poly_dist(px, py, xObsList{k}, yObsList{k});
        if d <= margin
            near = true;
            return;
        end
    end
end