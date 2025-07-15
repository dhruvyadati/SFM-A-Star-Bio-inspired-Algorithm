function inside = isInsideAnyPolygon(x, y, xObsList, yObsList)
% Check if point (x, y) is inside any obstacle polygon

    inside = false;
    for i = 1:length(xObsList)
        if inpolygon(x, y, xObsList{i}, yObsList{i})
            inside = true;
            return;
        end
    end
end
