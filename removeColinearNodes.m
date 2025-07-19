function simplePath = removeColinearNodes(path)
    if size(path,1) < 3
        simplePath = path;
        return;
    end

    keep = true(size(path,1),1);
    for i = 2:size(path,1)-1
        p1 = path(i-1,:);
        p2 = path(i,:);
        p3 = path(i+1,:);

        v1 = p2 - p1;
        v2 = p3 - p2;

        % Check if vectors are colinear (cross product = 0 in 2D)
        if abs(v1(1)*v2(2) - v1(2)*v2(1)) < 1e-10
            keep(i) = false;
        end
    end

    simplePath = path(keep,:);
end
