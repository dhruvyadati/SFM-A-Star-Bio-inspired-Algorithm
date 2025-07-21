function [xObsList, yObsList] = GenerateEnvironmentGraphObject(xMax, yMax, numObstacles, maxArea, bounds)
%   numObstacles        number of obstacles
%   maxArea             max area per obstacle
%   bounds              [xMin, xMax, yMin, yMax]
%   xObsList, yObsList  cell arrays of obstacle vertices

    rng(145, "twister");

    xObsList = {};
    yObsList = {};
    count = 0;
    maxTries = 10000;

    while count < numObstacles && maxTries > 0
        [xPoly, yPoly] = GenerateObstacle(maxArea, bounds);

        % Check for overall overlap
        hasConflict = false;
        for k = 1:numel(xObsList)

            % Check if edges intersect
            [xi, yi] = polyxpoly(xPoly, yPoly, xObsList{k}, yObsList{k});
            if ~isempty(xi)
                hasConflict = true;
                break;
            end

            
            % This basically checks if there is any polygon inside another
            % polygon, it does not go through the check

            % Check if any point of new poly is inside existing poly
            if any(inpolygon(xPoly(1:end-1), yPoly(1:end-1), xObsList{k}, yObsList{k}))
                hasConflict = true;
                break;
            end

            % Check if any point of existing poly is inside new poly
            if any(inpolygon(xObsList{k}(1:end-1), yObsList{k}(1:end-1), xPoly, yPoly))
                hasConflict = true;
                break;
            end
        end

        if ~hasConflict
            count = count + 1;
            xObsList{count} = xPoly;
            yObsList{count} = yPoly;
        end

        maxTries = maxTries - 1;
    end
end
