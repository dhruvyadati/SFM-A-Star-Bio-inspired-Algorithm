function [xObsList, yObsList] = GenerateEnvironmentGraphObject(xMax, yMax, numObstacles, maxArea, bounds)
%   numObstacles        number of obstacles
%   maxArea             max area per obstacle
%   bounds              [xMin, xMax, yMin, yMax]
%   xObsList, yObsList  cell arrays of obstacle vertices

    xObsList = {};
    yObsList = {};
    count = 0;
    maxTries = 10000;

    while count < numObstacles && maxTries > 0
        [xPoly, yPoly] = GenerateObstacle(maxArea, bounds);

        % Check for overlap with existing obstacles
        hasOverlap = false;
        for k = 1:numel(xObsList)
            [xi, yi] = polyxpoly(xPoly, yPoly, xObsList{k}, yObsList{k});
            if ~isempty(xi)
                hasOverlap = true;
                break;
            end
        end

        if ~hasOverlap
            count = count + 1;
            xObsList{count} = xPoly;
            yObsList{count} = yPoly;
        end

        maxTries = maxTries - 1;
    end
end

