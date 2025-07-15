% GenerateEnvironment creates obstacles and returns occupancy grid

function [xObsList, yObsList] = GenerateEnvironment(xMax, yMax, numObstacles, maxArea, bounds)
%   xMax, yMax          dimensions of the environment
%   numObstacles        number of obstacles
%   maxArea             max area per obstacle
%   bounds              [xMin, xMax, yMin, yMax]
%   xObsList, yObsList  cell arrays of obstacle vertices

    xObsList = cell(numObstacles, 1);
    yObsList = cell(numObstacles, 1);
    count = 0;

    while count < numObstacles
        try
            [xPoly, yPoly] = GenerateObstacle(maxArea, bounds);
            count = count + 1;
            xObsList{count} = xPoly;
            yObsList{count} = yPoly;
        catch
            continue;
        end
    end
end
