% GenerateEnvironment creates obstacles and returns occupancy grid

function [grid, xObsList, yObsList] = GenerateEnvironment(xMax, yMax, numObstacles, maxArea, bounds)
%   xMax, yMax          dimensions of the environment
%   numObstacles        number of obstacles
%   maxArea             max area per obstacle
%   bounds              [xMin, xMax, yMin, yMax]
%   grid                occupancy grid (1 = free, 0 = obstacle)
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

    % Create occupancy grid
    % Creates matrix with size xMax x yMax (rows = y, columns = x)
    % Filled with 1s. Any coordinate occupied by obstacle is filled with 0.
    grid = ones(yMax, xMax);
    [X, Y] = meshgrid(1:xMax, 1:yMax);

    for i = 1:numObstacles
        in = inpolygon(X, Y, xObsList{i}, yObsList{i});
        grid(in) = 0;
    end
    %padding to prevent collision w/ obstacles 
    kernel  = ones(3);
    obsMask = (grid == 0);
    inflated = conv2(double(obsMask), kernel, 'same') > 0;
    grid(inflated) = 0;
end
