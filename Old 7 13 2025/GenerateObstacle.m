% GenerateObstacle generates a random polygonal obstacle

function [xPoly, yPoly] = GenerateObstacle(areaMax, bounds)
%   areaMax         Max polygon area
%   bounds          [xMin, xMax, yMin, yMax]
%   xPoly, yPoly    Polygon vertex coordinates

    % Number of vertices randomly between 3 and 10
    numVertices = randi([3, 10]);
    xMin = bounds(1); xMax = bounds(2);
    yMin = bounds(3); yMax = bounds(4);

    % Max radius based on area constraint
    maxRadius = sqrt(areaMax / pi);

    maxAttempts = 1000;
    for attempt = 1:maxAttempts
        % Random center within safe margin so polygon stays in bounds
        margin = maxRadius;
        xCenter = xMin + margin + (xMax - xMin - 2 * margin) * rand;
        yCenter = yMin + margin + (yMax - yMin - 2 * margin) * rand;

        angles = sort(rand(numVertices, 1) * 2 * pi);
        radii = maxRadius * (0.4 + 0.6 * rand(numVertices, 1));  % Radius variation

        xPoly = xCenter + radii .* cos(angles);
        yPoly = yCenter + radii .* sin(angles);

        xPoly(end+1) = xPoly(1);
        yPoly(end+1) = yPoly(1);

        % Shoelace formula for area & check area and bounds
        A = 0.5 * abs(sum(xPoly(1:end-1).*yPoly(2:end) - xPoly(2:end).*yPoly(1:end-1)));
        
        if A <= areaMax && ...
           all(xPoly >= xMin) && all(xPoly <= xMax) && ...
           all(yPoly >= yMin) && all(yPoly <= yMax)
            return;
        end
    end
end
