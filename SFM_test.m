global goal 
global t_r
global p
global v_max
global a
global b
global r_inf
global v_min
global xObsList
global yObsList
global numObstacles
global a_s
global r_inf_s

%goal = [10, 1, 3;10, 1, 8];

p = 100;
goal = zeros(2, p);

t_r = 0.5;
v_max = 5;
v_min = 2;
r_inf = 0.75;
r_inf_s = 1;
b = 1;
a = 400;
a_s = 2000;
numObstacles = 5;

margin = 1;
xMin = 0;
xMax = 30;
yMin = 0;
yMax = 30;
bounds = [xMin, xMax, yMin, yMax];
maxArea = 40;
 

[xObsList, yObsList] = GenerateEnvironmentGraphObject(xMax, yMax, numObstacles, maxArea, bounds);

for i = 1:p
    while true
        xRand = xMax * rand();
        yRand = xMax * rand();
        if ~ptInObs(xRand, yRand, xObsList, yObsList, margin)
            goal(:, i) = [xRand; yRand];
            break;
        end
    end
end



hold on
for i = 1:numObstacles
    fill(xObsList{i}, yObsList{i}, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'k');
end


u_0 = zeros(4,p);
u_0 = 28 * rand(size(u_0));

% u_0_1 = [2; 2; 1; 1];
% u_0_2 = [11; 12;  -1; -1 ];
% u_0_3 = [10; 9; 3; -2];
% u_0 = [u_0_1, u_0_2, u_0_3];

t = [];
u = zeros(4,p,1000);
u(:,:,1) = u_0;
dt = 0.01;
i = 1;





axis([-2 30 -2 30]);
plot(goal(1,:), goal(2,:), 'o', color= 'Green');
%plot 



h = plot(u_0(:, 1), u_0(:,2), 'o');

while any(any(abs(u(1:2,:,i)-goal) > 0.1) ~= 0)
    i = i+1;
    F = Force_G(t,u(:,:,i-1)) + Force_O_Stat(t,u(:,:,i-1)) + Force_O_Dyn(t,u(:,:,i-1));
    
    u(1:2,:, i ) = u(1:2,:, i-1) + u(3:4,:, i-1) * dt + 1/2 * F' * dt^2;
    u(3:4,:, i ) = u(3:4,:, i -1) + F' * dt; 

    if norm(u(3:4,:,i)) > v_max 
        u(3:4,:,i) = u(3:4,:,i) / norm(u(3:4,:,i)) * v_max; 
    end

    if norm(u(3:4,:,i)) < v_min 
        u(3:4,:,i) = u(3:4,:,i) / norm(u(3:4,:,i)) * v_min; 
    end
    
   
    
    if i > 10000
        break
    end
   


    x = squeeze(u(1,:,i));
    y = squeeze(u(2,:,i));
    set(h, 'XData', x, color="red");
    set(h, 'YData', y);
    
    pause(0.01);

end