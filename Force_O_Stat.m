function [F] = Force_O_Stat(t,u)
    global xObsList
    global yObsList
    global p
    global numObstacles
    global b
    global a_s
    global r_inf_s

    x = u(1:2,:);
    vel = u(3:4,:);
    F = zeros(2, p);
    for i = 1:p 
        for j = 1:numObstacles
            [d, x_d, y_d] = p_poly_dist(x(1,i), x(2,i), xObsList{j}, yObsList{j});
            dist_v = [x_d; y_d];
            if d <= r_inf_s
                F_ij = (a_s * exp(d)/b) * -dist_v;
                F(:,i) = F(:,i) + F_ij ;
            end
        end
    end
   
   F = F';
end