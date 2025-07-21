function [F] = Force_O_Dyn(t,u)
    global goal
    global p
    global a
    global b
    global r_inf

    x = u(1:2,:);
    vel = u(3:4,:);

    F = zeros(2, p);
    for i = 1:p 
        for j = 1:p
            
            if j == i
                continue
            end
            
            dist_v = x(:,i) - x(:,j);
            
            dist = norm(dist_v);
            
            if dist > r_inf
                continue
            end
           
            F_ij = (a * exp(-dist)/b) * dist_v;
            F(:,i) =  F(:,i) + F_ij;
            

        end
    end
    F = F';
end