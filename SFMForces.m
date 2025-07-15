function [dudt] = SFMForces(t,u)

global p %number of agents in the crowd
global g %goal vectors for each agent
global v0 %intial speed
global A_g %coefficient of goal force
global B_o_s % coefficient of static obstacle avoidance %
global xObsList % 
global yObsList
global R_inf


% takes in input state u and extracts position vector
for i = 1:p
   x(i,1) = u(i,1);
   x(i,2) = u(i,2);
end

%takes input state u and extracts current velocity
for i = 1:p
    vel(i,1) = u(i,3);
    vel(i,2) = u(i,4);
end

%calculate Force vector towards the navigation goal
for i = 1:p
    V_g(i,:) = g(i,:) - x(i,:);
    V_g(i,:) = V_g(i, :)/(norm(V_g(i,:)));
    F_g = A_g * V_g;
end

%calculate repulsion from static regions
for k = 1:p
    num_obj = 0
    for i = 1:len(xObsList)
        for j = 1:len(xObsList{i})
            v = [xObsList{i},yObsList{i}];
            if (norm(x(k,:) - v(j,j)) <= R_inf)
                num_obj = num_obj + 1;
                Obs_list(k,num_obj) = i;
                break
            end
        end
    end
end

for k = 1:p
    [d(k), v_o_x(k), v_o_y(k)] = p_poly_dist(x(k,1), x(k,2), v(1,:), v(2,:));
end

v_o = [v_o_x; v_o_y];

v_o = -1* v_o;

for i = 1:len(v_o)
    v_o(i,:) = v_o(i,:)/(norm(v_o(i,:)));
    v_o(i,:) = v_o(i,:) * (1/d(i)); 
end

F_o_s = v_o(i,:) * B_o_s

% dynamic obstacle avoidance
% cook up smth 

F = F_o_s + F_g

for i=1:p
    dudt(i,1:2) = vel(i,:);
    dudt(i,3:4) = F(i,:);
end