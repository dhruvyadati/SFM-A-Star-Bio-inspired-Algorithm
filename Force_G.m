function [F] = Force_G(t,u)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

global goal
global t_r
global p
global v_max
global v_min


x = [u(1,:);u(2,:)];

vel = [u(3,:);u(4,:)];


x_g = goal - x;


for i = 1:p
    v_des(i) = norm(x_g(:,i));
    if v_des(i) > v_max
        v_des(i) = v_max;
    end
    if v_des(i) < v_min
        v_des(i) = v_min;
    end
end


F = (v_des.*x_g - vel)/t_r;
F = F';
end