function F = Force_G(~, u)
global goal t_r p v_max

pos  = u(1:2,:);
vel  = u(3:4,:);
dir  = goal - pos;
dist = vecnorm(dir);
dir  = dir ./ max(dist, 1e-9);
v_des = min(v_max, dist / t_r);

F = (v_des .* dir - vel) / t_r;
F = F.';
end
