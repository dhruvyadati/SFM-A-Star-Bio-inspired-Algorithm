function F = Force_O_Dyn(~, u, useSFM)
    if nargin < 3
        useSFM = true(1, size(u,2)); 
    end
    global p a b r_inf
    x = u(1:2,:);                     
    F = zeros(2, p);

    for i = 1:p
        if ~useSFM(i), continue, end    
        for j = 1:p
            if j == i, continue, end
            dvec = x(:,i) - x(:,j);
            dist = norm(dvec);
            if dist > r_inf, continue, end
            F(:,i) = F(:,i) + (a * exp(-dist) / b) * (dvec / max(dist,1e-6));
        end
    end
    F = F.';                             
end
