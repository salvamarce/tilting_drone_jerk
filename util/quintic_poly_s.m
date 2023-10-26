function [p, v, acc] = quintic_poly_s(a,s)
    
    p = [];
    v = [];
    acc = [];

    for i = 1:length(s)
        tau = s(i);
        p = [p, a(6)*(tau^5) + a(5)*(tau^4) + a(4)*(tau^3) + ...
            a(3)*(tau^2) + a(2)*tau + a(1)];
        
        v = [v, 5*a(6)*(tau^4) + 4*a(5)*(tau^3) + 3*a(4)*(tau^2) + ...
            2*a(3)*tau + a(2)];
        
        acc = [acc, 20*a(6)*(tau^3) + 12*a(5)*(tau^2) + 6*a(4)*tau + ...
              2*a(3)];
    end

end