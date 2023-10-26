function [p, v, acc] = seventh_order_poly_traj_SX(pin,pfin,a,N, T_tot)
    
    p = [];
    v = [];
    acc = [];
    t = linspace(0,1,N);
    T = T_tot;
    
    for i=1:N

        tau = t(i);

        p = [p, pin + (pfin-pin)*(a(1)*(tau^7) + a(2)*(tau^6) + a(3)*(tau^5) + ...
                    a(4)*(tau^4) + a(5)*(tau^3) + a(6)*(tau^2) + ...
                    a(7)*tau + a(8))];
        
        v = [v, ((pfin-pin)/T)*(7*a(1)*(tau^6) + 6*a(2)*(tau^5) + 5*a(3)*(tau^4) + ...
                4*a(4)*(tau^3) + 3*a(5)*(tau^2) + 2*a(6)*tau + ...
                a(7))];
        
        acc = [acc, ((pfin-pin)/(T^2))*(42*a(1)*(tau^5) + 30*a(2)*(tau^4) + 20*a(3)*(tau^3) + ...
                12*a(4)*(tau^2) + 6*a(5)*tau + 2*a(6))];
    end

end