function [p, v, acc] = quintic_poly_traj(a,T_tot,dt)
    
    T = T_tot;
    N = T / dt;
    t = 0:dt:T;

    p = [];
    v = [];
    acc = [];

    for i=1:N

        if( t(i) < T)
            tau = t(i)/T;
        else
            tau = 1;
        end
        

        p = [p, a(6)*(tau^5) + a(5)*(tau^4) + a(4)*(tau^3) + ...
                a(3)*(tau^2) + a(2)*tau + a(1)];
        
        v = [v, 5*a(6)*(tau^4) + 4*a(5)*(tau^3) + 3*a(4)*(tau^2) + ...
                2*a(3)*tau + a(2)];
        
        acc = [acc, 20*a(6)*(tau^3) + 12*a(5)*(tau^2) + 6*a(4)*tau + ...
                     2*a(3)];
    end

end