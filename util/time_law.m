function [s, s_dot, s_ddot] = time_law(cp, N_cp, dt, N)

    s = []; s_dot = []; s_ddot=[];
    t_law = linspace(0,1,N);
    
    for i_s = 1:N
        t = t_law(i_s);
        [s_coeff, ~, ~] = Bezier_curve(t,N_cp);
    
        s = [s, s_coeff*cp];
        
        if(i_s>1)
            s_dot = [s_dot, ( s(i_s)-s(i_s-1) )/dt];
        else
            s_dot = [s_dot, 0.0];
        end
        
        if(i_s>2)
            s_ddot = [s_ddot, ( s_dot(i_s)-s_dot(i_s-1) )/dt];
        else
            s_ddot = [s_ddot, 0.0];
        end
    
    end

end
