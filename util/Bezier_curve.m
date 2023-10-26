function [coeff, d_coeff, dd_coeff] = Bezier_curve(t,N_cp)

    coeff = [];
    d_coeff = [];
    dd_coeff = [];

    n = N_cp -1;
    
    for i=0:n
        coeff = [coeff, nchoosek(n,i) * t^i * (1-t)^(n-i)];
    end

    for i=0:n-1
        d_coeff = [d_coeff, nchoosek(n-1,i) * t^i * (1-t)^(n-1-i)];
    end

    for i=0:n-2
        dd_coeff = [dd_coeff, nchoosek(n-2,i) * t^i * (1-t)^(n-2-i)];
    end

end