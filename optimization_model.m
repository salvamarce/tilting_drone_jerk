function [] = optimization_model(ref_traj, dt, lb_states, ub_states)
    
    addpath('casadi_matlab')
    import casadi.*
    
    addpath('cpp_files')
    addpath('util')
    drone_settings

    %% Initialization
    t=0;
    N_traj = size(ref_traj,2);
       
    wr0 = (mass*9.81/(N_rotors*Kf)).*ones(N_rotors,1);
    wr_old = wr0;
    tilt0 = deg2rad(0)*[-1;0;1;-1;0;1].*ones(N_rotors,1);
    tilt_des0 = zeros(N_rotors,1);
    x0 = [ref_traj(1:9,1);deg2rad(0);deg2rad(0);ref_traj(15:21,1);tilt0(:,1)];
    ctrl0 = zeros(2*N_rotors, 1);
    PI0 = zeros(N_states,2,1);
    PI_csi0 = zeros(N_rotors,2,1);
    
    var = {};
    var0 = []; %variable's evolution vector
    lb_var = []; ub_var = []; %States bounds
    g={}; % constraints
    lbg = []; ubg = []; %constraints bounds
    J = 0; % cost function

    %% Initial conditions
    cp_s = SX.sym('cp_s',N_cp,1);
    var = {var{:}, cp_s};
    lb_var = [lb_var; [0.0; -100*ones(N_cp-1,1)]];
    ub_var = [ub_var; [0.0;  100*ones(N_cp-1,1)]];
    var0 = [var0; zeros(N_cp,1)];
    
    t_traj = linspace(0,1,N_traj);

    [z_opti,~,~] = time_law(cp_s, N_cp, dt, N_traj);

    %Initial state
    Xk = SX.sym('X0', N_states);
    var = {var{:}, Xk};
    lb_var = [lb_var; x0];
    ub_var = [ub_var; x0];
    var0 = [var0; x0];
    
    %% Next states
    for i=2:N_traj
   
        wr_old = wr_evo(:,i-1);

    R_des = Rz(r_des(12))*Ry(r_des(11))*Rx(r_des(10));
    q_des = rotm2quat(R_des);

    r = [r_des(1:9);q_des];

    %%% Input 
    n_des_k = SX.sym(['n_des_',num2str(i)], N_rotors);
    w = {w{:}, n_des_k}; 
    if(use_tubes)
        lbw = [lbw; minPropSpeedsq + u_tube(:,i+1)]; 
        ubw = [ubw; maxPropSpeedsq - u_tube(:,i+1)]; 
    else
        lbw = [lbw; minPropSpeedsq * ones(N_rotors,1)];
        ubw = [ubw; maxPropSpeedsq * ones(N_rotors,1)];
    end
    w0 = [w0; guess.rotor_vel_evo(:,i+1)]; %input initial guess

    alpha_des_k = SX.sym(['alpha_des_',num2str(i)], N_rotors);
    w = {w{:}, alpha_des_k}; 
    if(use_tubes)
        lbw = [lbw; -deg2rad(alpha_minmax) + alpha_tube(:,i+1)]; 
        ubw = [ubw;  deg2rad(alpha_minmax) - alpha_tube(:,i+1)]; 
    else
        lbw = [lbw; -deg2rad(alpha_minmax) * ones(N_rotors,1)]; 
        ubw = [ubw;  deg2rad(alpha_minmax) * ones(N_rotors,1)];
    end
    w0 = [w0; guess.alpha_evo(:,i+1)]; %input initial guess
    
    [n_des, alpha_des, f_ctrl] = CTRL(Xk,csi0,r,p0,cntrl_gains);
    %%%%%%%%%

    %%% State
    Xk_next = SX.sym(['X_' num2str(i+1)], N_states);
    w = [w, {Xk_next}];
    lbw = [lbw; lb_states];
    ubw = [ubw; ub_states];
    w0 = [w0; guess.x_evo(:,i+1)]; %state initial guess
    
    [k1, F_M] = X_DOT(Xk,p0,n_des, alpha_des,f_ext(:,i+1));
    x_next = Xk + k1 * (Tf_opt/N);

    csi_dt = CSI_DOT(x0,r,cntrl_gains);
    csi_next = csi0 + csi_dt * (Tf_opt/N);
    %%%%%%%%%
    
    if(use_tubes)

        PI_U_k = PI_U(Xk,csi0,PI0,PI_csi0,r,p0, cntrl_gains, n_des, alpha_des,f_ext(:,i+1));
        K_pi_u = diag(PI_U_k(1:N_rotors,:)*delta_mat*PI_U_k(1:N_rotors,:)');
    
        Pk1 = PI_DOT(Xk,csi0,PI0,PI_csi0,r,p0, cntrl_gains, n_des, alpha_des,f_ext(:,i+1));
        Pcsi_dt = PI_CSI_DOT(Xk,csi0,PI0,PI_csi0,r,p0, cntrl_gains, n_des, alpha_des,f_ext(:,i+1));
    
        PI_next = PI0 + Pk1 * dt;
        PI_csi_next = PI_csi0 + Pcsi_dt * dt;
    
        K_pi = diag(PI0*delta_mat*PI0');
    end

    % Forces ellipsoid
    ax_sq = AX_SQ(Xk,p0,n_des,alpha_des,f_ext(:,i+1));
    ay_sq = AY_SQ(Xk,p0,n_des,alpha_des,f_ext(:,i+1));
    
    %%%%%%%%%

    %%% Equality constraints   
    % Input 
    g = [g, {n_des_k - n_des}];
    lbg = [lbg; zeros(N_rotors,1)];
    ubg = [ubg; zeros(N_rotors,1)];

    g = [g, {alpha_des_k - alpha_des}];
    lbg = [lbg; zeros(N_rotors,1)];
    ubg = [ubg; zeros(N_rotors,1)];

    g = [g, { (alpha_des_k - alpha_des_k_old)/dt }];
    lbg = [lbg; -alpha_dot_minmax .* ones(N_rotors,1)];
    ubg = [ubg; alpha_dot_minmax .* ones(N_rotors,1)];

    % State
    g = [g, {Xk_next - x_next}];
    lbg = [lbg; zeros(N_states,1)];
    ubg = [ubg; zeros(N_states,1)];

    %%%%%%%%%
    
    % Cost
    J_int = J_int  + 1e-8*norm(n_des)^2 + 1e3*norm(alpha_des)^2;

    Xk = Xk_next;
    csi0 = csi_next;
    alpha_des_k_old = alpha_des_k;

    if(use_tubes)
        PI0 = PI_next;
        PI_csi0 = PI_csi_next;
    end
    

end


end