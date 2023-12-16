function [var0, var, ub_var, lb_var, constr, ub_constr, lb_constr, cost] = single_shooting(ref_traj, dt, lb_states, ub_states)
    
    addpath('casadi_matlab')
    import casadi.*
    
    addpath('cpp_files')
    addpath('util')
    
    ctrl_fun = controller_sym();
    system_fun = drone_model_sym();

    drone_settings
    parameters

    %% Initialization
    t=0;
    N_traj = size(ref_traj,2);
    
    var = {};
    var0 = []; %variable's evolution vector
    lb_var = []; ub_var = []; %States bounds
    constr={}; % constraints
    lb_constr = []; ub_constr = []; %constraints bounds
    J_int = 0;
    cost = 0; % cost function

    %% Initial conditions
    cp_s = SX.sym('cp_s',N_cp,2*N_rotors);
    for i_cp=1:2*N_rotors
        var = {var{:}, cp_s(:,i_cp)};
        lb_var = [lb_var; [0.0; -100*ones(N_cp-1,1)]];
        ub_var = [ub_var; [0.0;  100*ones(N_cp-1,1)]];
        var0 = [var0; zeros(N_cp,1)];
    end

    z_opti = [];

    for i_z = 1:2*N_rotors
        [temp,~,~] = time_law(cp_s(:,i_z), N_cp, dt, N_traj);
        z_opti = [z_opti; temp];
    end

    wr0 = (mass*9.81/(N_rotors*Kf)).*ones(N_rotors,1);
    tilt0 = deg2rad(0)*[-1;0;1;-1;0;1].*ones(N_rotors,1);
    x0 = [ref_traj(1:9,1);ref_traj(13:21,1);tilt0(:,1)];
    PI0 = zeros(N_states,2);
    PI_csi0 = zeros(N_rotors,2);
    tilt_des_old = zeros(N_rotors,1);
    
    Xk = x0;
    %% Next states
    for i=2:N_traj 

        % Input 
        [wr_dot_des, w_tilt_des] = ctrl_fun(Xk,wr0,K_lin,K_att,ref_traj(1:12,i),ref_traj(13:end,i), params, z_opti(:,i));

        % State
        tilt_des_next = tilt_des_old + w_tilt_des * dt;
        [pos_dot,vel_dot,acc_dot,eul_dot,wB_dot,wB_ddot,w_tilt] = system_fun(Xk,wr0,wr_dot_des,w_tilt_des,params,tilt_des_next);
        
        acc_next = Xk(7:9) + acc_dot * dt;
        vel_next = Xk(4:6) + acc_next * dt;
        pos_next = Xk(1:3) + vel_next * dt;
        w_dot_next = Xk(16:18) + wB_ddot * dt;
        w_next = Xk(13:15) + wB_dot * dt;
        eul_next = Xk(10:12) + eul_dot * dt;
        tilt_next = Xk(19:end) + w_tilt * dt;
        
        Xk = [pos_next; vel_next; acc_next; eul_next; w_next; w_dot_next; tilt_next];

        % csi_dot = cl_sens('CSI_FUN', wr_dot_des);
        
        %%%%%%%%%
    
        % if(use_tubes)
        % 
        %     PI_U_k = PI_U(Xk,csi0,PI0,PI_csi0,r,p0, cntrl_gains, n_des, alpha_des,f_ext(:,i+1));
        %     K_pi_u = diag(PI_U_k(1:N_rotors,:)*delta_mat*PI_U_k(1:N_rotors,:)');
        % 
        %     Pk1 = PI_DOT(Xk,csi0,PI0,PI_csi0,r,p0, cntrl_gains, n_des, alpha_des,f_ext(:,i+1));
        %     PI_dt = cl_sens('PI_FUN', PI_evo(:,:,i-1),PI_csi_evo(:,:,i-1),x0, wr0, wr_dot_des, w_tilt_des,K_lin,K_att,ref_traj(1:12,i),ref_traj(13:end,i), params, drone_params);
        %     Pcsi_dt = PI_CSI_DOT(Xk,csi0,PI0,PI_csi0,r,p0, cntrl_gains, n_des, alpha_des,f_ext(:,i+1));
        %     PI_csi_dt = cl_sens('PI_CSI_FUN',PI_evo(:,:,i-1),PI_csi_evo(:,:,i-1),x0, wr0, wr_dot_des, w_tilt_des,K_lin,K_att,ref_traj(1:12,i),ref_traj(13:end,i), params, drone_params);
        % 
        %     PI_next = PI0 + Pk1 * dt;
        %     PI_csi_next = PI_csi0 + Pcsi_dt * dt;
        % 
        %     K_pi = diag(PI0*delta_mat*PI0');
        % end

        %%% Equality constraints   
        % Input 
        constr = {constr{:}, wr_dot_des};
        lb_constr = [lb_constr; -1e5*ones(N_rotors,1)];
        ub_constr = [ub_constr;  1e5*zeros(N_rotors,1)];
    
        constr = {constr{:}, w_tilt_des};
        lb_constr = [lb_constr; -1e3*ones(N_rotors,1)];
        ub_constr = [ub_constr;  1e3*ones(N_rotors,1)];
    
        % State
        constr = {constr{:}, Xk};
        lb_constr = [lb_constr; lb_states];
        ub_constr = [ub_constr; ub_states];

        %%%%%%%%%
        
        % Cost
        J_int = J_int  + sum(tilt_des_next.^2);
    
        wr0 = wr0 + wr_dot_des * dt;
        tilt_des_old = tilt_des_next;
        % csi0 = csi_next;
    
        % if(use_tubes)
        %     PI0 = PI_next;
        %     PI_csi0 = PI_csi_next;
        % end
    
    end

    cost = J_int;


end