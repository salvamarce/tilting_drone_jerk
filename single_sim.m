%% Simulation initial conditions
t=0;

wr_evo = (mass*9.81/(N_rotors*Kf)).*ones(N_rotors,N);
tilt_evo = deg2rad(0)*[-1;1;-1;1].*ones(N_rotors,N);
x0 = [ref_traj(1:9,1);deg2rad(0);deg2rad(0);ref_traj(15:21,1);tilt_evo(:,1)];
state_evo = x0.*ones(N_states, N);
ctrl_evo = zeros(2*N_rotors, N);
K_force_evo = zeros(3,3,N);

%% Problem formulation
for i=2:N
    % ref_traj(1:12,i) = [ref_traj(1:9,1); zeros(3,1)];
    % ref_traj(13:end,i) = [deg2rad(0);deg2rad(0);zeros(10,1)];
    [wr_dot_des, w_tilt_des, jd,wd,A_inv] = controller('CTRL',x0,wr_evo(:,i-1),K_lin,K_att,ref_traj(1:12,i),ref_traj(13:end,i));
    % wr_dot_des
    % w_tilt_des = w_tilt_des *0.0174533;
    jd
    wd
    A_inv
    % A_inv*[jd;wd]

    % for rotor_i = 1:N_rotors
    % 
    %     if(n_des(rotor_i) < minPropSpeedsq)
    %         n_des(rotor_i) = minPropSpeedsq;
    %     else
    %         if( n_des(rotor_i) > maxPropSpeedsq)
    %             n_des(rotor_i) = maxPropSpeedsq;
    %         end
    %     end
    % 
    % 
    %     if(abs((alpha_des(rotor_i) - alpha_evo(rotor_i,i-1))/dt) > alpha_dot_minmax )
    %         alpha_des(rotor_i) = alpha_evo(rotor_i,i-1) + sign((alpha_des(rotor_i) - alpha_evo(rotor_i,i-1))) *alpha_dot_minmax * dt;
    %     end
    % 
    %     if(alpha_des(rotor_i) < -deg2rad(alpha_minmax))
    %         alpha_des(rotor_i) = -deg2rad(alpha_minmax);
    %     else
    %         if( alpha_des(rotor_i) > deg2rad(alpha_minmax))
    %             alpha_des(rotor_i) = deg2rad(alpha_minmax);
    %         end
    %     end
    % 
    % end
     
    [pos_dot,vel_dot,acc_dot,eul_dot,wB_dot,wB_ddot,Jr,Ja] = drone_model('X_DOT',x0,wr_evo(:,i-1),wr_dot_des,w_tilt_des);
    % rank( full( jerk_allocation('A_jerk_fun',x0(10:12),wr_evo(:,i-1),tilt_evo(:,i)) ) )
    
    % % Jr
    % Ja(1,1)
    % Ja(2,1)
    % Ja(1,2)
    % Ja(2,2)
    % Ja(:,1:N_rotors)
    % Ja(:,N_rotors+1:end)
    % acc_dot
    % vel_dot
    % pos_dot
    % w_tilt_des
    acc_next = x0(7:9) + acc_dot * dt;
    vel_next = x0(4:6) + vel_dot * dt;
    pos_next = x0(1:3) + pos_dot * dt;
    
    % eul_dot
    % wB_dot
    % wB_ddot
    w_dot_next = x0(16:18) + wB_ddot * dt;
    w_next = x0(13:15) + wB_dot * dt;
    eul_next = x0(10:12) + eul_dot * dt;

    wr_evo(:,i) = wr_evo(:,i-1) + wr_dot_des * dt;
    tilt_evo(:,i) = tilt_evo(:,i-1) + w_tilt_des * dt;
    state_evo(:,i) = [pos_next; vel_next; acc_next; eul_next; w_next; w_dot_next; tilt_evo(:,i)];
    ctrl_evo(:,i) = [wr_dot_des; w_tilt_des];
    t = t + dt;
    x0 = state_evo(:,i);

end
