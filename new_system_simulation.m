%% Simulation initial conditions
function [state_evo, ctrl_evo, wr_evo, tilt_evo, tilt_des_evo] = new_system_simulation(delta_p, ref_traj, N, dt, z_null, rotors_saturation, f_ext)

    if ~exist('rotors_saturation','var')
        rotors_saturation = false;
    end

    drone_settings
    parameters

    t=0;
    
    if(N_rotors == 6)
        signs = [-1;0;1;-1;0;1];
    else 
        if(N_rotors == 4)
            signs = [-1;1;-1;1]; 
        end
    end

    wr_evo = (mass*9.81/(N_rotors*Kf)).*ones(N_rotors,N);
    tilt_evo = deg2rad(0)*signs.*ones(N_rotors,N);
    tilt_des_evo = tilt_evo;
    x0 = [ref_traj(1:6,1);zeros(6,1);tilt_evo(:,1)];
    fake_x0 = [x0(1:6); zeros(3,1); x0(7:12); zeros(3,1); x0(13:end)];
    state_evo = fake_x0.*ones(N_states, N);
    ctrl_evo = zeros(2*N_rotors, N);

    drone_params = params;
    drone_params(2) = drone_params(2) + delta_p(1);
    drone_params(end-N_rotors+1:end) = drone_params(end-N_rotors+1:end) + delta_p(2:end);

    k1 = 50;
    % k2 = 2;
    k_null = -1;
    % old_tilt = zeros(N_rotors,1);
    

    %% Problem formulation
    for i=2:N
        
        wr0 = wr_evo(:,i-1);

        %Angolo
        for i_rot = 1:N_rotors
            z_null(N_rotors+i_rot) =  2 * (tilt_des_evo(i_rot,i-1) - signs(i_rot)*deg2rad(0)) / (2*deg2rad(alpha_minmax))^2;
            for j_rot = 1:N_rotors
                if(j_rot ~= i_rot)
                    z_null(N_rotors+i_rot) = z_null(N_rotors+i_rot) +  ( (tilt_des_evo(j_rot,i-1) - signs(j_rot)*deg2rad(0))/ (2*deg2rad(alpha_minmax)))^2;
                end
            end
        end

        z_null = k_null * k1 * (1/(2*N_rotors)) * z_null;
        
        [wr_dot_des, w_tilt_des] = controller('CTRL',fake_x0,wr0,K_lin,K_att,ref_traj(1:12,i),ref_traj(13:end,i), params, z_null);
        csi_dot = cl_sens('CSI_FUN', wr_dot_des);
        csi_next = wr0 + csi_dot * dt;
        tilt_des_next = tilt_des_evo(:,i-1) + w_tilt_des * dt;    

        if(rotors_saturation)
            for rotor_i = 1:N_rotors
                       
                csi_next(rotor_i) =  max(minPropSpeedsq, min(csi_next(rotor_i), maxPropSpeedsq));
                tilt_des_next(rotor_i) = max(-deg2rad(alpha_minmax), min(tilt_des_next(rotor_i), deg2rad(alpha_minmax)));

            end
        end
        
        [pos_dot,vel_dot,eul_dot,wB_dot,w_tilt] = new_drone_model('X_DOT',x0,csi_next,w_tilt_des,drone_params,dt, f_ext(:,i));
        
        tilt_next = tilt_evo(:,i-1) + w_tilt * dt;
        if(rotors_saturation)
            for rotor_i = 1:N_rotors
                tilt_next(rotor_i) = max(-deg2rad(alpha_minmax), min(tilt_next(rotor_i), deg2rad(alpha_minmax)));
            end
        end

        vel_next = x0(4:6) + vel_dot * dt; 
        pos_next = x0(1:3) + vel_next * dt; 
       
        w_next = x0(10:12) + wB_dot * dt;
        eul_next = x0(7:9) + eul_dot * dt;
    
        wr_evo(:,i) = csi_next; %wr0 + wr_dot_des * dt;
        tilt_des_evo(:,i) = tilt_des_next;
        tilt_evo(:,i) = tilt_next;
        state_evo(:,i) = [pos_next; vel_next; vel_dot; eul_next; w_next; wB_dot; tilt_next];
        ctrl_evo(:,i) = [wr_dot_des; w_tilt_des];
        
        t = t + dt;
        x0 = [pos_next; vel_next; eul_next; w_next; tilt_next];
        fake_x0 = [x0(1:6); vel_dot; x0(7:12); wB_dot; x0(13:end)];

    end

end
