drone_settings

% Control gains
K_lin_pos = 1*[1.0; 1.0; 1.0];
K_lin_vel = 0*[1.0; 1.0; 1.0];
K_lin_acc = 0*[1.0; 1.0; 1.0];
K_lin = [K_lin_pos; K_lin_vel; K_lin_acc];

K_att_pos = 0*[1.0; 1.0; 1.0];
K_att_vel = 0*[1.0; 1.0; 1.0];
K_att_acc = 0*[1.0; 1.0; 1.0];
K_att = [K_att_pos; K_att_vel; K_att_acc];

% %% Contraints
% % maxPropSpeedsq = 200^2;
% % minPropSpeedsq = 16^2;
% maxPropSpeedsq = 865^2; %rad/s
% minPropSpeedsq = 200^2; %rad/s
% 
% alpha_minmax = 45; %Degree
% alpha_dot_minmax = 60 * 1.745329 * 1e-2; %rad/s
% 
% %% Trajectories
% 
% quat0 = [1; 0; 0; 0];


% cntrl_gains = [55;55;45; %Kp_p
%                15;15;10;  %Kd_p
%                0*20; 0*20; 0*20; %Ki_p
%                25;25;25; %Kq
%                2.0;2.0;2.5]; %Kr
% 
% % % RK4
% % cntrl_gains = [45;45;30; %Kp_p
% %                3.5;1.5;1.5;  %Kd_p
% %                0.5;0.5;0.5; %Ki_p
% %                25;25;45; %Kq
% %                1.0;1.0;1.5]; %Kr
% 
% perc_mean = 0.10;
% delta = [(p0(2)*perc_mean/2.5)^2; 
%          ones(N_rotors,1)*(deg2rad( (perc_mean*100)/2 )^2);
%          ones(3,1)*0.00001^2];
% 
% delta_mat  = diag(delta);