clear
close all 

addpath('casadi_matlab')
import casadi.*

addpath('cpp_files')
addpath('util')

parameters

%% Trajectory
t0 = 0;
Tf = 2.0; 
Tf_sim = Tf +2.0;
dt = 0.001;
N = Tf_sim/dt;
N_diff = (Tf_sim-Tf)/dt;

center = 0;
arc = 1.57;
rho = 1.0;

cp_s = [0; 0; linspace(0,rho*arc,N_cp-4)'; rho*arc; rho*arc];
t_traj = linspace(0,Tf_sim,N);

[s, s_dot, s_ddot] = time_law(cp_s, N_cp, dt, N-N_diff);

sp = sin(s./rho);
cp = cos(s./rho);
ref_traj = [rho*cp; 0*rho*sp; 1*rho*sp;
         -1*s_dot.*sp; 0*s_dot.*cp; 1*s_dot.*cp;
         1*(-(s_dot.^2) .* cp./rho- s_ddot.*sp); 0*(-(s_dot.^2) .* sp./rho + s_ddot .*cp); 1*(-(s_dot.^2) .* sp./rho + s_ddot .*cp);
         zeros(3,N-N_diff); %jerk ff
         zeros(12,N-N_diff)];
% 
angle_des = deg2rad(10);
cp_s = [0; 0; linspace(0,angle_des,N_cp-4)'; angle_des; angle_des];
[s, s_dot, s_ddot] = time_law(cp_s, N_cp, dt, N-N_diff);
ref_traj(13,:) = 0*s;         ref_traj(14,:) = -s;
ref_traj(16,:) = 0*s_dot;     ref_traj(17,:) = -s_dot;
ref_traj(19,:) = 0*s_ddot;    ref_traj(20,:) = -s_ddot;

ref_traj = [ref_traj, ref_traj(:,end).*ones(24,N_diff)];

% temp = ref_traj(1,:);
% ref_traj(1,:) = ref_traj(2,:);
% ref_traj(2,:) = temp;
% 
% temp = ref_traj(4,:);
% ref_traj(4,:) = ref_traj(5,:);
% ref_traj(5,:) = temp;
% 
% temp = ref_traj(7,:);
% ref_traj(7,:) = ref_traj(8,:);
% ref_traj(8,:) = temp;
% 
% temp = ref_traj(10,:);
% ref_traj(10,:) = ref_traj(11,:);
% ref_traj(11,:) = temp;

%% Nominal value
delta_Kf = 0.0;
delta_K_tilt = 0.0; %K_tilt
delta_p = [delta_Kf, delta_K_tilt];

[x_nom,~,PI_nom,~,~,~] = system_simulation(delta_p, ref_traj, N, dt);


%% Perturbed
clearvars -except ref_traj x_nom PI_nom N dt Tf
parameters

delta_Kf = -Kf*0.0;
delta_K_tilt = 0.01; %K_tilt
delta_p = [delta_Kf, delta_K_tilt];

[x_pert,~,~,~,~,~] = system_simulation(delta_p, ref_traj, N, dt);

diff_x = (x_pert - x_nom)./(delta_K_tilt);

ylabels = ["x", "y", "z", "v_x", "v_y", "v_z","ax","ay","az",...
           "roll", "pitch","yaw", "w_roll", "w_pitch", "w_yaw", "dw_roll", "dw_pitch", "dw_yaw",...
           "a1", "a2", "a3", "a4","a5","a6"];
  
for indx = 1:N_states
    pi_nom = reshape(PI_nom(indx,2,:), [], 1)';
    diff = diff_x(indx,:);
    
    ff = figure('Units','centimeters','OuterPosition',[5, 5, 21, 12]);
    subplot(2,1,1)
    plot(pi_nom, 'DisplayName','Pi')
    hold on
    plot(diff, 'DisplayName', 'diff')
    hold off
    title(ylabels(indx))
    legend show
    grid on
    subplot(2,1,2)
    plot((diff-pi_nom)', 'DisplayName','error')
    % plot(pi_nom, 'DisplayName','Pi')
    legend show
    grid on
end
