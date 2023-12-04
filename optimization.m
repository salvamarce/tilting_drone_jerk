clear
close all
clc

addpath('casadi_matlab')
import casadi.*

addpath('cpp_files')
addpath('util')

parameters
delta_Kf = 0.0;
delta_K_tilt = 0.0; %K_tilt
delta_p = [delta_Kf, delta_K_tilt];

drone_params = params;
drone_params(2) = drone_params(2) + delta_Kf;
drone_params(end) = drone_params(end) + delta_p(2);

%% Trajectory
t0 = 0;
Tf = 5.0; 
Tf_sim = Tf;
dt = 0.001;
N = Tf_sim/dt;
N_diff = (Tf_sim-Tf)/dt;

center = 0;
arc = 1.57;
rho = 2.0;

cp_s = [0; 0; linspace(0,rho*arc,N_cp-4)'; rho*arc; rho*arc];
t_traj = linspace(0,Tf_sim,N);

[s, s_dot, s_ddot] = time_law(cp_s, N_cp, dt, N-N_diff);

sp = sin(s./rho);
cp = cos(s./rho);
ref_traj = [rho*cp; 0*rho*sp; 0*rho*sp;
         -1*s_dot.*sp; 0*s_dot.*cp; 0*s_dot.*cp;
         1*(-(s_dot.^2) .* cp./rho- s_ddot.*sp); 0*(-(s_dot.^2) .* sp./rho + s_ddot .*cp); 0*(-(s_dot.^2) .* sp./rho + s_ddot .*cp);
         zeros(3,N-N_diff); %jerk ff
         zeros(12,N-N_diff)];
% 
% angle_des = deg2rad(15);
% cp_s = [0; 0; linspace(0,angle_des,N_cp-4)'; angle_des; angle_des];
% [s, s_dot, s_ddot] = time_law(cp_s, N_cp, dt, N-N_diff);
% ref_traj(13,:) = s;         ref_traj(14,:) = -s;
% ref_traj(16,:) = s_dot;     ref_traj(17,:) = -s_dot;
% ref_traj(19,:) = s_ddot;    ref_traj(20,:) = -s_ddot;

ref_traj = [ref_traj, ref_traj(:,end).*ones(24,N_diff)];

[state_nom, ctrl_nom, PI_nom, PI_csi_nom, wr_nom, tilt_nom, tilt_des_nom] = system_simulation(delta_p, ref_traj, N, dt, true);

optimization_model()
