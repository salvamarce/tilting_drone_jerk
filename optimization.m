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

%% Trajectory
t0 = 0;
Tf = 0.5; 
Tf_sim = Tf;
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

[state_nom, ctrl_nom, PI_nom, PI_csi_nom, wr_nom, tilt_nom, tilt_des_nom] = system_simulation(delta_p, ref_traj, N, dt, zeros(2*N_rotors,1));

guess.state_evo = state_nom;
guess.wr_dot_evo = ctrl_nom(1:6,:);
guess.w_tilt_evo = ctrl_nom(7:end,:);

lb_states = [-2.0*ones(3,1); -5.0*ones(3,1); -10.0*ones(3,1); -1.57*ones(3,1); -2.0*ones(3,1); -4.0*ones(3,1); -deg2rad(alpha_minmax)*ones(N_rotors,1)];
ub_states = [ 2.0*ones(3,1);  5.0*ones(3,1);  10.0*ones(3,1);  1.57*ones(3,1);  2.0*ones(3,1);  4.0*ones(3,1);  deg2rad(alpha_minmax)*ones(N_rotors,1)];

[var0, var, ub_var, lb_var, constr, ub_constr, lb_constr, cost] = optimization_model(ref_traj, dt, lb_states, ub_states, guess);
% [var0, var, ub_var, lb_var, constr, ub_constr, lb_constr, cost] = single_shooting(ref_traj, dt, lb_states, ub_states);

prob = struct('f', cost,...
              'x', vertcat(var{:}),...
              'g', vertcat(constr{:}));


% opts.ipopt.print_level = false;
opts.verbose = true;
% opts.print_time = false;
% opts.expand = true;
opts.ipopt.hessian_approximation = 'limited-memory';
% opts.ipopt.max_iter = 7000;
% opts.ipopt.tol = 1e-3;
% opts.ipopt.acceptable_tol = 1e-2;
opts.ipopt.linear_solver = 'ma97';
% opts.jit = true;
% opts.compiler = 'shell';
% opts.jit_options.flags = {'-O3 -march=native'};
% opts.jit_options.verbose = true;

disp("Mando nlpsol");
solver = nlpsol('solver', 'ipopt', prob, opts);

prsol = tic;

% Solve the NLP
disp("Risolvo")
sol = solver('x0', var0, 'lbx', lb_var, 'ubx', ub_var,...
            'lbg', lb_constr, 'ubg', ub_constr);

prsol_t = toc(prsol);
