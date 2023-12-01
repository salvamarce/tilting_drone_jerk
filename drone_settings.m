%% Declaration of system's parameters

% Hexacopter
% N_rotors = 6;
% angle_span = deg2rad(60);
% first_angle = deg2rad(30);

% Quadcopter
N_rotors = 6;

N_states = 18+N_rotors; % px,py,pz,
                        % vx,vx,vz,
                        % ax,ay,az,
                        % phi,theta,psi,
                        % w_phi,w_theta,w_psi,
                        % w_d_phi,w_d_theta,w_d_psi,
                        % alpha

N_cp = 6;

% Parameters
N_params = 7+N_rotors+1; %mass,Kf,Km,Ixx,Iyy,Izz,arm,rotor_angles,K_tilt

