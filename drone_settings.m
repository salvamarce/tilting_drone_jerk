%% Declaration of system's parameters

N_rotors = 4;

N_states = 18+N_rotors; % px,py,pz,
                        % vx,vx,vz,
                        % ax,ay,az,
                        % phi,theta,psi,
                        % w_phi,w_theta,w_psi,
                        % w_d_phi,w_d_theta,w_d_psi,
                        % alpha

N_cp = 6;

% Parameters
N_params = 7+N_rotors+N_rotors; %mass,Kf,Km,Ixx,Iyy,Izz,arm,rotor_angles,K_tilt

