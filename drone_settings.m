%% Declaration of system's parameters

N_rotors = 4;
angle_span = deg2rad(90);
first_angle = deg2rad(45);

N_states = 18+N_rotors; % px,py,pz,
                        % vx,vx,vz,
                        % ax,ay,az,
                        % phi,theta,psi,
                        % w_phi,w_theta,w_psi,
                        % w_d_phi,w_d_theta,w_d_psi,
                        % alpha

N_cp = 12;


% Parameters
N_params = 7+N_rotors; %mass,Kf,Km,Ixx,Iyy,Izz,arm,rotor_angles

mass = 3.0;
Ix = 2.10e-2;
Iy = 2.10e-2;
Iz = 3.6235401e-2;
I_diag = [Ix; Iy; Iz];
Kf = 1.498e-5;
Km = 0.936e-5;
arm = 0.4;
rotor_angles = first_angle:angle_span:deg2rad(360)-first_angle;

param = [mass; Kf; Km; I_diag; arm; rotor_angles'];
