drone_settings

angle_span = deg2rad(60);
first_angle = deg2rad(30);

mass = 3.0;
Ix = 2.10e-2;
Iy = 2.10e-2;
Iz = 3.6235401e-2;
I_diag = [Ix; Iy; Iz];
Kf = 1.498e-5;
Km = 0.936e-5;
arm = 0.4;
rotor_angles = first_angle:angle_span:deg2rad(360)-first_angle;
K_tilt = 1.0;

params = [mass; Kf; Km; I_diag; arm; rotor_angles';K_tilt];

% Control gains
K_lin_pos = 1800*[1.5; 1.5; 1.0];
K_lin_vel = 750*[1.0; 1.0; 1.0];
K_lin_acc = 250*[1.0; 1.0; 1.0];
K_lin = [K_lin_pos; K_lin_vel; K_lin_acc];

K_att_pos = 2500*[1.0; 1.0; 1.0];
K_att_vel = 1500*[1.0; 1.0; 1.0];
K_att_acc = 50*[1.0; 1.0; 1.0];
K_att = [K_att_pos; K_att_vel; K_att_acc];

%% Contraints
maxPropSpeedsq = 650^2; %rad/s
minPropSpeedsq = 200^2; %rad/s

alpha_minmax = 45; %Degree
% alpha_dot_minmax = 60 * 1.745329 * 1e-2; %rad/s
