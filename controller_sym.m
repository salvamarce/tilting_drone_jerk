%% Symbolic controller for tilting quadrotor to the jerk
clear

addpath('casadi_matlab')
addpath('cpp_files')
addpath('util')

drone_settings

% States
states = SX.sym('state',N_states);
pos = states(1:3);
vel = states(4:6);
acc = states(7:9);
eul = states(10:12);
wB = states(13:15);
wB_dot = states(16:18);
tilt = states(19:19+N_rotors-1);

% Parameters
mass = param(1);
Kf = param(2);
Km = param(3);
I_diag = param(4:6);
arm = param(7);
rotor_angles = param(8:8+N_rotors-1);

%% Nominal allocation
wr = SX.sym('wr',N_rotors,1);

R_bw = Rx(eul(1))*Ry(eul(2))*Rz(eul(3));
I_mat = diag(I_diag);

Jr = [(1/mass) * R_bw, zeros(3,3); zeros(3,3), inv(I_mat)];
A = compute_A_SX(rotor_angles,tilt, arm, Kf, Km);

sum_a_diff = jacobian(A(:,1),tilt)*wr(1);
for i_A = 2:N_rotors
    sum_a_diff = sum_a_diff + jacobian(A(:,i_A),tilt)*wr(i_A);
end

Ja = [A, sum_a_diff];

A_jerk = Jr*Ja;

R_bw_dot = R_bw * skewMat(wB);

b_vec = (1/mass)*R_bw_dot*A(1:3,:)*wr;

%% Control equations
lin_ref = SX.sym('pos_ref',12,1);
pos_ref = lin_ref(1:3);
vel_ref = lin_ref(4:6);
acc_ref = lin_ref(7:9);
jerk_ff = lin_ref(10:12);

att_ref = SX.sym('att_ref',12,1);
eul_ref   = att_ref(1:3);
w_ref     = att_ref(4:6);
w_dot_ref = att_ref(7:9);
w_ddot_ff = att_ref(10:12);

K_lin = SX.sym('K_lin',9,1);
Kp1 = K_lin(1:3);
Kp2 = K_lin(4:6);
Kp3 = K_lin(7:9);

K_att = SX.sym('K_att',9,1);
Kw1 = K_att(1:3);
Kw2 = K_att(4:6);
Kw3 = K_att(7:9);

R_bw = Rx(eul(1))*Ry(eul(2))*Rz(eul(3));
Rd = Rx(eul_ref(1))*Ry(eul_ref(2))*Rz(eul_ref(3));
eR = vmap(Rd,R_bw);
% eR = eul_ref - eul;

jerk_des = jerk_ff + diag(Kp3)*(acc_ref - acc) + diag(Kp2)*(vel_ref - vel) ...
          + diag(Kp1)*(pos_ref - pos);

w_ddot_des = w_ddot_ff + diag(Kw3)*(w_dot_ref - wB_dot) + diag(Kw2)*(w_ref - wB) ...
          + diag(Kw1)*eR;

z = zeros(2*N_rotors,1);

pinv_A_jerk = pinv(A_jerk);
ctrl_out = pinv_A_jerk*([jerk_des; w_ddot_des] - 0*[b_vec; zeros(3,1)]);% + (eye(8)- pinv_A_jerk*A_jerk)*z;

wr_dot = ctrl_out(1:N_rotors);
w_tilt = ctrl_out(N_rotors+1:end);

CTRL = Function('CTRL',{states,wr,K_lin,K_att,lin_ref,att_ref},{wr_dot, w_tilt,jerk_des,w_ddot_des,A_jerk}, ...
                         {'x0', 'wr', 'K_lin', 'K_att', 'lin_ref', 'att_ref'}, {'wr_dot', 'w_tilt','jd','wd','a_inv'});

%% C++ code generation
gen_opts = struct('main', true, ...
                  'mex', true, ...
                  'verbose', true, ...
                  'cpp', true, ...
                  'with_header', true);

code_gen = CodeGenerator('controller.cpp', gen_opts);
code_gen.add(CTRL);
code_gen.generate();
mex controller.cpp -DMATLAB_MEX_FILE -DCASASI_MEX_ALLOW_DENSE
movefile controller.cpp cpp_files/
movefile controller.h cpp_files/
movefile controller.mexa64 cpp_files/
