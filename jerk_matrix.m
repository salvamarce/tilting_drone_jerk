%% Symbolic function to compute the jerk allocation matrix
% Nominal allocation
clear

addpath('casadi_matlab')
import casadi.*

addpath('cpp_files')
addpath('util')

drone_settings

params = SX.sym('param',N_params);
mass = params(1);
Kf = params(2);
Km = params(3);
I_diag = params(4:6);
arm = params(7);
rotor_angles = params(8:8+N_rotors-1);
K_tilt = params(end);


eul = SX.sym('eul',3,1);
wr = SX.sym('wr',N_rotors,1);
tilt = SX.sym('tilt',N_rotors,1);

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

% W = diag([1e-3*ones(1,N_rotors),1e8*ones(1,N_rotors)]);
% inv_W = inv(W);
% pinv_A_jerk = inv_W* A_jerk'*inv(A_jerk*inv_W*A_jerk');
% ellips = pinv_A_jerk(:,1:3)'*pinv_A_jerk(:,1:3);
ellips = inv(A_jerk(1:3,:)*A_jerk(1:3,:)');

A_jerk_fun = Function('A_jerk_fun',{eul,wr,tilt,params},{A_jerk});
jerk_ellips = Function('jerk_ellips',{eul,wr,tilt,params},{ellips});

%% C++ code generation

gen_opts = struct('main', true, ...
                  'mex', true, ...
                  'verbose', true, ...
                  'cpp', true, ...
                  'with_header', true);

code_gen = CodeGenerator('jerk_allocation.cpp', gen_opts);
code_gen.add(A_jerk_fun);
code_gen.add(jerk_ellips);
code_gen.generate();
mex jerk_allocation.cpp -DMATLAB_MEX_FILE -DCASASI_MEX_ALLOW_DENSE
movefile jerk_allocation.cpp cpp_files/
movefile jerk_allocation.h cpp_files/
movefile jerk_allocation.mexa64 cpp_files/
