%% Symbolic model of a tilting drone to the jerk
function [X_DOT] = drone_model_sym(generate_cpp)
    
    if ~exist('generate_cpp','var')
        generate_cpp = false;
    end

    addpath('casadi_matlab')
    import casadi.*
    
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
    wB_acc = states(16:18);
    tilt = states(19:19+N_rotors-1);
    
    % Parameters
    params = SX.sym('param',N_params);
    mass = params(1);
    Kf = params(2);
    Km = params(3);
    I_diag = params(4:6);
    arm = params(7);
    rotor_angles = params(8:8+N_rotors-1);
    K_tilt = params(end);
    
    %% System equations
    wr = SX.sym('wr',N_rotors,1);
    wr_dot = SX.sym('wr_dot',N_rotors,1);
    w_tilt= SX.sym('w_tilt',N_rotors,1);
    tilt_des= SX.sym('tilt_des',N_rotors,1);
    gravity = 9.81;
    
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
    
    Tmat = [1, 0, sin(eul(2))*cos(eul(1));
            0, cos(eul(1)), -sin(eul(1));
            0, -sin(eul(1)), cos(eul(2))*cos(eul(1))];
    
    out = A_jerk * [wr_dot;w_tilt] + [b_vec; zeros(3,1)];
    eul_dot = Tmat\wB;
    
    % tilt_dot = K_tilt * w_tilt;
    tilt_dot = K_tilt * (tilt_des-tilt);
    
    pos_dot = vel;
    % vel_wB_dot = [0;0;-gravity;zeros(3,1)] + Jr*[A, zeros(6,N_rotors)]*[wr;w_tilt];
    vel_dot = acc;
    wB_dot = wB_acc;
    
    X_DOT = Function('X_DOT',{states,wr,wr_dot,w_tilt,params,tilt_des},{pos_dot,vel_dot,out(1:3),eul_dot,wB_dot,out(4:6),tilt_dot}, ...
                             {'x0', 'wr', 'wr_dot', 'w_tilt', 'params','tilt_des'}, {'pos_dot','vel_dot','acc_dot','eul_dot','wB_dot','wB_ddot','tilt_dot'});
    
    %% C++ code generation
    if(generate_cpp)
        gen_opts = struct('main', true, ...
                          'mex', true, ...
                          'verbose', true, ...
                          'cpp', true, ...
                          'with_header', true);
        
        code_gen = CodeGenerator('drone_model.cpp', gen_opts);
        code_gen.add(X_DOT);
        code_gen.generate();
        mex drone_model.cpp -DMATLAB_MEX_FILE -DCASASI_MEX_ALLOW_DENSE
        movefile drone_model.cpp cpp_files/
        movefile drone_model.h cpp_files/
        movefile drone_model.mexa64 cpp_files/
    end
end