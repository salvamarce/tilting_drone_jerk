%% Symbolic model of a tilting drone to the jerk
function [X_DOT] = new_drone_model_sym(generate_cpp)
    
    if ~exist('generate_cpp','var')
        generate_cpp = false;
    end

    addpath('casadi_matlab')
    import casadi.*
    
    addpath('cpp_files')
    addpath('util')
    
    drone_settings
    
    % States
    states = SX.sym('state',12+N_rotors);
    pos = states(1:3);
    vel = states(4:6);
    eul = states(7:9);
    wB = states(10:12);
    tilt = states(13:13+N_rotors-1);
    
    % Parameters
    params = SX.sym('param',N_params);
    mass = params(1);
    Kf = params(2);
    Km = params(3);
    I_diag = params(4:6);
    arm = params(7);
    rotor_angles = params(8:8+N_rotors-1);
    K_tilt = params(end-N_rotors+1:end);
    
    %% System equations
    wr = SX.sym('wr',N_rotors,1);
    w_tilt= SX.sym('w_tilt',N_rotors,1);
    dt = SX.sym('dt');
    f_ext = SX.sym('f_ext', 3);
    gravity = 9.81;

    R_bw = Rx(eul(1))*Ry(eul(2))*Rz(eul(3));
    I_mat = diag(I_diag);
    
    
    A = compute_A_SX(rotor_angles, tilt, arm, Kf, Km);
    tilt_dot = (1-K_tilt) .* w_tilt;
    % tilt_new = tilt + tilt_dot * dt;
        
    R_bw_dot = R_bw * skewMat(wB);
    
    
    Tmat = [1, 0, sin(eul(2))*cos(eul(1));
            0, cos(eul(1)), -sin(eul(1));
            0, -sin(eul(1)), cos(eul(2))*cos(eul(1))];
    
    eul_dot = Tmat\wB;
    
    pos_dot = vel;
    vel_dot = [0;0;-gravity] + (1/mass) * R_bw * A(1:3,:) * wr + f_ext;

    wB_dot = I_mat\A(4:6,:)*wr;
    
    
    X_DOT = Function('X_DOT',{states,wr,w_tilt,params, dt, f_ext},{pos_dot,vel_dot,eul_dot,wB_dot,tilt_dot}, ...
                             {'x0', 'wr', 'w_tilt', 'params', 'dt','f_ext'}, {'pos_dot','vel_dot','eul_dot','wB_dot','tilt_dot'});
    
    %% C++ code generation
    if(generate_cpp)
        gen_opts = struct('main', true, ...
                          'mex', true, ...
                          'verbose', true, ...
                          'cpp', true, ...
                          'with_header', true);
        
        code_gen = CodeGenerator('new_drone_model.cpp', gen_opts);
        code_gen.add(X_DOT);
        code_gen.generate();
        mex new_drone_model.cpp -DMATLAB_MEX_FILE -DCASASI_MEX_ALLOW_DENSE
        movefile new_drone_model.cpp cpp_files/
        movefile new_drone_model.h cpp_files/
        movefile new_drone_model.mexa64 cpp_files/
    end
end