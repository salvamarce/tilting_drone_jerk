%% Closed loop sensitivity
function [PI_fun, PI_csi_fun, csi_fun] = closed_loop_sens(generate_cpp)
    
    if ~exist('generate_cpp','var')
        generate_cpp = false;
    end

    addpath('casadi_matlab')
    import casadi.*
    
    addpath('cpp_files')
    addpath('util')
    
    %% Define models
    X_DOT = drone_model_sym();
    CTRL = controller_sym();
    
    %% Sensitivity
    drone_settings
    states = SX.sym('state',N_states);
    params = SX.sym('param',N_params);
    drone_params = SX.sym('drone_param',N_params);
    
    csi = SX.sym('csi',N_rotors,1);
    dt = SX.sym('dt');
    wr_dot_des = SX.sym('wr_dot_des',N_rotors,1);
    w_tilt_des = SX.sym('w_tilt_des',N_rotors,1);
    wr_des = SX.sym('wr_des',N_rotors,1);
    tilt_des = SX.sym('tilt_des',N_rotors,1);
    lin_ref = SX.sym('pos_ref',12,1);
    att_ref = SX.sym('att_ref',12,1);
    K_lin = SX.sym('K_lin',9,1);
    K_att = SX.sym('K_att',9,1);
    z_null = SX.sym('z_null',2*N_rotors,1);
    
    [pos_dot,vel_dot,jerk,eul_dot,wB_dot,wB_ddot,w_tilt] = X_DOT(states,csi,wr_dot_des,w_tilt_des,drone_params,tilt_des,dt);
    [wr_dot_ctrl, w_tilt_ctrl] = CTRL(states,csi,K_lin,K_att,lin_ref,att_ref,params,z_null);  
    
    system_out = [pos_dot; vel_dot; jerk; eul_dot; wB_dot; wB_ddot; w_tilt];
    system_in = [wr_dot_des; w_tilt_des];
    control_out = [wr_dot_ctrl; w_tilt_ctrl];
    pi_params = [drone_params(2), drone_params(end)];
    N_pi_par = length(pi_params);
    
    %csi is wr_ctrl
    csi_dot = wr_dot_des;
    
    %Gradients
    dfdq = jacobian(system_out,states);
    dfdu = jacobian(system_out,system_in);
    dfdp = jacobian(system_out,pi_params);
    %my u is the h in the paper
    dudq = jacobian(control_out,states);
    dudcsi = jacobian(control_out,csi);
    % g is the function of internal controller's states
    dgdq = jacobian(csi_dot,states);
    dgdcsi = jacobian(csi_dot,csi);
    
    
    PI = SX.sym('PI',N_states,N_pi_par);
    PI_csi = SX.sym('PI_csi',length(csi),N_pi_par);
    
    dfdq_n = dfdq+dfdu*dudq;
    PI_dot = dfdq_n*PI + dfdp + dfdu*dudcsi*PI_csi ;
    PI_csi_dot = dgdq*PI + dgdcsi*PI_csi;
    
    % PI_u = dudq*PI ;%+ dudcsi*PI_csi;
    
    PI_fun = Function('PI_FUN', {PI,PI_csi,states, csi, wr_dot_des, w_tilt_des, K_lin, K_att, lin_ref, att_ref, params, drone_params,tilt_des,z_null}, {PI_dot});
    PI_csi_fun = Function('PI_CSI_FUN', {PI,PI_csi,states, csi, wr_dot_des, w_tilt_des, K_lin, K_att, lin_ref, att_ref, params, drone_params,tilt_des,z_null}, {PI_csi_dot});
    csi_fun = Function('CSI_FUN', {wr_dot_des}, {csi_dot});
    
    %% C++ code generation
    if(generate_cpp)
        gen_opts = struct('main', true, ...
                          'mex', true, ...
                          'verbose', true, ...
                          'cpp', true, ...
                          'with_header', true);
        
        code_gen = CodeGenerator('cl_sens.cpp', gen_opts);
        code_gen.add(PI_fun);
        code_gen.add(PI_csi_fun);
        code_gen.add(csi_fun);
        code_gen.generate();
        mex cl_sens.cpp -DMATLAB_MEX_FILE -DCASASI_MEX_ALLOW_DENSE
        movefile cl_sens.cpp cpp_files/
        movefile cl_sens.h cpp_files/
        movefile cl_sens.mexa64 cpp_files/
    end
end