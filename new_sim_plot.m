clearvars
% close all
clc

addpath('casadi_matlab')
import casadi.*

addpath('cpp_files')
addpath('util')

plot_state = true;

parameters
delta_Kf = -params(2)*0.0;
delta_K_tilt = 0.0;
delta_p = [delta_Kf, delta_K_tilt];

drone_params = params;
drone_params(2) = drone_params(2) + delta_Kf;
drone_params(end) = drone_params(end) + delta_K_tilt;

%% Trajectory
t0 = 0;
Tf = 6.0; 
Tf_sim = Tf +5.0;
dt = 0.001;
N = Tf_sim/dt;
N_diff = (Tf_sim-Tf)/dt;

center = 0;
arc = 1.57;
rho = 4.0;

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
% angle_des = deg2rad(10);
% cp_s = [0; 0; linspace(0,angle_des,N_cp-4)'; angle_des; angle_des];
% [s, s_dot, s_ddot] = time_law(cp_s, N_cp, dt, N-N_diff);
% ref_traj(13,:) = 0*s;         ref_traj(14,:) = -s;
% ref_traj(16,:) = 0*s_dot;     ref_traj(17,:) = -s_dot;
% ref_traj(19,:) = 0*s_ddot;    ref_traj(20,:) = -s_ddot;

ref_traj = [ref_traj, ref_traj(:,end).*ones(24,N_diff)];

f_x_ext = -2.0;
f_ext = [linspace(0,f_x_ext,N/2), f_x_ext*ones(1,N/2); zeros(1,N); zeros(1,N)];

[state_evo, ctrl_evo, wr_evo, tilt_evo, tilt_des_evo] = new_system_simulation(delta_p, ref_traj, N, dt, zeros(2*N_rotors,1), true, f_ext);

if(plot_state)
    t= 0:dt:Tf_sim-dt;

    figure() %'WindowState','maximized'
    %vel
    subplot(3,2,1)
    plot(t,state_evo(1,:), 'DisplayName','x', 'LineWidth',1.5, 'Color', 'red')
    hold on
    plot(t,state_evo(2,:), 'DisplayName','y', 'LineWidth',1.5, 'Color', 'green')
    hold on
    plot(t,state_evo(3,:), 'DisplayName','z', 'LineWidth',1.5, 'Color', 'blue')
    hold on
    plot(t,ref_traj(1,:),'LineStyle','--','DisplayName','x_{ref}', 'Color', 'red')
    hold on
    plot(t,ref_traj(2,:),'LineStyle','--','DisplayName','y_{ref}', 'Color', 'green')
    hold on
    plot(t,ref_traj(3,:),'LineStyle','--','DisplayName','z_{ref}', 'Color', 'blue')
    hold off
    legend show
    grid on
    xlabel('[s]')
    ylabel('position [m]')
    subplot(3,2,3)
    plot(t,state_evo(4,:), 'DisplayName','v_x', 'LineWidth',1.5, 'Color', 'red')
    hold on
    plot(t,state_evo(5,:), 'DisplayName','v_y', 'LineWidth',1.5, 'Color', 'green')
    hold on
    plot(t,state_evo(6,:), 'DisplayName','v_z', 'LineWidth',1.5, 'Color', 'blue')
    hold on
    plot(t,ref_traj(4,:),'LineStyle','--','DisplayName','v_{x_{ref}}', 'Color', 'red')
    hold on
    plot(t,ref_traj(5,:),'LineStyle','--','DisplayName','v_{y_{ref}}', 'Color', 'green')
    hold on
    plot(t,ref_traj(6,:),'LineStyle','--','DisplayName','v_{z_{ref}}', 'Color', 'blue')
    hold off
    legend show
    grid on
    xlabel('[s]')
    ylabel('velocity [m/s]')
    axis([0 Tf_sim min(min(ref_traj(4:6,:)))-0.5 max(max(ref_traj(4:6,:)))+0.5])
    %Err
    subplot(3,2,5)
    plot(t,ref_traj(1,:)-state_evo(1,:), 'DisplayName','x', 'LineWidth',1.5, 'Color', 'red')
    hold on
    plot(t,ref_traj(2,:)-state_evo(2,:), 'DisplayName','y', 'LineWidth',1.5, 'Color', 'green')
    hold on
    plot(t,ref_traj(3,:)-state_evo(3,:), 'DisplayName','z', 'LineWidth',1.5, 'Color', 'blue')
    hold off
    legend show
    grid on
    xlabel('[s]')
    ylabel('error [m]')
    %Att
    subplot(3,2,2)
    plot(t,rad2deg(state_evo(10,:)), 'DisplayName','\phi', 'LineWidth',1.5, 'Color', 'red')
    hold on
    plot(t,rad2deg(state_evo(11,:)), 'DisplayName','\theta', 'LineWidth',1.5, 'Color', 'green')
    hold on
    plot(t,rad2deg(state_evo(12,:)), 'DisplayName','\psi', 'LineWidth',1.5, 'Color', 'blue')
    hold on
    plot(t,rad2deg(ref_traj(13,:)),'LineStyle','--','DisplayName','\phi_{ref}', 'Color', 'red')
    hold on
    plot(t,rad2deg(ref_traj(14,:)),'LineStyle','--','DisplayName','\theta_{ref}', 'Color', 'green')
    hold on
    plot(t,rad2deg(ref_traj(15,:)),'LineStyle','--','DisplayName','\psi_{ref}', 'Color', 'blue')
    hold off
    legend show
    grid on
    xlabel('[s]')
    ylabel('Attitude [deg]')
    axis([0 Tf_sim rad2deg(min(min(ref_traj(13:15,:))))-0.5 rad2deg(max(max(ref_traj(13:15,:))))+0.5])
    %Angular vel
    subplot(3,2,4)
    plot(t,state_evo(13,:), 'DisplayName','w_\phi', 'LineWidth',1.5, 'Color', 'red')
    hold on
    plot(t,state_evo(14,:), 'DisplayName','w_\theta', 'LineWidth',1.5, 'Color', 'green')
    hold on
    plot(t,state_evo(15,:), 'DisplayName','w_\psi', 'LineWidth',1.5, 'Color', 'blue')
    hold on
    plot(t,ref_traj(16,:),'LineStyle','--','DisplayName','w_\phi_{ref}', 'Color', 'red')
    hold on
    plot(t,ref_traj(17,:),'LineStyle','--','DisplayName','w_\theta_{ref}', 'Color', 'green')
    hold on
    plot(t,ref_traj(18,:),'LineStyle','--','DisplayName','w_\psi_{ref}', 'Color', 'blue')
    hold off
    legend show
    grid on
    xlabel('[s]')
    ylabel('Angular vel [rad/s]')
    axis([0 Tf_sim min(min(ref_traj(16:18,:)))-0.5 max(max(ref_traj(16:18,:)))+0.5])
    %Err att
    subplot(3,2,6)
    plot(t,rad2deg(ref_traj(13,:)-state_evo(10,:)), 'DisplayName','\phi', 'LineWidth',1.5, 'Color', 'red')
    hold on
    plot(t,rad2deg(ref_traj(14,:)-state_evo(11,:)), 'DisplayName','\theta', 'LineWidth',1.5, 'Color', 'green')
    hold on
    plot(t,rad2deg(ref_traj(15,:)-state_evo(12,:)), 'DisplayName','\psi', 'LineWidth',1.5, 'Color', 'blue')
    hold off
    legend show
    grid on
    xlabel('[s]')
    ylabel('error [deg]')
   
    figure()
    for i=1:N_rotors
        plot(linspace(0,Tf_sim,N),rad2deg(tilt_evo(i,:)), 'DisplayName',['\alpha_{', num2str(i), '}'], 'LineWidth', 1.5)
        hold on
    end
    hold off
    ylabel('\alpha [deg]')
    axis([0 Tf_sim -alpha_minmax alpha_minmax])
    legend show
    grid on

    figure()
    for i=1:N_rotors
        plot(linspace(0,Tf_sim,N),rad2deg(tilt_des_evo(i,:))-rad2deg(tilt_evo(i,:)), 'DisplayName',['\alpha_{', num2str(i), '}'], 'LineWidth', 1.5)
        hold on
    end
    hold off
    ylabel('\alpha_{err} [deg]')
    axis([0 Tf_sim -alpha_minmax alpha_minmax])
    legend show
    grid on
    
    figure()
    for i=1:N_rotors
        plot(linspace(0,Tf_sim,N),wr_evo(i,:), 'DisplayName',['w_{r_', num2str(i),'}'], 'LineWidth', 1.5)
        hold on
    end
    % hold on
    % yline(minPropSpeedsq, '--', 'LineWidth',1.5,'HandleVisibility','off')
    % hold on
    % yline(maxPropSpeedsq, '--', 'LineWidth',1.5,'HandleVisibility','off')
    hold off
    ylabel('n [rad/s]')
    axis([0 Tf_sim minPropSpeedsq maxPropSpeedsq])
    legend show
    grid on

    figure()
    for i=1:N_rotors
        plot(linspace(0,Tf_sim,N),ctrl_evo(i,:), 'DisplayName',['dot w_{r_', num2str(i),'}'], 'LineWidth', 1.5)
        hold on
    end
    hold off
    ylabel('w dot r [rad/s^2]')
    axis([0 Tf_sim min(min(ctrl_evo(1:N_rotors,:)))-5 max(max(ctrl_evo(1:N_rotors,:)))+5])
    legend show
    grid on

    figure()
    for i=N_rotors+1:2*N_rotors
        plot(linspace(0,Tf_sim,N),ctrl_evo(i,:), 'DisplayName',['dot \alpha_{', num2str(i), '}'], 'LineWidth', 1.5)
        hold on
    end
    hold off
    ylabel('dot \alpha [rad/s]')
    % axis([0 Tf_sim min(min(ctrl_evo(N_rotors+1:2*N_rotors,:)))-5 max(max(ctrl_evo(N_rotors+1:2*N_rotors,:)))+5])
    axis([0 Tf_sim -4 4])
    legend show
    grid on

    figure()
    plot(t,state_evo(7,:), 'DisplayName','a_x', 'LineWidth',1.5, 'Color', 'red')
    hold on
    plot(t,state_evo(8,:), 'DisplayName','a_y', 'LineWidth',1.5, 'Color', 'green')
    hold on
    plot(t,state_evo(9,:), 'DisplayName','a_z', 'LineWidth',1.5, 'Color', 'blue')
    hold on
    plot(t,ref_traj(7,:),'LineStyle','--','DisplayName','a_{x_{ref}}', 'Color', 'red')
    hold on
    plot(t,ref_traj(8,:),'LineStyle','--','DisplayName','a_{y_{ref}}', 'Color', 'green')
    hold on
    plot(t,ref_traj(9,:),'LineStyle','--','DisplayName','a_{z_{ref}}', 'Color', 'blue')
    hold off

end
