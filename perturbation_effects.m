%% Compare nominal and perturbated cases
clearvars
% close all
clc

addpath('casadi_matlab')
import casadi.*

addpath('cpp_files')
addpath('util')

plot_state = true;
saturation = false;

parameters
delta_Kf = -params(2)*0.0;
delta_K_tilt = zeros(N_rotors,1);
delta_p = [delta_Kf; delta_K_tilt];

drone_params = params;
drone_params(2) = drone_params(2) + delta_Kf;
drone_params(end-N_rotors+1:end) = drone_params(end-N_rotors+1:end) + delta_K_tilt;

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
ref_traj = [1*rho*cp; 0*rho*sp; rho*sp;
         -1*s_dot.*sp; 0*s_dot.*cp; s_dot.*cp;
         1*(-(s_dot.^2) .* cp./rho- s_ddot.*sp); 0*(-(s_dot.^2) .* sp./rho + s_ddot .*cp); (-(s_dot.^2) .* sp./rho + s_ddot .*cp);
         zeros(3,N-N_diff); %jerk ff
         zeros(12,N-N_diff)];

% angle_des = deg2rad(20);
% cp_s = [0; 0; linspace(0,angle_des,N_cp-4)'; angle_des; angle_des];
% [s, s_dot, s_ddot] = time_law(cp_s, N_cp, dt, N-N_diff);
% ref_traj(13,:) = 0*s;         ref_traj(14,:) = -s;
% ref_traj(16,:) = 0*s_dot;     ref_traj(17,:) = -s_dot;
% ref_traj(19,:) = 0*s_ddot;    ref_traj(20,:) = -s_ddot;

ref_traj = [ref_traj, ref_traj(:,end).*ones(24,N_diff)];
f_x_ext = -6.0*(1/mass); %meglio considerare 1/m ? ma è giusto?
f_ext = [linspace(0,f_x_ext,N/2), f_x_ext*ones(1,N/2); zeros(1,N); zeros(1,N)];


% temp = ref_traj(1,:);
% ref_traj(1,:) = ref_traj(2,:);
% ref_traj(2,:) = temp;
% 
% temp = ref_traj(4,:);
% ref_traj(4,:) = ref_traj(5,:);
% ref_traj(5,:) = temp;
% 
% temp = ref_traj(7,:);
% ref_traj(7,:) = ref_traj(8,:);
% ref_traj(8,:) = temp;
% 
% temp = ref_traj(10,:);
% ref_traj(10,:) = ref_traj(11,:);
% ref_traj(11,:) = temp;

% Nominal case
% [state_evo_nom, ctrl_evo_nom, PI_evo, PI_csi_evo, wr_evo_nom, tilt_evo_nom, tilt_des_evo_nom] = system_simulation(delta_p, ref_traj, N, dt, zeros(2*N_rotors,1), saturation);
[state_evo_nom, ctrl_evo_nom, wr_evo_nom, tilt_evo_nom, tilt_des_evo_nom] = new_system_simulation(delta_p, ref_traj, N, dt, zeros(2*N_rotors,1), true, f_ext);

% Perturbated case
delta_Kf = -params(2)*0.10;
delta_K_tilt = 0.0*[1; 0.85; 0.8; 1.0];% 0.95; 0.8];
delta_p = [delta_Kf; delta_K_tilt];

% [state_evo_pert, ctrl_evo_pert, PI_evo, PI_csi_evo, wr_evo_pert, tilt_evo_pert, tilt_des_evo_pert] = system_simulation(delta_p, ref_traj, N, dt, zeros(2*N_rotors,1), saturation);
[state_evo_pert, ctrl_evo_pert, wr_evo_pert, tilt_evo_pert, tilt_des_evo_pert] = new_system_simulation(delta_p, ref_traj, N, dt, zeros(2*N_rotors,1), true, f_ext);


if(plot_state)
    t= 0:dt:Tf_sim-dt;

    figure() %'WindowState','maximized'
    %vel
    subplot(4,2,1)
    plot(t,state_evo_pert(4,:),'LineStyle','--', 'DisplayName','v_{x_{pert}}', 'LineWidth',1.5, 'Color', 'red')
    hold on
    plot(t,state_evo_pert(5,:),'LineStyle','--', 'DisplayName','v_{y_{pert}}', 'LineWidth',1.5, 'Color', 'green')
    hold on
    plot(t,state_evo_pert(6,:),'LineStyle','--', 'DisplayName','v_{z_{pert}}', 'LineWidth',1.5, 'Color', 'blue')
    hold on
    set(gca,'ColorOrderIndex',1)
    plot(t,state_evo_nom(4,:),'DisplayName','v_{x_{nom}}', 'Color', 'red')
    hold on
    plot(t,state_evo_nom(5,:),'DisplayName','v_{y_{nom}}', 'Color', 'green')
    hold on
    plot(t,state_evo_nom(6,:),'DisplayName','v_{z_{nom}}', 'Color', 'blue')
    hold off
    legend show
    grid on
    xlabel('[s]')
    ylabel('velocity [m/s]')
    %axis([0 Tf_sim min(min(state_evo_pert(4:6,:)))-0.5 max(max(state_evo_pert(4:6,:)))+0.5])
    %acc
    subplot(4,2,3)
    plot(t,state_evo_pert(7,:), 'LineStyle','--', 'DisplayName','a_{x_{pert}}', 'LineWidth',1.5, 'Color', 'red')
    hold on
    plot(t,state_evo_pert(8,:), 'LineStyle','--', 'DisplayName','a_{y_{pert}}', 'LineWidth',1.5, 'Color', 'green')
    hold on
    plot(t,state_evo_pert(9,:), 'LineStyle','--', 'DisplayName','a_{z_{pert}}', 'LineWidth',1.5, 'Color', 'blue')
    hold on
    set(gca,'ColorOrderIndex',1)
    plot(t,state_evo_nom(7,:), 'DisplayName','a_{x_{nom}}', 'Color', 'red')
    hold on
    plot(t,state_evo_nom(8,:), 'DisplayName','a_{y_{nom}}', 'Color', 'green')
    hold on
    plot(t,state_evo_nom(9,:), 'DisplayName','a_{z_{nom}}', 'Color', 'blue')
    hold off
    legend show
    grid on
    xlabel('[s]')
    ylabel('linear acceleration [m/s^2]')
    %axis([0 Tf_sim min(min(state_evo_pert(7:9,:)))-0.5 max(max(state_evo_pert(7:9,:)))+0.5])
    %Err
    subplot(4,2,5)
    plot(t,state_evo_nom(7,:)-state_evo_pert(7,:), 'DisplayName','e_{a_x}', 'LineWidth',1.5, 'Color', 'red')
    hold on
    plot(t,state_evo_nom(8,:)-state_evo_pert(8,:), 'DisplayName','e_{a_y}', 'LineWidth',1.5, 'Color', 'green')
    hold on
    plot(t,state_evo_nom(9,:)-state_evo_pert(9,:), 'DisplayName','e_{a_z}', 'LineWidth',1.5, 'Color', 'blue')
    hold off
    legend show
    grid on
    xlabel('[s]')
    ylabel('error [m/s^2]')
    %Att
    subplot(4,2,2)
    plot(t,rad2deg(state_evo_pert(10,:)), 'LineStyle','--', 'DisplayName','\phi_{pert}', 'LineWidth',1.5, 'Color', 'red')
    hold on
    plot(t,rad2deg(state_evo_pert(11,:)), 'LineStyle','--', 'DisplayName','\theta_{pert}', 'LineWidth',1.5, 'Color', 'green')
    hold on
    plot(t,rad2deg(state_evo_pert(12,:)), 'LineStyle','--', 'DisplayName','\psi_{pert}', 'LineWidth',1.5, 'Color', 'blue')
    hold on
    set(gca,'ColorOrderIndex',1)
    plot(t,rad2deg(state_evo_nom(10,:)), 'DisplayName','\phi_{nom}', 'Color', 'red')
    hold on
    plot(t,rad2deg(state_evo_nom(11,:)), 'DisplayName','\theta_{nom}', 'Color', 'green')
    hold on
    plot(t,rad2deg(state_evo_nom(12,:)), 'DisplayName','\psi_{nom}', 'Color', 'blue')
    hold off
    legend show
    grid on
    xlabel('[s]')
    ylabel('Attitude [deg]')
    %axis([0 Tf_sim rad2deg(min(min(state_evo_pert(10:12,:))))-0.5 rad2deg(max(max(state_evo_pert(10:12,:))))+0.5])
    %Angular vel
    subplot(4,2,4)
    plot(t,state_evo_pert(13,:), 'LineStyle','--', 'DisplayName','w_{\phi_{pert}}', 'LineWidth',1.5, 'Color', 'red')
    hold on
    plot(t,state_evo_pert(14,:), 'LineStyle','--', 'DisplayName','w_{\theta_{pert}}', 'LineWidth',1.5, 'Color', 'green')
    hold on
    plot(t,state_evo_pert(15,:), 'LineStyle','--', 'DisplayName','w_{\psi_{pert}}', 'LineWidth',1.5, 'Color', 'blue')
    hold on
    set(gca,'ColorOrderIndex',1)
    plot(t,state_evo_nom(13,:), 'DisplayName','w_{\phi_{nom}}', 'Color', 'red')
    hold on
    plot(t,state_evo_nom(14,:), 'DisplayName','w_{\theta_{nom}}', 'Color', 'green')
    hold on
    plot(t,state_evo_nom(15,:), 'DisplayName','w_{\psi_{nom}}', 'Color', 'blue')
    hold off
    legend show
    grid on
    xlabel('[s]')
    ylabel('Angular vel [rad/s]')
    %axis([0 Tf_sim min(min(state_evo_pert(13:15,:)))-0.5 max(max(state_evo_pert(13:15,:)))+0.5])
    %Err att
    subplot(4,2,6)
    plot(t,rad2deg(state_evo_nom(10,:)-state_evo_pert(10,:)), 'DisplayName','e_\phi', 'LineWidth',1.5, 'Color', 'red')
    hold on
    plot(t,rad2deg(state_evo_nom(11,:)-state_evo_pert(11,:)), 'DisplayName','e_\theta', 'LineWidth',1.5, 'Color', 'green')
    hold on
    plot(t,rad2deg(state_evo_nom(12,:)-state_evo_pert(12,:)), 'DisplayName','e_\psi', 'LineWidth',1.5, 'Color', 'blue')
    hold off
    legend show
    grid on
    xlabel('[s]')
    ylabel('error [deg]')
    %POS
    subplot(4,2,[7,8])
    plot(t,state_evo_nom(1,:)-state_evo_pert(1,:), 'DisplayName','e_{x}', 'LineWidth',1.5, 'Color', 'red')
    hold on
    plot(t,state_evo_nom(2,:)-state_evo_pert(2,:), 'DisplayName','e_{y}', 'LineWidth',1.5, 'Color', 'green')
    hold on
    plot(t,state_evo_nom(3,:)-state_evo_pert(3,:), 'DisplayName','e_{z}', 'LineWidth',1.5, 'Color', 'blue')
    hold off
    hold off
    legend show
    grid on
    xlabel('[s]')
    ylabel('Position [m]')
    %axis([0 Tf_sim min(min(state_evo_pert(1:3,:)))-0.5 max(max(state_evo_pert(1:3,:)))+0.5])

    figure()
    for i=1:N_rotors
        plot(linspace(0,Tf_sim,N),rad2deg(tilt_evo_nom(i,:)), 'DisplayName',['\alpha_{', num2str(i), '_{nom}}'], 'LineWidth', 1.5)
        hold on
    end
    set(gca,'ColorOrderIndex',1)
    for i=1:N_rotors
        plot(linspace(0,Tf_sim,N),rad2deg(tilt_evo_pert(i,:)),'LineStyle','--', 'DisplayName',['\alpha_{', num2str(i), '_{pert}}'], 'LineWidth', 1.5)
        hold on
    end
    hold on
    yline(alpha_minmax, '--', 'LineWidth',1.5,'HandleVisibility','off')
    hold on
    yline(-alpha_minmax, '--', 'LineWidth',1.5,'HandleVisibility','off')
    hold off
    ylabel('\alpha [deg]')
    %axis([0 Tf_sim -alpha_minmax alpha_minmax])
    legend show
    grid on

    figure()
    for i=1:N_rotors
        plot(linspace(0,Tf_sim,N),rad2deg(tilt_evo_nom(i,:))-rad2deg(tilt_evo_pert(i,:)), 'DisplayName',['e_{\alpha_{', num2str(i), '}}'], 'LineWidth', 1.5)
        hold on
    end
    hold off
    ylabel('\alpha_{err} [deg]')
    %axis([0 Tf_sim -alpha_minmax alpha_minmax])
    legend show
    grid on
    
    figure()
    for i=1:N_rotors
        plot(linspace(0,Tf_sim,N),wr_evo_nom(i,:), 'DisplayName',['w_{r_', num2str(i),'_{nom}}'], 'LineWidth', 1.5)
        hold on
    end
    set(gca,'ColorOrderIndex',1)
    for i=1:N_rotors
        plot(linspace(0,Tf_sim,N),wr_evo_pert(i,:),'LineStyle','--', 'DisplayName',['w_{r_', num2str(i),'_{pert}}'], 'LineWidth', 1.5)
        hold on
    end
    hold on
    yline(minPropSpeedsq, '--', 'LineWidth',1.5,'HandleVisibility','off')
    hold on
    yline(maxPropSpeedsq, '--', 'LineWidth',1.5,'HandleVisibility','off')
    hold off
    ylabel('n [rad/s]')
    %axis([0 Tf_sim minPropSpeedsq maxPropSpeedsq])
    legend show
    grid on

    figure()
    for i=1:N_rotors
        plot(linspace(0,Tf_sim,N),ctrl_evo_nom(i,:), 'DisplayName',['dot w_{r_', num2str(i),'_{nom}}'], 'LineWidth', 1.5)
        hold on
    end
    set(gca,'ColorOrderIndex',1)
    for i=1:N_rotors
        plot(linspace(0,Tf_sim,N),ctrl_evo_pert(i,:),'LineStyle','--', 'DisplayName',['dot w_{r_', num2str(i),'_{pert}}'], 'LineWidth', 1.5)
        hold on
    end
    hold off
    ylabel('w dot r [rad/s^2]')
    %axis([0 Tf_sim min(min(ctrl_evo_pert(1:N_rotors,:)))-5 max(max(ctrl_evo_pert(1:N_rotors,:)))+5])
    legend show
    grid on

    figure()
    for i=N_rotors+1:2*N_rotors
        plot(linspace(0,Tf_sim,N),ctrl_evo_nom(i,:), 'DisplayName',['dot \alpha_{', num2str(i), '_{nom}}'], 'LineWidth', 1.5)
        hold on
    end
    set(gca,'ColorOrderIndex',1)
    for i=N_rotors+1:2*N_rotors
        plot(linspace(0,Tf_sim,N),ctrl_evo_pert(i,:),'LineStyle','--', 'DisplayName',['dot \alpha_{', num2str(i), '_{pert}}'], 'LineWidth', 1.5)
        hold on
    end
    hold off
    ylabel('dot \alpha [rad/s]')
    % axis([0 Tf_sim min(min(ctrl_evo(N_rotors+1:2*N_rotors,:)))-5 max(max(ctrl_evo(N_rotors+1:2*N_rotors,:)))+5])
    legend show
    grid on

end