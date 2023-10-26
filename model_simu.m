clearvars
close all
clc

addpath('casadi_matlab')
addpath('cpp_files')
addpath('util')
% addpath('../')
import casadi.*

plot_state = true;
plot_animation = false;
record = false;
video_name = 'video/opti_pert';

params

%% Trajectory
t0 = 0;
Tf = 10.0;
dt = 0.001;
N = Tf/dt;

center = 0;
arc = 1.57;
rho = 1.0; %radius

cp_s = [0; 0; linspace(0,rho*arc,N_cp-4)'; rho*arc; rho*arc];
t_traj = linspace(0,Tf,N);
s = []; s_dot = []; s_ddot=[];

for i_s = 1:N
    t = t_traj(i_s)/Tf;
    [s_coeff, ds_coeff, dds_coeff] = Bezier_curve(t,N_cp);
    s = [s, s_coeff*cp_s];
    if(i_s>1)
        s_dot = [s_dot, ( s(i_s)-s(i_s-1) )/dt];
    else
        s_dot = [s_dot, 0.0];
    end
    
    if(i_s>2)
        s_ddot = [s_ddot, ( s_dot(i_s)-s_dot(i_s-1) )/dt];
    else
        s_ddot = [s_ddot, 0.0];
    end
end

sp = sin(s./rho);
cp = cos(s./rho);
ref_traj = [rho*cp; rho*sp; rho*sp;
         -1*s_dot.*sp; 1*s_dot.*cp; 1*s_dot.*cp;
         1*(-(s_dot.^2) .* cp./rho- s_ddot.*sp); 1*(-(s_dot.^2) .* sp./rho + s_ddot .*cp); 1*(-(s_dot.^2) .* sp./rho + s_ddot .*cp);;
         zeros(3,N); %jerk ff
         zeros(12,N)];


single_sim



% if (plot_animation)
%     scale= 5e-5;
%     x_scaled = state_evo(1:3,:).*scale;
% 
%     figure()
%     subplot(3,2,[1,3,5])
%     plot3(x_scaled(1,:),x_scaled(2,:),x_scaled(3,:))
%     hold on
%     % H1 = plot_ellipse(reshape(K_force_evo(:,:,1),3,[]), x_scaled(1:3,1), 'edgecolor', 'b');
%     axis equal
%     subplot(3,2,2)
%     ax_plot = animatedline;
%     ax_plot.Color = 'r';
%     % ax_plot.Marker = '*';
%     hold on
%     ay_plot = animatedline;
%     ay_plot.Color = 'g';
%     % ay_plot.Marker = '*';
%     hold on
%     az_plot = animatedline;
%     az_plot.Color = 'b';
%     % az_plot.Marker = '*';
%     hold off
%     grid on
%     title("axis length")
% 
%     colors = ['r', 'g', 'b', 'c', "#7E2F8E", "#EDB120", 'k'];
% 
%     subplot(3,2,4)
%     for j=1:N_rotors
%         alpha_plot(j) = animatedline;
%         alpha_plot(j).Color = colors(j);
%         % alpha_plot(j).Marker = '*';
%     end
%     grid on
%     title("alpha angle")
% 
%     subplot(3,2,6)
%     for j=1:N_rotors
%         rotor_plot(j) = animatedline;
%         rotor_plot(j).Color = colors(j);
%         % rotor_plot(j).Marker = '*';
%     end
%     grid on
%     title("rotor vel")
% 
%     axd = linspace(0.5,1.0,N);
%     J_int = 0;
%     %% Ellipsoid
%     if(record)
%         obj = VideoWriter(video_name, 'Uncompressed AVI');
%         obj.FrameRate = 1/(dt);
%         open(obj);
%     end
% 
%     for i=1:N
% 
%         % plot_ellipse(reshape(K_force_evo(:,:,i),3,[]), x_scaled(1:3,i), 'alter', H1);
%         % J_int = J_int - ax_sq_evo(i) - ay_sq_evo(i);
%         ax_i = maxPropSpeedsq*sqrt(ax_sq_evo(i));
%         ay_i = maxPropSpeedsq*sqrt(ay_sq_evo(i));
%         if(~isreal(ax_i))
%             ax_i = 0.0;
%         end
%         if(~isreal(ay_i))
%             ay_i = 0.0;
%         end
% 
%         % az_i = axd(i);
%         % az_i =  1e4*az_sq_evo(i);
%         % ax_i = det(reshape(K_force_evo(:,:,i),3,[]));
%         addpoints(ax_plot, i,ax_i);
%         addpoints(ay_plot, i,ay_i);
%         % addpoints(az_plot, i,az_i);
%         for j=1:N_rotors
%             addpoints(alpha_plot(j), i, rad2deg(alpha_evo(j,i) + p0(9+(j-1)) ));
%             addpoints(rotor_plot(j), i, sqrt(n_evo(j,i)));
%         end
%         drawnow
% 
%         if(record)
%             pause(dt)
%             frame = getframe(gcf);
%             writeVideo(obj, frame);
%         end
% 
%     end
% end
% 
% if(record)
%     close(obj);
% end

%%
if(plot_state)
    t= 0:dt:Tf-dt;

    figure() %'WindowState','maximized'
    %Pos
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
    axis([0 Tf min(min(ref_traj(1:3,:)))-0.5 max(max(ref_traj(1:3,:)))+0.5])
    %Vel
    subplot(3,2,3)
    plot(t,state_evo(4,:), 'DisplayName','v_x', 'LineWidth',1.5, 'Color', 'red')
    hold on
    plot(t,state_evo(5,:), 'DisplayName','v_y', 'LineWidth',1.5, 'Color', 'green')
    hold on
    plot(t,state_evo(6,:), 'DisplayName','v_z', 'LineWidth',1.5, 'Color', 'blue')
    hold on
    plot(t,ref_traj(4,:),'LineStyle','--','DisplayName','x_{ref}', 'Color', 'red')
    hold on
    plot(t,ref_traj(5,:),'LineStyle','--','DisplayName','y_{ref}', 'Color', 'green')
    hold on
    plot(t,ref_traj(6,:),'LineStyle','--','DisplayName','z_{ref}', 'Color', 'blue')
    hold off
    legend show
    grid on
    xlabel('[s]')
    ylabel('linear velocity [m/s]')
    axis([0 Tf min(min(ref_traj(4:6,:)))-0.5 max(max(ref_traj(4:6,:)))+0.5])
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
    axis([0 Tf rad2deg(min(min(ref_traj(13:15,:))))-0.5 rad2deg(max(max(ref_traj(13:15,:))))+0.5])
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
    axis([0 Tf min(min(ref_traj(16:18,:)))-0.5 max(max(ref_traj(16:18,:)))+0.5])
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
        plot(linspace(0,Tf,N),rad2deg(tilt_evo(i,:)), 'DisplayName',['\alpha_{', num2str(i), '}'], 'LineWidth', 1.5)
        hold on
    end
    hold off
    ylabel('\alpha [deg]')
    axis([0 Tf -60 60])
    legend show
    grid on
    
    figure()
    for i=1:N_rotors
        plot(linspace(0,Tf,N),wr_evo(i,:), 'DisplayName',['w_{r_', num2str(i),'}'], 'LineWidth', 1.5)
        hold on
    end
    % hold on
    % yline(minPropSpeedsq, '--', 'LineWidth',1.5,'HandleVisibility','off')
    % hold on
    % yline(maxPropSpeedsq, '--', 'LineWidth',1.5,'HandleVisibility','off')
    hold off
    ylabel('n [rad/s]')
    axis([0 Tf 200^2 800^2])
    legend show
    grid on

    figure()
    for i=1:N_rotors
        plot(linspace(0,Tf,N),ctrl_evo(i,:), 'DisplayName',['dot w_{r_', num2str(i),'}'], 'LineWidth', 1.5)
        hold on
    end
    hold off
    ylabel('w dot r [rad/s^2]')
    axis([0 Tf min(min(ctrl_evo(1:N_rotors,:)))-5 max(max(ctrl_evo(1:N_rotors,:)))+5])
    legend show
    grid on

    figure()
    for i=N_rotors+1:2*N_rotors
        plot(linspace(0,Tf,N),ctrl_evo(i,:), 'DisplayName',['dot \alpha_{', num2str(i), '}'], 'LineWidth', 1.5)
        hold on
    end
    hold off
    ylabel('dot \alpha [rad/s]')
    % axis([0 Tf min(min(ctrl_evo(N_rotors+1:2*N_rotors,:)))-5 max(max(ctrl_evo(N_rotors+1:2*N_rotors,:)))+5])
    axis([0 Tf -10 10])
    legend show
    grid on

    % figure()
    % % subplot(2,1,1)
    % for fmi=1:3
    %     plot(t(1:end-1),F_M_evo(fmi,2:end), 'DisplayName',['F_', num2str(fmi)], 'LineWidth', 1.5)
    %     hold on
    % end
    % hold on
    % yline(3.5, '--', 'LineWidth', 1.5,'HandleVisibility','off')
    % hold on
    % yline(2.5, '--', 'LineWidth', 1.5,'HandleVisibility','off')
    % grid on
    % legend show
    % subplot(2,1,2)
    % for fmi=4:6
    %     plot(t,F_M_evo(fmi,:), 'DisplayName',['M_', num2str(fmi-3)], 'LineWidth', 1.5)
    %     hold ont= 0:dt:Tf_sim-dt;
end
