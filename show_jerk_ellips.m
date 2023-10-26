%% Visualization of the Jerk ellipsoid
close all

jerk_matrix
clearvars -except A_jerk_fun N_rotors jerk_ellips

wr_num = 300^2 * ones(N_rotors,1);
tilt = deg2rad(0) * [1;-1;1;-1];
eul = deg2rad([0.0; 0.0; 0.0]);

figure()
% Kj = full( jerk_ellips(eul, wr_num, tilt) )
% plot_ellipse(Kj);
% hold on

tilt = deg2rad(5) * [1;-1;1;-1];
Kj = full( jerk_ellips(eul, wr_num, tilt) )
plot_ellipse(Kj,'edgecolor', 'b')
hold on

tilt = deg2rad(10) * [1;-1;1;-1];
wr_num = 450^2 * ones(N_rotors,1);
Kj = full( jerk_ellips(eul, wr_num, tilt) )
plot_ellipse(Kj,'edgecolor', 'r')
hold on

tilt = deg2rad(-15) * ones(N_rotors,1);
wr_num = 350^2 * ones(N_rotors,1);
Kj = full( jerk_ellips(eul, wr_num, tilt) )
plot_ellipse(Kj,'edgecolor', 'g')
hold on

eul = deg2rad([0.0; 10.0; 0.0]);
tilt = deg2rad(10) * [1;-1;1;-1];
wr_num = 450^2 * ones(N_rotors,1);
Kj = full( jerk_ellips(eul, wr_num, tilt) )
plot_ellipse(Kj,'edgecolor', 'black')
hold off
% axis equal