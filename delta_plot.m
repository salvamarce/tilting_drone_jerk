clear 
close all

figure()
t_traj = linspace(0,5,5/0.001);
load("nom.mat")
subplot(3,1,1)
plot(t_traj,xlength,'r','LineWidth',1.5)
hold on
subplot(3,1,2)
plot(t_traj,ylength,'r','LineWidth',1.5)
hold on
subplot(3,1,3)
plot(t_traj,zlength,'r','LineWidth',1.5)
hold on

load("delta98.mat")
subplot(3,1,1)
plot(t_traj,xlength,'g','LineWidth',1.5)
hold on
subplot(3,1,2)
plot(t_traj,ylength,'g','LineWidth',1.5)
hold on
subplot(3,1,3)
plot(t_traj,zlength,'g','LineWidth',1.5)
hold on

load("delta95.mat")
subplot(3,1,1)
plot(t_traj,xlength,'b','LineWidth',1.5)
ylabel('x')
hold off
subplot(3,1,2)
plot(t_traj,ylength,'b','LineWidth',1.5)
ylabel('y')
hold off
subplot(3,1,3)
plot(t_traj,zlength,'b','LineWidth',1.5)
ylabel('z')
xlabel('time [s]')
hold off