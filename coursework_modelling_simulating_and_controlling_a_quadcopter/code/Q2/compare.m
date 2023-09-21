close all;
clear all;

load("q1ori.mat");
load("q1pos.mat");

load("q2ori.mat");
load("q2pos.mat");


figure(1)
Trajectory_Plot1=plot3(pos1(1,:),pos1(2,:),pos1(3,:),'b');
hold on;
Trajectory_Plot2=plot3(pos(1,:),pos(2,:),pos(3,:),'r');
grid on;

Trajectory_Plot1.LineWidth = 1; 
start_point.LineWidth = 1; 
end_point.LineWidth = 1; 
title('Drone Position' ); 

xlabel('x');
ylabel('y');
zlabel('z');
xlim([-10 10]); 
ylim([-10 10]); 
zlim([0 15]); 
legend('Q1 Trajectory','Q2 Trajectory');



figure(2)
title('Orientation over time')
subplot(3,1,1);
time = 0: 0.02 : 8 ; 
PlotRoll1 = plot(time,ori1(1,:) ,'r');
hold on;
PlotRoll2 = plot(time,ori(1,:),'k--');
xlabel('roll');
xlim([0 8]); 
ylim([0 1.5]); 
legend('Q1','Q2') 


subplot(3,1,2);

PlotRoll3 = plot(time,ori1(2,:) ,'g');
hold on;
PlotRoll4 = plot(time,ori(2,:),'k--');
xlabel('pitch');
xlim([0 8]); 
ylim([0 1.5]); 
legend('Q1','Q2') 
 
subplot(3,1,3);
PlotRoll5 = plot(time,ori1(3,:) ,'b');
hold on;
PlotRoll6 = plot(time,ori(3,:),'k--');
xlabel('yaw');
xlim([0 8]); 
ylim([0 1.5]); 
legend('Q1','Q2') 
 

