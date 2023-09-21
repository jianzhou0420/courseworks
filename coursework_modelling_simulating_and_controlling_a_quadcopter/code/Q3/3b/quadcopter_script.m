%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Visualisation code for quadcopter 
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;

%Define total width, length and height of flight arena (metres)
spaceDim = 15;
spaceLimits = [-spaceDim/2 spaceDim/2 -spaceDim/2 spaceDim/2 0 spaceDim/2];

%do you want to draw a ground image on the figure?
draw_ground = false;
if(draw_ground)
    ground_img = imread('ground.png');
end


%figure to display drone simulation
f1 = figure;
ax1 = gca;
view(ax1, 3);
axis equal;
axis(spaceLimits)
grid ON
grid MINOR
caxis(ax1, [0 spaceDim]);
hold(ax1,'on')
axis vis3d

running_time=250;

num_drones = 1;

%instantiate a drone object, input the axis and arena limits
drones = [];
for i = 1:num_drones
    drones = [drones Drone(ax1, spaceDim, num_drones)];
end

while(drones(1).time < running_time)
    % clear axis
    cla(ax1);
    
    
    %% main: update and draw drones
    for i = 1:num_drones
        % Jian 15/12/2022: I changed update(drones(1)) to drones.update()
        % for better code readability
        drones.update(); 
    end
    
    %% optionally 
    if(draw_ground) % draw the ground image
        imagesc([-spaceDim,spaceDim],[-spaceDim,spaceDim],ground_img);
    end
   
    camlight %apply fancy lighting
    
    %update figure
    % Jian 15/12/2022: I commented the line "drawnow",
    % because, in drones.update(),there has already been a 'drones.draw' 
    drawnow
end

pos1 = drones.posRecord;
ori1 = drones.orienRecord;

% Trajectory display
figure(2);


Trajectory_Plot=plot3(pos1(1,:),pos1(2,:),pos1(3,:),'b');
hold on 
start_point=plot3(pos1(1,1),pos1(2,1),pos1(3,1),'ro'); 
hold on
end_point = plot3(pos1(1,end),pos1(2,end),pos1(3,end),'gx');
grid on;

Trajectory_Plot.LineWidth = 1; 
start_point.LineWidth = 1; 
end_point.LineWidth = 1; 
title('Drone Position' ); 

xlabel('x');
ylabel('y');
zlabel('z');
xlim([-10 10]); 
ylim([-10 10]); 
zlim([0 15]); 

legend('Drone Trajectory','Start Point','End Point');

% Roll, Pitch, Yaw display

figure(3) ;
time = 0: drones.time_interval : running_time-0.02; 
PlotRoll = plot(time,ori1(1,:) ,'r');
hold on
PlotPitch = plot(time,ori1(2,:) ,'g');
hold on
PlotYaw = plot(time,ori1(3,:) ,'b');
hold on
legend('Roll','Pitch','Yaw');
title('Orientation');
xlabel('Time (second)');
ylabel('Angle (radian)');
