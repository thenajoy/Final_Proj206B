clear

folder = '/home/kotaru/workspace/catkin_ws/qrotor_firmware_ws/bags/experiments/';
file = '3qPLANK_2021-06-23-18-33-22.mat';
filename = strcat(folder,file);
% bag = rosbag(filename);

load(filename)

%%
names = {'white', 'red', 'blue'};
[drone, plank] = getBagDataRefined(data);


%%
close all

figure;
subplot(3,1,1); hold on
% plot(drone(1).mocap.t, drone(1).mocap.pos(1,:), 'linewidth',1);
for i = 1:3
    scatter(drone(i).mocap.t, drone(i).mocap.pos(1,:), 4, 'filled')    
end
scatter(plank.mocap.t, plank.mocap.pos(1,:), 4, 'filled')   
grid on; grid minor;
latex_title(strcat(names{1},'-$$p_x$$'));

subplot(3,1,2); hold on
% plot(drone(1).mocap.t, drone(1).mocap.pos(2,:), 'linewidth',1);
for i = 1:3
    scatter(drone(i).mocap.t, drone(i).mocap.pos(2,:), 4, 'filled')    
end
scatter(plank.mocap.t, plank.mocap.pos(2,:), 4, 'filled')  
grid on; grid minor;
latex_title(strcat(names{1},'-$$p_y$$'));

subplot(3,1,3); hold on
% plot(drone(1).mocap.t, drone(1).mocap.pos(2,:), 'linewidth',1);
for i = 1:3
    scatter(drone(i).mocap.t, drone(i).mocap.pos(3,:), 4, 'filled')    
end
scatter(plank.mocap.t, plank.mocap.pos(3,:), 4, 'filled')  
grid on; grid minor;
latex_title(strcat(names{1},'-$$p_z$$'));

latex_legend({names{:}, 'plank'});


figure('name', 'loop_rate');
for i = 1:3
    subplot(3,1,i);
    plot(drone(i).log.t, drone(i).log.loop_rate, 'linewidth',1);
%     scatter(drone(i).log.t, drone(i).log.loop_rate, 4, 'filled')    
end

figure('name', 'accel');
for i = 1:3
   subplot(3, 1, i); hold on
   for j = 1:3
      plot(drone(j).imu.t, drone(j).imu.accel(i,:), 'linewidth', 1); 
   end
end

figure('name', 'position estimate');
for i = 1:3
    subplot(3,1,i); hold on
    for j = 1:3
        scatter(drone(j).est.t, drone(j).est.pos(i,:), 4, 'filled');
    end
end





















