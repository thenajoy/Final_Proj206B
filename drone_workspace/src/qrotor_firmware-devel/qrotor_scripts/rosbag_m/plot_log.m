clear;
%%

folder = '/home/kotaru/workspace/catkin_ws/qrotor_firmware_ws/bags/';
file = '2021-04-18-17-37-05.bag';
filename = strcat(folder,file);
bag = rosbag(filename);

%%
log = DebagQrotor.readQrotormsgsLog(bag, '/falcon/log');

%%
close all;

figure
stairs(log.time, log.thrust, 'linewidth',1);
grid on; grid minor;
latex_title('$$f$$');

figure
for i = 1:3
    subplot(3,1,i); hold on;
    stairs(log.time, log.moment(:,i), 'linewidth', 1);
    grid on; grid minor;
end
