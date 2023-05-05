clear

% folder = '/home/kotaru/Workspace/catkin_ws/qrotor_ws/bags/';
% file = 'test2.bag';
folder = '/home/kotaru/workspace/catkin_ws/qrotor_firmware_ws/';
file = 'test.bag';
% file = 'eppid.bag';

filename = strcat(folder,file);
bag = rosbag(filename);

%% debag
log = DebagQrotor.readQrotormsgsLog(bag, '/red_falcon/log');

pose_est = DebagQrotor.readNavmsgsOdometry(bag, '/falcon/odom/estimate');
pose_est.time = pose_est.time+pose_est.t0-log.t0;

pose_truth = DebagQrotor.readNavmsgsOdometry(bag, '/falcon/odom/ground_truth');
pose_truth.time = pose_truth.time+pose_truth.t0-log.t0;

% pose_sp = DebagQrotor.readNavmsgsOdometry(bag, 'falcon/odom/setpoint');
% pose_sp.time = pose_sp.time+pose_sp.t0-log.t0;

% imu = DebagQrotor.readSensormsgsImu(bag, '/falcon/imu');
% imu.time = imu.time+imu.t0-log.t0;


%%
close all;

% DebagQrotor.plotPowerReadings(log.time, log.voltage, log.current);

legends = { 'est', 'truth', 'onboard'};
euler_legend = {'roll', 'pitch', 'yaw'};
axes_char = ['x', 'y', 'z'];
ls = {':r', ':b', '-k'};
lw = [2,2,1];
xlim_ = [20, 26];
ylim_ = [-2,2];

nsp = 3;
figure('name', 'Euler angles');
hold on;
for i = 1:nsp
    subplot(nsp,1,i); hold on;
    plot(pose_est.time, pose_est.Eul(:,i), 'k', 'linewidth', 2);
    plot(pose_truth.time, pose_truth.Eul(:,i), 'b', 'linewidth', 1.5);
    plot(log.time, log.Eul(:,i), 'r', 'linewidth', 1);
        
%     xlim(xlim_);
%     ylim(ylim_);
%     if i==3
%         ylim(ylim_+60);    
%     end
    grid on; grid minor;
    latex_title(strcat('$$', euler_legend{i},'$$'), 15);
    latex_ylabel('[deg]');
    latex_legend(legends);
end

figure('name', 'moment');
hold on;
for i = 1:3
    subplot(3,1,i); hold on;
    plot(log.time, log.moment(:,i), 'r', 'linewidth', 1);

    grid on; grid minor;
    latex_title(strcat('$$M_', axes_char(i),'$$'), 15);
    latex_ylabel('[Nm]');
end

figure('name', 'Lyapunov')
hold on
plot(log.time, log.Lyap, 'linewidth', 2);
grid on; grid minor;

%%




%% the end

