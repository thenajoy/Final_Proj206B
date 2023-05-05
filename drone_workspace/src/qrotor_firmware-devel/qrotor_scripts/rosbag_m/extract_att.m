

clear
folder = '/home/kotaru/workspace/catkin_wss/qrotor_ws/bags/experiments/';
file = 'rs_imu_att_hover.bag';

filename = strcat(folder,file);
bag = rosbag(filename);

bagselect = select(bag, 'Topic',  '/falcon/odometry/t265');
pose_struct = readMessages(bagselect,'DataFormat','struct');

quat = [cellfun(@(m) double(m.Pose.Pose.Orientation.W), pose_struct), ...
        cellfun(@(m) double(m.Pose.Pose.Orientation.X), pose_struct), ...
        cellfun(@(m) double(m.Pose.Pose.Orientation.Y), pose_struct),...
        cellfun(@(m) double(m.Pose.Pose.Orientation.Z), pose_struct)];

t = cellfun(@(m) double(m.Header.Stamp.Sec), pose_struct) ...
    + cellfun(@(m) double(m.Header.Stamp.Nsec), pose_struct)*1e-9 ;
t0 = t(1);
t = t-t0;




bagselect = select(bag, 'Topic',  '/falcon/imu');
imu_struct = readMessages(bagselect,'DataFormat','struct');

gyro = [cellfun(@(m) double(m.AngularVelocity.X), imu_struct), ...
        cellfun(@(m) double(m.AngularVelocity.Y), imu_struct),...
        cellfun(@(m) double(m.AngularVelocity.Z), imu_struct)];

tImu = cellfun(@(m) double(m.Header.Stamp.Sec), imu_struct) + ...
        cellfun(@(m) double(m.Header.Stamp.Nsec), imu_struct)*1e-9;
tImu = tImu-tImu(1);

accel = [cellfun(@(m) double(m.LinearAcceleration.X), imu_struct), ...
        cellfun(@(m) double(m.LinearAcceleration.Y), imu_struct),...
        cellfun(@(m) double(m.LinearAcceleration.Z), imu_struct)];

    

bagselect = select(bag, 'Topic',  '/falcon/log');
log_struct = readMessages(bagselect,'DataFormat','struct');

eul = [cellfun(@(m) double(m.Euler.X), log_struct), ...
        cellfun(@(m) double(m.Euler.Y), log_struct),...
        cellfun(@(m) double(m.Euler.Z), log_struct)];

tEul = cellfun(@(m) double(m.Header.Stamp.Sec), log_struct) + ...
        cellfun(@(m) double(m.Header.Stamp.Nsec), log_struct)*1e-9;
tEul = tEul -tEul (1);

    
%% post-processing
N = length(t);

Rot = quat2rotm(quat);
Eul = zeros(N,3);

for i = 1:N
    Eul(i,:) = Rot2RPY_ZXY(Rot(:,:,i))';
end





%% plots
close all
xlim_ = [0,85];

figure;
subplot(3,1,1); hold on;
plot(t, rad2deg(Eul(:,1)), '.r');
plot(tEul, rad2deg(eul(:,1)), 'b');
grid on; grid minor;xlim(xlim_);
latex_legend({'t265', 'ahrs'});

subplot(3,1,2); hold on;
plot(t, rad2deg(Eul(:,2)), 'r');
plot(tEul, rad2deg(eul(:,2)), 'b');
grid on; grid minor;xlim(xlim_);

subplot(3,1,3); hold on;
plot(t, rad2deg(Eul(:,3)), 'r');
plot(tEul, rad2deg(eul(:,3)), 'b');
grid on; grid minor;
xlim(xlim_);







%% the end

