clear
folder = '/home/kotaru/workspace/catkin_wss/qrotor_ws/bags/experiments/';
file = 'rs_imu_att.bag';

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


%% post-process
N = length(t);

Rot = quat2rotm(quat);
Eul = quat2eul(quat);

Eul2 = rotm2eul(Rot);

Eul3 = zeros(N,3);
for i = 1:N
    Eul3(i,:) = Rot2RPY_ZXY(Rot(:,:,i))';
end
Eul = Eul3;
    
% tImu = cellfun(@(m) double(m.Header.Stamp.Sec), pose_struct) + ...
%         cellfun(@(m) double(m.Header.Stamp.Nsec), pose_struct)*1e-9;
% tImu = tImu-tImu(1);
% 
% accel = [cellfun(@(m) double(m.LinearAcceleration.X), pose_struct), ...
%         cellfun(@(m) double(m.LinearAcceleration.Y), pose_struct),...
%         cellfun(@(m) double(m.LinearAcceleration.Z), pose_struct)];


%%

close all;
figure;
subplot(1,3,1); hold on;
plot(t, rad2deg(Eul(:,1)), '.r');
grid on; grid minor;

subplot(1,3,2); hold on;
plot(t, rad2deg(Eul(:,2)), 'r');
grid on; grid minor;

subplot(1,3,3); hold on;
plot(t, rad2deg(Eul(:,3)), 'r');
grid on; grid minor;


%%











