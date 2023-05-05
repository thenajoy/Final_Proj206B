clear;
close all;
addpath('../');

%%
folder = '/home/kotaru/Workspace/catkin_ws/qrotor_ws/bags/';

file_payload = 'jun08/payload_estm_test1_jun08.bag';
file_redfalcon = 'jun08/red_falcon_estm_test1_jun08.bag';
file_whitefalcon = 'jun08/white_falcon_estm_test1_jun08.bag';

%%

filename = strcat(folder, file_payload);
bag_payload = rosbag(filename);
data.payload.odom = ros.DebagQrotor.readNavmsgsOdometry(bag_payload, '/payload/odometry/mocap');

filename = strcat(folder, file_redfalcon);
bag_redfalcon = rosbag(filename);
data.red.odom.mocap = ros.DebagQrotor.readNavmsgsOdometry(bag_redfalcon, '/red_falcon/odometry/mocap');
data.red.odom.estm = ros.DebagQrotor.readNavmsgsOdometry(bag_redfalcon, '/red_falcon/odom/estimate');
data.red.log = ros.DebagQrotor.readQrotormsgsLog(bag_redfalcon, '/red_falcon/log');
data.red.imu = ros.DebagQrotor.readSensormsgsImu(bag_redfalcon,'/red_falcon/imu');


filename = strcat(folder, file_whitefalcon);
bag_white = rosbag(filename);
data.white.odom.mocap = ros.DebagQrotor.readNavmsgsOdometry(bag_white, '/white_falcon/odometry/mocap');
data.white.odom.estm = ros.DebagQrotor.readNavmsgsOdometry(bag_white, '/white_falcon/odom/estimate');
data.white.log = ros.DebagQrotor.readQrotormsgsLog(bag_white, '/white_falcon/log');
data.white.imu = ros.DebagQrotor.readSensormsgsImu(bag_white,'/white_falcon/imu');

% plotQrotor(bag, '/red_falcon');
% plotQrotor(bag, '/white_falcon');

save('data_test1.mat', 'data');

%% 
% end of the file!





