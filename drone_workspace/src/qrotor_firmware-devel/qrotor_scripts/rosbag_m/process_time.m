% pose estimation
clear

folder = '/home/kotaru/workspace/catkin_ws/qrotor_firmware_ws/bags/experiments/estimation/';
% folder = '/home/kotaru/Workspace/catkin_ws/qrotor_ws/bags/';
file = 'pose_onbrd_perf.bag';

filename = strcat(folder,file);
bag = rosbag(filename);

%%

imu = debag_sensormsgs_imu(bag, '/falcon/imu');
rb_tcam = debag_navmsgs_odom(bag, '/falcon/odometry/t265');
rb_mocap = debag_navmsgs_odom(bag, '/falcon/odometry/mocap');

%% process the time-line

imu_t = imu.time;
tcam_t = rb_tcam.t+rb_tcam.t0-imu.t0;
tmocap_t = rb_mocap.t+rb_mocap.t0-imu.t0;

Ni = length(imu_t);
Nc = length(tcam_t);
Nm = length(tmocap_t);

datum = [[imu_t, zeros(Ni,1), linspace(1,Ni,Ni)'];...
            [tcam_t, ones(Nc,1), linspace(1,Nc,Nc)'];...
            [tmocap_t, 2*ones(Nm,1), linspace(1,Nm,Nm)']];
datum = sortrows(datum,1);
id0 = find(datum(:,1)>=0,1,'first');
datum(1:id0-1,:)=[];

save('data.mat');

