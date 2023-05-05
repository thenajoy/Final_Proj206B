clear
close all
addpath('../ahrs_m/madgwick_algorithm_matlab/quaternion_library/');
folder = '/home/kotaru/Workspace/catkin_ws/qrotor_ws/bags/may25/';
file = 'white_falcon_2021-05-25-21-26-06.bag';
%%
filename = strcat(folder,file);
bag = rosbag(filename);

% plotQrotor(bag, '/red_falcon');
plotQrotor(bag, '/white_falcon');

keyboard
%%
function plotQrotor(bag, vehicle_name)
    bagselect = select(bag, 'Topic',  strcat(vehicle_name,'/imu'));
    imu_struct = readMessages(bagselect,'DataFormat','struct');

    Gyroscope = [cellfun(@(m) double(m.AngularVelocity.X), imu_struct), ...
            cellfun(@(m) double(m.AngularVelocity.Y), imu_struct),...
            cellfun(@(m) double(m.AngularVelocity.Z), imu_struct)];

    time = cellfun(@(m) double(m.Header.Stamp.Sec), imu_struct) + ...
            cellfun(@(m) double(m.Header.Stamp.Nsec), imu_struct)*1e-9;
    t0 = time(1);
    time = time-t0;

    Accelerometer = [cellfun(@(m) double(m.LinearAcceleration.X), imu_struct), ...
            cellfun(@(m) double(m.LinearAcceleration.Y), imu_struct),...
            cellfun(@(m) double(m.LinearAcceleration.Z), imu_struct)]/9.81;

    pose_mocap = DebagQrotor.readNavmsgsOdometry(bag, strcat(vehicle_name,'/odometry/mocap'));
    t_mocap = pose_mocap.time + pose_mocap.t0-t0;
    quat_mocap = pose_mocap.quat;
    euler_mocap = pose_mocap.Eul;


    pose_t265 = DebagQrotor.readNavmsgsOdometry(bag, strcat(vehicle_name,'/odom/t265'));
    t_t265 = pose_t265.time + pose_t265.t0-t0;
    quat_t265 = pose_t265.quat;
    euler_t265 = pose_t265.Eul;

    pose_est = DebagQrotor.readNavmsgsOdometry(bag, strcat(vehicle_name,'/odom/estimate'));
    t_est = pose_est.time + pose_est.t0-t0;
    quat_est = pose_est.quat;
    euler_est = pose_est.Eul;


    pose_sp = DebagQrotor.readNavmsgsOdometry(bag, strcat(vehicle_name,'/odom/setpoint'));
    t_sp= pose_sp.time + pose_sp.t0-t0;
    quat_sp = pose_sp.quat;
    euler_sp = pose_sp.Eul;

    log = DebagQrotor.readQrotormsgsLog(bag, strcat(vehicle_name,'/log'));
    t_onboard = log.time + log.t0-t0;
    euler_onboard = log.Eul;

    legends = {'mocap', 't265', 'onboard'};
    euler_legend = {'roll', 'pitch', 'yaw'};

    axes_char = ['x', 'y', 'z'];
    ls = {':r', ':b', '-k'};
    lw = [2,2,1];
    xlim_ = [0, 400];

    nsp = 3;
    figure('name', strcat(vehicle_name, '/Eulers'))
    hold on;
    legends = {'mocap', 'estimate', 'onboard'};
    for i = 1:nsp
        subplot(nsp,1,i); hold on;
        plot(t_mocap, euler_mocap(:,i), 'm', 'linewidth', 2);
        plot(t_est, euler_est(:,i), 'b', 'linewidth', 1.5);
        plot(t_onboard, euler_onboard(:,i), 'k', 'linewidth', 1);

        xlim(xlim_);
        latex_title(strcat('$$', euler_legend{i},'$$'), 15);
        latex_ylabel('[deg]');
        latex_legend(legends);
    end

	figure('name', strcat(vehicle_name, '/position'))
    subplot(3,1,1); hold on
    plot(t_sp, pose_sp.pos(:,1), '.k', 'linewidth', 2);
    plot(t_mocap, pose_mocap.pos(:,1), 'm', 'linewidth', 2);
    plot(t_est, pose_est.pos(:,1), 'b', 'linewidth', 1.5);
    latex_title('$$p_x$$',15);
    latex_legend({ 'setpoint', 'mocap', 'estimate'});

    subplot(3,1,2); hold on
    plot(t_sp, pose_sp.pos(:,2), '.k', 'linewidth', 2);
    plot(t_mocap, pose_mocap.pos(:,2), 'm', 'linewidth', 2);
    plot(t_est, pose_est.pos(:,2), 'b', 'linewidth', 1.5);
    latex_title('$$p_y$$',15);
    latex_legend({ 'setpoint', 'mocap', 'estimate'});

    subplot(3,1,3); hold on
    plot(t_sp, pose_sp.pos(:,3), '.k', 'linewidth', 2);
    plot(t_mocap, pose_mocap.pos(:,3), 'm', 'linewidth', 2);
    plot(t_est, pose_est.pos(:,3), 'b', 'linewidth', 1.5);
    latex_title('$$p_z$$',15);
    latex_legend({ 'setpoint', 'mocap', 'estimate'});

    figure('name', strcat(vehicle_name, '/mu_s'))
    hold on;
    for i  = 1:4
        plot(log.PWM(:,i),'linewidth', 1);
    end
    grid on; grid minor;

end

