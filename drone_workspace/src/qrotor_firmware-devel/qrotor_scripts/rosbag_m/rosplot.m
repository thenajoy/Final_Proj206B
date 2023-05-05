clear

folder = '/home/kotaru/workspace/catkin_ws/qrotor_firmware_ws/bags/experiments/estimation/';
file = 'pose_onbrd_perf.bag';

% folder = '/home/kotaru/Workspace/catkin_ws/qrotor_ws/bags/';
% file = 'exp_pose_estimate1.bag';

filename = strcat(folder,file);
bag = rosbag(filename);
G_SI = 9.80665;

mocap_flag = true;
estimate_flag = 0;

%%

bagselect = select(bag, 'Topic',  '/falcon/log');
log_struct = readMessages(bagselect,'DataFormat','struct');
% 
bagselect = select(bag, 'Topic',  '/falcon/odometry/t265');
odom_struct = readMessages(bagselect,'DataFormat','struct');

bagselect = select(bag, 'Topic',  '/falcon/imu');
imu_struct = readMessages(bagselect,'DataFormat','struct');

if (mocap_flag)
    bagselect = select(bag, 'Topic',  '/falcon/odometry/mocap');
    mocap_struct = readMessages(bagselect,'DataFormat','struct');
end
if (estimate_flag)
    bagselect = select(bag, 'Topic',  '/falcon/odometry/pose_estimate');
    est_struct = readMessages(bagselect,'DataFormat','struct');
end
    



%%

p = [cellfun(@(m) double(m.Pose.Pose.Position.X), odom_struct), ...
        cellfun(@(m) double(m.Pose.Pose.Position.Y), odom_struct),...
        cellfun(@(m) double(m.Pose.Pose.Position.Z), odom_struct)];
    
v = [cellfun(@(m) double(m.Twist.Twist.Linear.X), odom_struct), ...
        cellfun(@(m) double(m.Twist.Twist.Linear.Y), odom_struct),...
        cellfun(@(m) double(m.Twist.Twist.Linear.Z), odom_struct)];
            
ang_vel_t265 = [cellfun(@(m) double(m.Twist.Twist.Angular.X), odom_struct), ...
        cellfun(@(m) double(m.Twist.Twist.Angular.Y), odom_struct),...
        cellfun(@(m) double(m.Twist.Twist.Angular.Z), odom_struct)];
    
q = [cellfun(@(m) double(m.Pose.Pose.Orientation.W), odom_struct), ...
    cellfun(@(m) double(m.Pose.Pose.Orientation.X), odom_struct), ...
        cellfun(@(m) double(m.Pose.Pose.Orientation.Y), odom_struct),...
        cellfun(@(m) double(m.Pose.Pose.Orientation.Z), odom_struct)];
R = quat2rotm(q);
N = length(q);
Eul_t265 = zeros(N,3);
for i = 1:N
    Eul_t265(i,:) = Rot2RPY_ZXY(R(:,:,i))';
end

tOdom = cellfun(@(m) double(m.Header.Stamp.Sec), odom_struct) + ...
        cellfun(@(m) double(m.Header.Stamp.Nsec), odom_struct)*1e-9;
t0 =tOdom(1);
tOdom = tOdom-t0;


if (mocap_flag)
    p_mocap = [cellfun(@(m) double(m.Pose.Pose.Position.X), mocap_struct), ...
            cellfun(@(m) double(m.Pose.Pose.Position.Y), mocap_struct),...
            cellfun(@(m) double(m.Pose.Pose.Position.Z), mocap_struct)];

    v_mocap  = [cellfun(@(m) double(m.Twist.Twist.Linear.X), mocap_struct), ...
            cellfun(@(m) double(m.Twist.Twist.Linear.Y), mocap_struct),...
            cellfun(@(m) double(m.Twist.Twist.Linear.Z), mocap_struct)];

    q_mocap  = [cellfun(@(m) double(m.Pose.Pose.Orientation.W), mocap_struct), ...
        cellfun(@(m) double(m.Pose.Pose.Orientation.X), mocap_struct), ...
            cellfun(@(m) double(m.Pose.Pose.Orientation.Y), mocap_struct),...
            cellfun(@(m) double(m.Pose.Pose.Orientation.Z), mocap_struct)];
    R_mocap  = quat2rotm(q_mocap );
    N = length(q_mocap );
    Eul_mocap = zeros(N,3);
    for i = 1:N
        Eul_mocap (i,:) = Rot2RPY_ZXY(R_mocap (:,:,i))';
    end

    t_mocap = cellfun(@(m) double(m.Header.Stamp.Sec), mocap_struct) + ...
            cellfun(@(m) double(m.Header.Stamp.Nsec), mocap_struct)*1e-9;
    t_mocap = t_mocap-t0; 
end
if (estimate_flag)
    p_est = [cellfun(@(m) double(m.Pose.Pose.Position.X), est_struct), ...
            cellfun(@(m) double(m.Pose.Pose.Position.Y), est_struct),...
            cellfun(@(m) double(m.Pose.Pose.Position.Z), est_struct)];

    v_est = [cellfun(@(m) double(m.Twist.Twist.Linear.X), est_struct), ...
            cellfun(@(m) double(m.Twist.Twist.Linear.Y), est_struct),...
            cellfun(@(m) double(m.Twist.Twist.Linear.Z), est_struct)];

    q_est  = [cellfun(@(m) double(m.Pose.Pose.Orientation.W), est_struct), ...
        cellfun(@(m) double(m.Pose.Pose.Orientation.X), est_struct), ...
            cellfun(@(m) double(m.Pose.Pose.Orientation.Y), est_struct),...
            cellfun(@(m) double(m.Pose.Pose.Orientation.Z), est_struct)];
    R_est  = quat2rotm(q_est );
    N = length(q_est);
    Eul_est= zeros(N,3);
    for i = 1:N
        Eul_est (i,:) = Rot2RPY_ZXY(R_est (:,:,i))';
    end

    t_est = cellfun(@(m) double(m.Header.Stamp.Sec), est_struct) + ...
            cellfun(@(m) double(m.Header.Stamp.Nsec), est_struct)*1e-9;
    t_est = t_est-t0; 
end



tLog = cellfun(@(m) double(m.Header.Stamp.Sec), log_struct) + ...in
        cellfun(@(m) double(m.Header.Stamp.Nsec), log_struct)*1e-9;
tLog = tLog-t0;
LoopRate = cellfun(@(m) double(m.LoopRate), log_struct);
PWM = [cellfun(@(m) m.EscInUs(1), log_struct),...
    cellfun(@(m) m.EscInUs(2), log_struct),...
    cellfun(@(m) m.EscInUs(3), log_struct),...
    cellfun(@(m) m.EscInUs(4), log_struct)];

body_rates = [cellfun(@(m) m.BodyRates.X, log_struct),...
    cellfun(@(m) m.BodyRates.Y, log_struct),...
    cellfun(@(m) m.BodyRates.Z, log_struct)];

Euler = [cellfun(@(m) m.Euler.X, log_struct),...
    cellfun(@(m) m.Euler.Y, log_struct),...
    cellfun(@(m) m.Euler.Z, log_struct)];

lin_accel = [cellfun(@(m) m.LinearAcceleration.X, log_struct),...
    cellfun(@(m) m.LinearAcceleration.Y, log_struct),...
    cellfun(@(m) m.LinearAcceleration.Z, log_struct)];

voltage = cellfun(@(m) m.Voltage, log_struct);
current = cellfun(@(m) m.Current, log_struct);
power = voltage.*current;


gyro = [cellfun(@(m) double(m.AngularVelocity.X), imu_struct), ...
        cellfun(@(m) double(m.AngularVelocity.Y), imu_struct),...
        cellfun(@(m) double(m.AngularVelocity.Z), imu_struct)];

tImu = cellfun(@(m) double(m.Header.Stamp.Sec), imu_struct) + ...
        cellfun(@(m) double(m.Header.Stamp.Nsec), imu_struct)*1e-9;
tImu = tImu-t0;

accel = [cellfun(@(m) double(m.LinearAcceleration.X), imu_struct), ...
        cellfun(@(m) double(m.LinearAcceleration.Y), imu_struct),...
        cellfun(@(m) double(m.LinearAcceleration.Z), imu_struct)];

    
%%

DebagQrotor.plotPowerReadings(tLog, voltage, current);

legnds = {'t265'};
if (mocap_flag)
    legnds= {legnds{:}, 'mocap'};
end
if (estimate_flag)
    legnds= {legnds{:}, 'estimate'};
end
skip = 1;
close all
%%
figure('name', 'positions');
subplot(3,1,1);
hold on;
plot(tOdom(skip:end), p(skip:end,1), 'b', 'linewidth', 1);
if mocap_flag
    plot(t_mocap(skip:end), p_mocap(skip:end,1), 'r', 'linewidth', 1);
end
if estimate_flag
    plot(t_est(skip:end), p_est(skip:end,1), 'g', 'linewidth', 1);
end
latex_title('$$p_x$$');
latex_legend(legnds);

subplot(3,1,2);
hold on;
plot(tOdom(skip:end), p(skip:end,2), 'b', 'linewidth', 1);
if mocap_flag
    plot(t_mocap(skip:end), p_mocap(skip:end,2), 'r', 'linewidth', 1);
end
if estimate_flag
    plot(t_est(skip:end), p_est(skip:end,2), 'g', 'linewidth', 1);
end
latex_title('$$p_y$$');
latex_legend(legnds);

subplot(3,1,3);
hold on;
plot(tOdom(skip:end), p(skip:end,3), 'b', 'linewidth', 1);
if mocap_flag
    plot(t_mocap(skip:end), p_mocap(skip:end,3), 'r', 'linewidth', 1);
end
if estimate_flag
    plot(t_est(skip:end), p_est(skip:end,3), 'g', 'linewidth', 1);
end
latex_title('$$p_z$$');
latex_legend(legnds);

%%
figure('name', 'velocities');
subplot(3,1,1);
hold on;
plot(tOdom(skip:end), v(skip:end,1), 'b', 'linewidth', 1);
if mocap_flag
    plot(t_mocap(skip:end), v_mocap(skip:end,1), 'r', 'linewidth', 1);
end
if estimate_flag
    plot(t_est(skip:end), v_est(skip:end,1), 'g', 'linewidth', 1);
end
latex_title('$$v_x$$');
latex_legend(legnds);;

subplot(3,1,2);
hold on;
plot(tOdom(skip:end), v(skip:end,2), 'b', 'linewidth', 1);
if mocap_flag
    plot(t_mocap(skip:end), v_mocap(skip:end,2), 'r', 'linewidth', 1);
end
if estimate_flag
    plot(t_est(skip:end), v_est(skip:end,2), 'g', 'linewidth', 1);
end
latex_title('$$v_y$$');
latex_legend(legnds);;

subplot(3,1,3);
hold on;
plot(tOdom(skip:end), v(skip:end,3), 'b', 'linewidth', 1);
if mocap_flag
    plot(t_mocap(skip:end), v_mocap(skip:end,3), 'r', 'linewidth', 1);
end
if estimate_flag
    plot(t_est(skip:end), v_est(skip:end,3), 'g', 'linewidth', 1);
end
latex_title('$$v_z$$');
latex_legend(legnds);;

%%

flog = figure('name', 'esc n loop-rate');
subplot(2,2,[1,2]); hold on;
l = plot(tLog, PWM(:,1)); l.Color(4) = 0.2;
l = plot(tLog, PWM(:,2)); l.Color(4) = 0.2;
l = plot(tLog, PWM(:,3)); l.Color(4) = 0.2;
l = plot(tLog, PWM(:,4)); l.Color(4) = 0.2;
latex_title('pwm signals in $$\mu_s$$');
latex_legend({'1', '2', '3', '4'});

subplot(2,2,3); hold on;
plot(tLog(skip:end), LoopRate(skip:end), '.r');
latex_title('Loop Rate');

subplot(2,2,4);
histogram(LoopRate(skip:end));
latex_title('Histogram of loop rate');

%%
figure('name', 'attitude');
subplot(3,1,1); hold on;
l = plot(tLog, Euler(:,1)*180/pi, 'b', 'linewidth', 2); l.Color(4) = 0.4;
plot(tOdom, Eul_t265(:,1)*180/pi, 'r');
latex_title('roll: $$\phi$$'); 
latex_legend({'onboard-used-in-ctrl', 't265'});
subplot(3,1,2); hold on;
l =plot(tLog, Euler(:,2)*180/pi, 'b', 'linewidth', 2); l.Color(4) = 0.4;
plot(tOdom, Eul_t265(:,2)*180/pi, 'r'); 
latex_title('pitch: $$\theta$$'); 
latex_legend({'onboard-used-in-ctrl', 't265'});
subplot(3,1,3); hold on;
l =plot(tLog, Euler(:,3)*180/pi, 'b', 'linewidth', 2); l.Color(4) = 0.4;
plot(tOdom, Eul_t265(:,3)*180/pi, 'r'); 
latex_title('yaw: $$\psi$$'); 
latex_legend({'onboard-used-in-ctrl', 't265'});
latex_xlabel('Time[s]');
latex_ylabel('[deg]');

%%
figure('name', 'ang_vel');
subplot(3,1,1); hold on;
l = plot(tImu, gyro(:,1), 'b' ,'linewidth', 1);l.Color(4)= 0.5;
plot(tLog, body_rates(:,1), 'r','linewidth', 1.5)
plot(tOdom, ang_vel_t265(:,1), 'g','linewidth', 1); 
latex_title('$$g_x$$');
latex_legend({'raw-from-imu', 'filtered-used-in-ctrl', 't265'});

subplot(3,1,2); hold on;
l = plot(tImu, gyro(:,2),'b', 'linewidth', 1);l.Color(4)= 0.5;
plot(tLog, body_rates(:,2), 'r','linewidth', 1)
plot(tOdom, ang_vel_t265(:,2), 'g','linewidth', 1); 
latex_title('$$g_y$$');
latex_legend({'raw-from-imu', 'filtered-used-in-ctrl', 't265'});

subplot(3,1,3); hold on;
l = plot(tImu, gyro(:,3),'b','linewidth', 1);l.Color(4)= 0.5;
plot(tLog, body_rates(:,3), 'r','linewidth', 1)
plot(tOdom, ang_vel_t265(:,3), 'g','linewidth', 1); 
latex_title('$$g_z$$');
latex_legend({'raw-from-imu', 'filtered-used-in-ctrl', 't265'});
latex_ylabel('[rad/s]');
latex_xlabel('Time [s]');

%%
figure('name', 'accelerometer');
subplot(3,1,1); hold on;
plot(tImu, accel(:,1), 'linewidth', 1);
plot(tLog, lin_accel(:,1)*G_SI, 'r','linewidth', 1)
latex_legend({'$$a_x$$', '$${a_f}_x$$'});
subplot(3,1,2); hold on;
plot(tImu, accel(:,2), 'linewidth', 1);
plot(tLog, lin_accel(:,2)*G_SI, 'r','linewidth', 1)
latex_legend({'$$a_y$$', '$${a_f}_y$$'});
subplot(3,1,3); hold on;
plot(tImu, accel(:,3), 'linewidth', 1);
plot(tLog, lin_accel(:,3)*G_SI, 'r','linewidth', 1)
latex_legend({'$$a_z$$', '$${a_f}_z$$'});

%% 



%%
accel_norm = norm2(accel,2);
figure
subplot(1,2,1); hold on;
plot(tImu, accel_norm);
plot(tImu, 9.8*ones(size(tImu)), 'linewidth', 3)
voltage = cellfun(@(m) m.Voltage, log_struct);
current = cellfun(@(m) m.Current, log_struct);

gyro_norm = norm2(gyro,2);
subplot(1,2,2); hold on;
plot(tImu, gyro_norm);
plot(tImu, zeros(size(tImu)), 'linewidth', 3)



