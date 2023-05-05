clear
folder = '/home/kotaru/workspace/catkin_ws/qrotor_firmware_ws/bags/';
file = 'pose_onbrd_perf.bag';

filename = strcat(folder,file);
bag = rosbag(filename);

bagselect = select(bag, 'Topic',  '/falcon/imu');
imu_struct = readMessages(bagselect,'DataFormat','struct');
bagselect = select(bag, 'Topic',  '/falcon/imu_filtered');
imuf_struct = readMessages(bagselect,'DataFormat','struct');

%%

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

    
 
Gyro_f = [cellfun(@(m) double(m.AngularVelocity.X), imuf_struct), ...
        cellfun(@(m) double(m.AngularVelocity.Y), imuf_struct),...
        cellfun(@(m) double(m.AngularVelocity.Z), imuf_struct)];

time_f = cellfun(@(m) double(m.Header.Stamp.Sec), imuf_struct) + ...
        cellfun(@(m) double(m.Header.Stamp.Nsec), imuf_struct)*1e-9;
    t0 = time_f(1);
time_f = time_f-t0;

Accel_f = [cellfun(@(m) double(m.LinearAcceleration.X), imuf_struct), ...
        cellfun(@(m) double(m.LinearAcceleration.Y), imuf_struct),...
        cellfun(@(m) double(m.LinearAcceleration.Z), imuf_struct)]/9.81;
   
%%
close all

figure; 
subplot(3,1,1); hold on;
plot(time, Gyroscope(:,1), 'b', 'linewidth', 1);
plot(time_f, Gyro_f(:,1), 'r', 'linewidth', 2);
grid on; grid minor;
latex_title('$$g_x$$');

subplot(3,1,2); hold on;
plot(time, Gyroscope(:,2), 'b', 'linewidth', 1);
plot(time_f, Gyro_f(:,2), 'r', 'linewidth', 2);
grid on; grid minor;
latex_title('$$g_y$$');

subplot(3,1,3); hold on;
plot(time, Gyroscope(:,3), 'b', 'linewidth', 1);
plot(time_f, Gyro_f(:,3), 'r', 'linewidth', 2);
grid on; grid minor;
latex_title('$$g_z$$');

figure; 
subplot(3,1,1); hold on;
plot(time, Accelerometer(:,1), 'b', 'linewidth', 1);
plot(time_f, Accel_f(:,1), 'r', 'linewidth', 2);
grid on; grid minor;
latex_title('$$a_x$$');

subplot(3,1,2); hold on;
plot(time, Accelerometer(:,2), 'b', 'linewidth', 1);
plot(time_f, Accel_f(:,2), 'r', 'linewidth', 2);
grid on; grid minor;
latex_title('$$a_y$$');

subplot(3,1,3); hold on;
plot(time, Accelerometer(:,3), 'b', 'linewidth', 1);
plot(time_f, Accel_f(:,3), 'r', 'linewidth', 2);
grid on; grid minor;
latex_title('$$a_z$$');







    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
