% ExampleScript.m
%
% This script demonstrates use of the MadgwickAHRS and MahonyAHRS algorithm
% classes with example data. ExampleData.mat contains calibrated gyroscope,
% accelerometer and magnetometer data logged from an AHRS device (x-IMU)
% while it was sequentially rotated from 0 degrees, to +90 degree and then
% to -90 degrees around the X, Y and Z axis.  The script first plots the
% example sensor data, then processes the data through the algorithm and
% plots the output as Euler angles.
%
% Note that the Euler angle plot shows erratic behaviour in phi and psi
% when theta approaches ±90 degrees. This due to a singularity in the Euler
% angle sequence known as 'Gimbal lock'.  This issue does not exist for a
% quaternion or rotation matrix representation.
%
% Date          Author          Notes
% 28/09/2011    SOH Madgwick    Initial release
% 13/04/2012    SOH Madgwick    deg2rad function no longer used
% 06/11/2012    Seb Madgwick    radian to degrees calculation corrected

%% Start of script

addpath('quaternion_library');      % include quaternion library
% close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% Import and plot sensor data

load('../../rosbag_m/Imu.mat');
% 
% figure('Name', 'Sensor Data');
% axis(1) = subplot(2,1,1);
% hold on;
% plot(time, Gyroscope(:,1), 'r');
% plot(time, Gyroscope(:,2), 'g');
% plot(time, Gyroscope(:,3), 'b');
% legend('X', 'Y', 'Z');
% xlabel('Time (s)');
% ylabel('Angular rate (deg/s)');
% title('Gyroscope');
% hold off;
% axis(2) = subplot(2,1,2);
% hold on;
% plot(time, Accelerometer(:,1), 'r');
% plot(time, Accelerometer(:,2), 'g');
% plot(time, Accelerometer(:,3), 'b');
% legend('X', 'Y', 'Z');
% xlabel('Time (s)');
% ylabel('Acceleration (g)');
% title('Accelerometer');
% hold off;
% % axis(3) = subplot(3,1,3);
% % hold on;
% % plot(time, Magnetometer(:,1), 'r');
% % plot(time, Magnetometer(:,2), 'g');
% % plot(time, Magnetometer(:,3), 'b');
% % legend('X', 'Y', 'Z');
% % xlabel('Time (s)');
% % ylabel('Flux (G)');
% % title('Magnetometer');
% % hold off;
% linkaxes(axis, 'x');

%% Process sensor data through algorithm
dt = mean(diff(time));
for iter = 1
    beta = 0.025;
    kp = 0.2;
    Madgwick = MadgwickAHRS('SamplePeriod', dt, 'Beta', beta);
    Mahony = MahonyAHRS('SamplePeriod', dt, 'Kp',kp, 'Ki', 0.01 );

    quaternion{1} = zeros(length(time), 4);
    quaternion{2} = zeros(length(time), 4);
    for t = 1:length(time)
        Madgwick.UpdateIMU(Gyroscope(t,:), Accelerometer(t,:));
        Mahony.UpdateIMU(Gyroscope(t,:), Accelerometer(t,:));

        quaternion{1}(t, :) = Madgwick.Quaternion;
        quaternion{2}(t, :) = Mahony.Quaternion;
    end

    %% Plot algorithm output as Euler angles
    % The first and third Euler angles in the sequence (phi and psi) become
    % unreliable when the middle angles of the sequence (theta) approaches ±90
    % degrees. This problem commonly referred to as Gimbal Lock.
    % See: http://en.wikipedia.org/wiki/Gimbal_lock

    for i = 1:length(quaternion)
        % use conjugate for sensor frame relative to Earth and convert to degrees.
        euler{i} = quatern2euler(quaternConj(quaternion{i})) * (180/pi);	
    end
    Eultrue = quatern2euler(quaternConj(qtrue))*180/pi;
    Eul_rf = quatern2euler(quaternConj(quat_rf))*180/pi;
    Eul_mgwk = quatern2euler(quaternConj(quat_mgwk))*180/pi;
%     euler{3} = 0.5*(euler{1}+euler{2});

    %%
    close all;
    legends = {'t265', 'mgwk-mat', 'mahony-mat', 'onboard', 'mahony-onb', 'madgwick-onb'};
    euler_legend = {'roll', 'pitch', 'yaw'};

    axes_char = ['x', 'y', 'z'];
    ls = {':r', ':b', '-k'};
    lw = [2,2,1];
    m = length(euler);
    xlim_ = [0, 200];

    nsp = 2;
    figure('Name', strcat('Beta: ',num2str(beta), ' Kp ', num2str(kp)));
     hold on;
    for i = 1:nsp
        subplot(nsp,1,i); hold on;

        l = plot(ttrue, Eultrue(:,i), '-m', 'linewidth', 2.5);% l.Color(4) = 0.4;%
        for j = 1:m
            ang = euler{j}(:,i);
            plot(time, (ang), ls{j}, 'linewidth', lw(j));
        end
        plot(tEul, rad2deg(Eul(:,i)), '-k', 'linewidth', 1);
        plot(t_rf, Eul_rf(:,i), '-r', 'linewidth', 1);
        plot(t_mgwk, Eul_mgwk(:,i), '-b', 'linewidth', 1);
        xlim(xlim_);

        latex_title(strcat('$$', euler_legend{i},'$$'), 15);
        latex_ylabel('[deg]');
        latex_legend(legends);
    end
end

%% End of script