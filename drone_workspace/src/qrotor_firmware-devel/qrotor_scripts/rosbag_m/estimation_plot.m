clear
% folder = '/home/kotaru/workspace/catkin_wss/qrotor_ws/bags/experiments/';
% folder = '/home/kotaru/workspace/catkin_ws/qrotor_firmware_ws/bags/';
% file = 'att_t265_vid.bag';
% file = '2021-03-09-15-59-19.bag';

folder = '/home/kotaru/Workspace/catkin_ws/qrotor_ws/bags/';
file = 'test.bag';

filename = strcat(folder,file);
bag = rosbag(filename);


t265 = debag_navmsgs_odom(bag, '/falcon/odometry/t265');
% est = debag_navmsgs_odom(bag, '/falcon/odometry/mocap');
% est.t  = est.t+est.t0-t265.t0;

bagselect = select(bag, 'Topic',  '/falcon/log');
log_struct = readMessages(bagselect,'DataFormat','struct');
tLog = cellfun(@(m) double(m.Header.Stamp.Sec), log_struct) + ...in
        cellfun(@(m) double(m.Header.Stamp.Nsec), log_struct)*1e-9;
tLog = tLog-t265.t0;
ahrs_eul = [cellfun(@(m) m.Euler.X, log_struct),...
    cellfun(@(m) m.Euler.Y, log_struct),...
    cellfun(@(m) m.Euler.Z, log_struct)]; 

legend1 = 'mocap';
legend0 = 't265';
euler_legend = {'roll', 'pitch', 'yaw'};

%%
% close all;
figure
subplot(3,1,1); hold on
plot(t265.t, t265.pos(:,1), 'r', 'linewidth', 1);
% plot(est.t, est.pos(:,1), 'b', 'linewidth', 1);
latex_title('$$p_x$$',15);
latex_legend({legend0, legend1});


subplot(3,1,2); hold on
plot(t265.t, t265.pos(:,2), 'r', 'linewidth', 1);
% plot(est.t, est.pos(:,2), 'b', 'linewidth', 1);
latex_title('$$p_y$$',15);
latex_legend({legend0, legend1});

subplot(3,1,3); hold on
plot(t265.t, t265.pos(:,3), 'r', 'linewidth', 1);
% plot(est.t, est.pos(:,3), 'b', 'linewidth', 1);
latex_title('$$p_z$$',15);
latex_legend({legend0, legend1});

axes_char = ['x', 'y', 'z'];
figure;
for i = 1:3
    subplot(3,1,i); hold on;
    plot(t265.t, rad2deg(t265.Eul(:,i)), '-r', 'linewidth', 2);
%     plot(est.t, rad2deg(est.Eul(:,i)), '-b', 'linewidth', 1);
    plot(tLog, rad2deg(ahrs_eul(:,i)), 'k', 'linewidth', 1);
    latex_title(strcat('$$', euler_legend{i},'$$'), 15);
    latex_ylabel('[deg]')
    latex_legend({legend0, legend1, 'ahrs'});
end


figure;
for i = 1:3
    subplot(3,1,i); hold on;
    plot(t265.t, t265.vel(:,i), '-r', 'linewidth', 2);
%     plot(est.t, est.vel(:,i), '-b', 'linewidth', 1);
    latex_title(strcat('$$v_', axes_char(i),'$$'), 15);
    latex_legend({legend0, legend1});
end

