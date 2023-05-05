clear;
% folder = '/home/kotaru/workspace/catkin_wss/qrotor_ws/bags/experiments/';
folder = '/home/kotaru/workspace/catkin_ws/qrotor_firmware_ws/bags/';
file = 'sim1.bag';

filename = strcat(folder,file);
bag = rosbag(filename);

dat(1) = debag_navmsgs_odom(bag, '/gazebo_falcon/odom/ground_truth');


bagselect = select(bag, 'Topic',  '/gazebo_falcon/mahony_rf');
mahony_struct = readMessages(bagselect,'DataFormat','struct');

dat(2).quat = [cellfun(@(m) double(m.Quaternion.W), mahony_struct), ...
        cellfun(@(m) double(m.Quaternion.X), mahony_struct), ...
        cellfun(@(m) double(m.Quaternion.Y), mahony_struct),...
        cellfun(@(m) double(m.Quaternion.Z), mahony_struct)];

dat(2).t = cellfun(@(m) double(m.Header.Stamp.Sec), mahony_struct) ...
    + cellfun(@(m) double(m.Header.Stamp.Nsec), mahony_struct)*1e-9 -dat(1).t0;
N = length(dat(2).t);
dat(2).Eul = zeros(N,3);
R = quat2rotm(dat(2).quat);
for i = 1:N
    dat(2).Eul(i,:) = Rot2RPY_ZXY(R(:,:,i))';
end


bagselect = select(bag, 'Topic',  '/gazebo_falcon/log');
log_struct = readMessages(bagselect,'DataFormat','struct');
dat(3).t = cellfun(@(m) double(m.Header.Stamp.Sec), log_struct) ...
    + cellfun(@(m) double(m.Header.Stamp.Nsec), log_struct)*1e-9 -dat(1).t0;
dat(3).Eul = [cellfun(@(m) double(m.Euler.X), log_struct), ...
        cellfun(@(m) double(m.Euler.Y), log_struct),...
        cellfun(@(m) double(m.Euler.Z), log_struct)];


%%
% close all;
legends = {'truth', 'mahony', 'ahrs'};
euler_legend = {'roll', 'pitch', 'yaw'};

axes_char = ['x', 'y', 'z'];
ls = {'-r', '-b', '-k'};
lw = [2,1.5,1];
m = length(dat);
% xlim_ = [0, 125];

nsp = 3;
figure; hold on;
for i = 1:nsp
    for j = 1:m
        subplot(nsp,1,i); hold on;
        plot(dat(j).t, rad2deg(dat(j).Eul(:,i)), ls{j}, 'linewidth', lw(j));
        latex_title(strcat('$$', euler_legend{i},'$$'), 15);
        latex_ylabel('[deg]');
%         xlim(xlim_);
    end
    latex_legend(legends);
end






