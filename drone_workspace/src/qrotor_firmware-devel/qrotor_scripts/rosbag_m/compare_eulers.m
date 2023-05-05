clear
folder = '/home/kotaru/workspace/catkin_wss/qrotor_ws/bags/experiments/';
file = 'att_t265.bag';

filename = strcat(folder,file);
bag = rosbag(filename);
dat(1) = debag_navmsgs_odom(bag, '/falcon/odometry/t265');
dat(1).t  = dat(1).t;
t0 = dat(1).t0;

bagselect = select(bag, 'Topic',  '/falcon/log');
log_struct = readMessages(bagselect,'DataFormat','struct');
tLog = cellfun(@(m) double(m.Header.Stamp.Sec), log_struct) + ...in
        cellfun(@(m) double(m.Header.Stamp.Nsec), log_struct)*1e-9;
dat(2).t = tLog-t0;
dat(2).Eul = [cellfun(@(m) m.Euler.X, log_struct),...
    cellfun(@(m) m.Euler.Y, log_struct),...
    cellfun(@(m) m.Euler.Z, log_struct)]; 
dat(2).ang_vel=[cellfun(@(m) m.BodyRates.X, log_struct),...
    cellfun(@(m) m.BodyRates.Y, log_struct),...
    cellfun(@(m) m.BodyRates.Z, log_struct)]; 

% bagselect = select(bag, 'Topic', '/falcon/euler2');
% eul_struct = readMessages(bagselect, 'DataFormat', 'struct');
% teul =  cellfun(@(m) double(m.Header.Stamp.Sec), eul_struct) + ...in
%         cellfun(@(m) double(m.Header.Stamp.Nsec), eul_struct)*1e-9;
% dat(3).t = teul-t0;
% dat(3).quat = [cellfun(@(m) m.Quaternion.W, eul_struct),...
%     cellfun(@(m) m.Quaternion.X, eul_struct),...
%     cellfun(@(m) m.Quaternion.Y, eul_struct),...
%     cellfun(@(m) m.Quaternion.Z, eul_struct)];
% R = quat2rotm(dat(3).quat);
% N = length(dat(3).t);
% dat(3).Eul =  zeros(N,3);
% for i = 1:N
%    dat(3).Eul(i,:) = Rot2RPY_ZXY(R(:,:,i))'; 
% end


legends = {'t265', 'navio-ahrs', 'mahony-rf'};
euler_legend = {'roll', 'pitch', 'yaw'};

%%
close all;

axes_char = ['x', 'y', 'z'];
ls = {'-r', '-b', '-k'};
lw = [2,1.5,1];
m = length(dat);
xlim_ = [0, 125];



figure; hold on;
for i = 1:2
    for j = 1:m
        subplot(2,1,i); hold on;
        plot(dat(j).t, rad2deg(dat(j).Eul(:,i)), ls{j}, 'linewidth', lw(j));
        latex_title(strcat('$$', euler_legend{i},'$$'), 15);
        latex_ylabel('[deg]')
        xlim(xlim_);
    end
    latex_legend(legends);
end


figure; hold on;
for i = 1:3
    for j = 1:m
        subplot(3,1,i); hold on;
        plot(dat(j).t, rad2deg(dat(j).ang_vel(:,i)), ls{j}, 'linewidth', lw(j));
        latex_title(strcat('$$g_',axes_char(i),'$$'), 15);
        latex_ylabel('[deg]')
        xlim(xlim_);
    end
    latex_legend(legends);
end


