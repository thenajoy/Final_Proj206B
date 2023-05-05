clear

folder = '/home/kotaru/workspace/catkin_ws/qrotor_firmware_ws/bags/experiments/estimation/';
file = 'pose_offbrd_eign_perf.bag';

% folder = '/home/kotaru/Workspace/catkin_ws/qrotor_ws/bags/';
% file = 'exp_pose_estimate1.bag';

filename = strcat(folder,file);
bag = rosbag(filename);

dat(1) = debag_navmsgs_odom(bag, '/falcon/odometry/t265');
dat(2) = debag_navmsgs_odom(bag, '/falcon/odometry/pose_estimate');
dat(3) = debag_navmsgs_odom(bag, '/falcon/odometry/mocap');

m = length(dat);
for i = 2:m
    dat(i).t=dat(i).t+dat(i).t0-dat(1).t0;
end
    
    
%%
lgnds = {'t265', 'est', 'mocap'};

colors = {'r', 'b', 'g'};
lw = [2, 1.5, 1];

close all
skip = 1;

figure('name', 'positions');
ax = subplot(3,1,1); hold on;
for j = 1:m
  plot(dat(j).t(skip:end),  dat(j).pos(skip:end,1), colors{j}, 'linewidth', lw(j))
end
grid on; grid minor;
latex_legend(lgnds);
latex_title('$$p_x$$');

subplot(3,1,2); hold on;
for j = 1:m
  plot(dat(j).t(skip:end),  dat(j).pos(skip:end,2), colors{j}, 'linewidth', lw(j))
end
grid on; grid minor;
latex_legend(lgnds);
latex_title('$$p_y$$');

subplot(3,1,3); hold on;
for j = 1:m
  plot(dat(j).t(skip:end),  dat(j).pos(skip:end,3), colors{j}, 'linewidth', lw(j))
end
grid on; grid minor;
latex_legend(lgnds);
latex_title('$$p_z$$');


figure('name', 'velocity');
ax = subplot(3,1,1); hold on;
for j = 1:m
  plot(dat(j).t(skip:end),  dat(j).vel(skip:end,1), colors{j}, 'linewidth', lw(j))
end
grid on; grid minor;
latex_legend(lgnds);
latex_title('$$v_x$$');

subplot(3,1,2); hold on;
for j = 1:m
  plot(dat(j).t(skip:end),  dat(j).vel(skip:end,2), colors{j}, 'linewidth', lw(j))
end
grid on; grid minor;
latex_legend(lgnds);
latex_title('$$v_y$$');

subplot(3,1,3); hold on;
for j = 1:m
  plot(dat(j).t(skip:end),  dat(j).vel(skip:end,3), colors{j}, 'linewidth', lw(j))
end
grid on; grid minor;
latex_legend(lgnds);
latex_title('$$v_z$$');


figure('name', 'Euler');
ax = subplot(3,1,1); hold on;
for j = 1:m
  plot(dat(j).t(skip:end),  dat(j).Eul(skip:end,1), colors{j}, 'linewidth', lw(j))
end
grid on; grid minor;
latex_legend(lgnds);
latex_title('$$\phi$$');

subplot(3,1,2); hold on;
for j = 1:m
  plot(dat(j).t(skip:end),  dat(j).Eul(skip:end,2), colors{j}, 'linewidth', lw(j))
end
grid on; grid minor;
latex_legend(lgnds);
latex_title('$$\theta$$');

subplot(3,1,3); hold on;
for j = 1:m
  plot(dat(j).t(skip:end),  dat(j).Eul(skip:end,3), colors{j}, 'linewidth', lw(j))
end
grid on; grid minor;
latex_legend(lgnds);
latex_title('$$\psi$$');






