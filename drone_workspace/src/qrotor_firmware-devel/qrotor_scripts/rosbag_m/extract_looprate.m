clear; 
close all

folder = '/home/kotaru/Workspace/catkin_ws/qrotor_ws/bags/experiments/loop_rate_tests/';
filename = 'blue_white_O3.bag';
pic_filename = 'blue_white_O3_blue';

bag = rosbag(strcat(folder,filename));
vehicle_name ="blue_falcon";

bagselect = select(bag, 'Topic',  strcat('/',vehicle_name', '/log'));
log_struct = readMessages(bagselect,'DataFormat','struct');
% 

tLog = cellfun(@(m) double(m.Header.Stamp.Sec), log_struct) + ...in
        cellfun(@(m) double(m.Header.Stamp.Nsec), log_struct)*1e-9;
t0 = tLog(1);
tLog = tLog-t0;
tBagged = bagselect.MessageList.Time-t0;

LoopRate = cellfun(@(m) double(m.LoopRate), log_struct);

mean(LoopRate)
std(LoopRate)

%%
skip = 100;
close all

fig_hdl = figure('name', filename);
subplot(3,1,1); hold on;
scatter(tLog(skip:end), LoopRate(skip:end), 3, 'filled');
plot(tLog(skip:end), mean(LoopRate(skip:end))*ones(size(tLog(skip:end))),...
    'linewidth', 2);
grid on; grid minor;
latex_title('Loop Rate');

subplot(3,1,2);
histogram(LoopRate(skip:end),200);
latex_title('Histogram of loop rate');
grid on; grid minor;

subplot(3,1,3);
plot(tLog, tBagged-tLog);
grid on; grid minor;
latex_title('Baggtime - Loggedtime');

opts.print.index = 4;
opts.print.filename = pic_filename;
opts.print.ext = '-dpng';
print_fig(opts, fig_hdl);




