clear

folder = '/home/kotaru/Workspace/catkin_ws/qrotor_ws/bags/apr12/';
file1 = 'looprate.bag';
file2 = 'looprate2.bag';

bag1 = rosbag(strcat(folder,file1));
bag2 = rosbag(strcat(folder,file2));


bagselect = select(bag1, 'Topic',  '/falcon/loop_rate_');
msg_struct1 = readMessages(bagselect,'DataFormat','struct');
loop_rate1 = cellfun(@(m) double(m.Data), msg_struct1);

bagselect = select(bag2, 'Topic',  '/falcon/loop_rate_');
msg_struct2 = readMessages(bagselect,'DataFormat','struct');
loop_rate2 = cellfun(@(m) double(m.Data), msg_struct2);

%%
close all
figure;
hold on;
subplot(2,1,1);
plot(loop_rate1, 'linewidth' ,1);
subplot(2,1,2)
plot(loop_rate2,'linewidth', 1);

figure; hold on;
histogram(loop_rate1,500,'Normalization','probability'); 
histogram(loop_rate2,'Normalization','probability');
xlim([0,500]);
latex_legend({'1 meas update', '2 meas updates'});



