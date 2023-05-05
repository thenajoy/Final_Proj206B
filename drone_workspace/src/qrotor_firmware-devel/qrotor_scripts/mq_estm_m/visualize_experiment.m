clear;
close all;
addpath('../');

%%

load('../multi_quad_m/data_test1.mat');
t0 = data.red.odom.estm.t0;

quad = struct();
quad(1).odom = [data.red.odom.estm.pos, data.red.odom.estm.quat];
quad(1).t = data.red.odom.estm.time -t0 + data.red.odom.estm.t0;
quad(1).flag = true;

quad(2).odom = [data.white.odom.estm.pos, data.white.odom.estm.quat];
quad(2).t = data.white.odom.estm.time -t0 + data.white.odom.estm.t0;
quad(2).flag = true;


%% 
close all

figure;
for i = 1:3
    subplot(3,1,i); hold on;
    plot(quad(1).t, quad(1).odom(:,i), 'linewidth', 2); 
    plot(quad(2).t, quad(2).odom(:,i), 'linewidth', 2); 
    
    latex_legend({'$$red$$', '$$white$$'});
    grid on; grid minor;

end

