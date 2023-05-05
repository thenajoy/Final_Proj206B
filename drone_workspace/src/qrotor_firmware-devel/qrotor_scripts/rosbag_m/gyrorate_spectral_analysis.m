clear; close all
folder = '/home/kotaru/workspace/catkin_wss/qrotor_ws/bags/experiments';

dat = load('imu.mat');

t = dat.tImu;
t = t - t(1);
fs = 1 / mean(diff(t));
dgyro = diff(dat.gyro);
dt = diff(t);
gyro_rate = dgyro./dt;

axis_val = ['x', 'y', 'z'];
f1 = figure;
hold on;
for i = 1:3
    x = gyro_rate(:, i);
    y = x;

    subplot(3, 2, 2*i-1); hold on
    [f, p] = fft_analysis(y, fs);
%     ylim([0, 10000]);
    latex_title(strcat('$$\dot{g}_',axis_val(i),'$$'),18);

end
latex_xlabel('Freq (Hz)') ; 
latex_ylabel('abs(fft(q))') ;

for i = 1:3
    x = dat.ang_accel(:, i);
    y = x;

    subplot(3, 2, 2*i); hold on
    [f, p] = fft_analysis(y, fs);
%     ylim([0, 10000]);
    latex_title(strcat('$$\dot{\Omega}_',axis_val(i),'$$'),18);

end

clear dt t
%%
Omega = dat.body_rates;
tOmega = dat.tLog;
fs = 1 / mean(diff(tOmega));
dt = diff(tOmega);
dOmega = diff(Omega);
Omega_rate = dOmega./dt;
t = tOmega(2:end);

f2 = figure; hold on;
hold on;
for i = 1:3
    x = Omega_rate(:, i);
    y = x;
%     y = lowpass(y, 40, fs);
    y = notch_filter(y, fs, 80, 0.0);
%     y = apply_lpf_so(t, y,fs, 60);
%     y = notch_filter(y, fs, 40, 0.75);


    subplot(3, 1, i); hold on
    [f, p] = fft_analysis(y, fs);
%     ylim([0, 1000]);
    latex_title(strcat('$$\dot{\Omega}_',axis_val(i),'$$'),18);
end
latex_xlabel('Freq (Hz)') ; 

%%
% opts.print_pos_sz = [0.25, 0.25, 10, 5];
% opts.print.filename = 'tex/imgs/fft_imu_test2';
% print_fig(opts, f1);
