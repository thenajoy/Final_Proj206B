% clear; close all
folder = '/home/kotaru/workspace/catkin_wss/qrotor_ws/bags/experiments';

dat = load('imu.mat');
t = dat.tImu;
t = t - t(1);
fs = 1 / mean(diff(t));

axis_val = ['x', 'y', 'z'];
a0 = [0;0;9.8];
f1 = figure;
hold on;
for i = 1:3
    x = dat.accel(:, i)-a0(i);
    y = x;
    y = notch_filter(y, fs, 80, 0.75);
    y = apply_lpf_so(t, y,fs, 100);

    subplot(3, 2, 2*i-1); hold on
    [f, p] = fft_analysis(y, fs);
%     ylim([0, 10000]);
    latex_title(strcat('$$a_',axis_val(i),'$$'),18);
    
%     if i==2
%         subplot(3,2,3); hold on;
%         % create a new pair of axes inside current figure
%         axes('position',[.34 .475 .15 .15])
%         box on % put box around new pair of axes
%         indexOfInterest = (f > 50) & (f < 125); % range of t near perturbation
%         stem(f(indexOfInterest),abs(p(indexOfInterest)), '.') % plot on new axes
%         axis tight
%     end
end
latex_xlabel('Freq (Hz)') ; 
latex_ylabel('abs(fft(q))') ;


for i = 1:3
    x = dat.lin_accel(:, i)-a0(i);
    y = x;

    subplot(3, 2, 2*i); hold on
    [f, p] = fft_analysis(y, fs);
%     ylim([0, 10000]);
    latex_title(strcat('filtered-$$a_',axis_val(i),'$$'),18);
end

%%
f2 = figure;
hold on;
for i = 1:3
    x = dat.gyro(:, i);
    y = x;

    subplot(3, 2, 2*i-1); hold on
    [f, p] = fft_analysis(y, fs);
%     ylim([0, 1000]);
    latex_title(strcat('$$g_',axis_val(i),'$$'),18);
end
latex_xlabel('Freq (Hz)') ; 
% latex_ylabel('abs(fft(q))') ;

%%
if isfield(dat,'body_rates')
    Omega = dat.body_rates;
    tOmega = dat.tLog;
    fs = 1 / mean(diff(tOmega));

    hold on;
    for i = 1:3
        x = Omega(:, i);
        y = x;

        subplot(3, 2, 2*i); hold on
        [f, p] = fft_analysis(y, fs);
%         ylim([0, 1000]);
        latex_title(strcat('$$\Omega_',axis_val(i),'$$'),18);
    end
    latex_xlabel('Freq (Hz)') ; 
end

%%
% opts.print_pos_sz = [0.25, 0.25, 10, 5];
% opts.print.filename = 'tex/imgs/fft_imu_test2';
% print_fig(opts, f1);
