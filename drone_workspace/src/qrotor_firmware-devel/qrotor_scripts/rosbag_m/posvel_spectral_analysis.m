clear; close all
dat = load('pv_gnd_prop_hover.mat');
t = dat.tOdom;
t = t - t(1);
fs = 1 / mean(diff(t));

axis_val = ['x', 'y', 'z'];
f1 = figure;
hold on;
for i = 1:3
    x = dat.p(:, i);
    y = x;

    subplot(3, 2, 2*i-1); hold on
    [f, p] = fft_analysis(y, fs);
    ylim([0, 200]);
    latex_title(strcat('$$p_',axis_val(i),'$$'),18);
    
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
    x = dat.v(:, i);
    y = x;

    subplot(3, 2, 2*i); hold on
    [f, p] = fft_analysis(y, fs);
    ylim([0, 200]);
    latex_title(strcat('$$v_',axis_val(i),'$$'),18);
end
latex_xlabel('Freq (Hz)') ; 
% latex_ylabel('abs(fft(q))') ;



opts.print_pos_sz = [0.25, 0.25, 10, 5];
opts.print.filename = 'tex/imgs/fft_pv_test2';
print_fig(opts, f1);

%%
%
