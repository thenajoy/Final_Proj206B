clear;
close all
%%
T = readtable('3prop_force_calib.csv');

esc = T.ESCSignal__s_; % in micro-seconds range(1000-2000);
rpm = T.MotorElectricalSpeed_RPM_; % rpm
w = rpm*2*pi/60; % rad/s

% single motor
thrust_kgf = T.Thrust_kgf_;
thrust = -thrust_kgf*g;  % N
torque = T.Torque_N_m_;

%

%% compute the trends
close all;

e = 1000:100:2000;
f = [[0:0.01:0.9],[1:1:8]];
% forces/torques to pwm
% Pforce2esc = polyfit(thrust, esc, 2);
% figure; hold on;
% scatter(thrust, esc, 30, 'filled');

%  figure; hold on
%  plot(esc, rpm)
%  figure; hold on
%  plot(rpm,thrust)


% pwm to forces/torques

pesc2force = polyfit(esc, thrust, 2);
pesc2torque = polyfit(esc, torque, 2);

f_min = polyval(pesc2force, 1000)
f_max = polyval(pesc2force, 2000)

Fmin = 4*f_min
Fmax = 4*f_max
f_hover = 0.25*0.775*g;
a = pesc2force(1);
b = pesc2force(2);
c = pesc2force(3)-f_hover;
x1 = -b+sqrt(b*b-4*a*c)/(2*a)
x2 = -b-sqrt(b*b-4*a*c)/(2*a)

hvr_thr = (x1-1000)/1000

% 
p2 = polyfit(sqrt(sqrt(thrust)), esc, 2);
polyfit(sqrt(thrust), esc, 1);
polyval(p2, sqrt(sqrt(1.79)));
num2str(p2(1), 20)
num2str(p2(2), 20)
num2str(p2(3), 20)
p2(1)*sqrt(sqrt(1.79))^2+p2(2)*sqrt(1.79)+p2(3)

% % e2 = [];
% % for f_ = f
% %     a = pesc2force(1);
% %     b = pesc2force(2);
% %     c = pesc2force(3)-f_;
% %     x1 = -b+sqrt(b*b-4*a*c)/(2*a);
% %     x2 = -b-sqrt(b*b-4*a*c)/(2*a);
% %     e2 = [e2, 1000+x1];
% % %    e2 =  
% % end
%     
% 
figure; 
subplot(1,2,1);
hold on;
scatter(esc, thrust, 30, 'filled');
plot(e, polyval(pesc2force, e), 'r', 'linewidth', 1);
plot(polyval(p2,sqrt( sqrt(f))), f, 'b', 'linewidth', 1);
% plot(e2, f, 'k', 'linewidth', 1);
grid on; grid minor;
latex_title('$$f$$');
latex_ylabel('thrust [N]');
latex_xlabel('ESC $$\mu s$$');
latex_legend({'datum', 'trend'});
% num2str(pesc2force(1), 20)
% num2str(pesc2force(2), 20)
% num2str(pesc2force(3), 20)

% 
% subplot(1,2,2);
% hold on;
% scatter(esc, torque, 30, 'filled');
% plot(e, polyval(pesc2torque, e), 'r', 'linewidth', 1);
% grid on; grid minor;
% latex_title('$$\tau$$');
% latex_ylabel('torque [Nm]');
% latex_xlabel('ESC $$\mu s$$');
% latex_legend({'datum', 'trend'});
% % num2str(pesc2torque(1), 20)
% % num2str(pesc2torque(2), 20)
% % num2str(pesc2torque(3), 20)
% 
% 
% 
% 
% 
% 
% 
% 
