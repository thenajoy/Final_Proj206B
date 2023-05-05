clear
load('pwm.mat');

alpha = 0.3;

N = length(tLog);
pold = 1000*ones(1,4);

pwm_f = zeros(N,4);
for i = 1:N
    pwm_f(i,:) = (1-alpha)*pold + alpha*PWM(i,:);
    pold = pwm_f(i,:);
end

%%
close all
figure; hold on;
plot(tLog, PWM(:,1), 'r');
plot(tLog, pwm_f(:,1), 'b');

