clear;
avg_pwm_in_us = 1450;

pwm2hz(avg_pwm_in_us)
% pwm2hz(1200)
% pwm2hz(1100)



function [hz] = pwm2hz(pwm)
w2pwm_k =  0.8514439377261; % rad/s to micro-second
w2pwm_c =  994.6035424300900; % micro-second
% pwm_us = w2pwm_k*omega + w2pwm_c

avg_prop_speed = (pwm-w2pwm_c)/w2pwm_k; % rad/s
hz = avg_prop_speed/(2*pi);

end



