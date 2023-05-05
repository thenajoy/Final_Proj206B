

dt = 0.02;
R = eye(3);
v = zeros(3,1);
p = zeros(3,1);
covar = 0.01*eye(9);
accel = [0;0;9.81];
gyro = [0.1; 0.3; -2];

[R, v, p, new_covar] = time_update(dt, R, v, p, covar, accel, gyro);


