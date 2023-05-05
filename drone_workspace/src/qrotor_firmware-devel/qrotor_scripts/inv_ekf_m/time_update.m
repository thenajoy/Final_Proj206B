function [R, v, p, covar] = time_update(dt, R, v, p, covar, accel, gyro)
% predefined variables
gvec = [0;0;-9.81];
A = [zeros(3), zeros(3,3), zeros(3,3);
                hat(gvec), zeros(3,3), zeros(3,3);
                zeros(3,3), eye(3), zeros(3,3);];
gyro_covar = 10*eye(3);
accel_covar = 20*eye(3);
vel_covar = 1*eye(3);
Q = blkdiag(gyro_covar, accel_covar, vel_covar);

accel_bias = zeros(3,1);
gyro_bias = zeros(3,1);

w = gyro-gyro_bias;
a = accel-accel_bias;

% computing the rotation
eta = w*dt;
th = norm(eta);
K = hat(eta/th);
if (th < 1e-6)
    rot = eye(3);
else
    rot = eye(3)+ sin(th)*K + (1-cos(th))*K*K;
end

% net acceleration in inertial-frame
net_accel = R*a + gvec;

% time-propagating the IMU motion model
p = p + v*dt + 0.5*dt*dt*net_accel;
v = v + net_accel*dt;
R = R*rot;


% discretization of the linear dynamics
Ak = eye(9)+A*dt;

% covariance update
covar = Ak * covar * Ak'+ Q;
end


