% sym_time_update

syms dt g th real
gc = sym('gc',[3,1],'real');
ac = sym('ac',[3,1],'real');
vc = sym('vc',[3,1],'real');

% state variables
R = sym('R', [3,3], 'real');
v = sym('v', [3,1], 'real');
p = sym('p', [3,1], 'real');

% imu variables
w = sym('w',[3,1], 'real');
a = sym('a',[3,1], 'real');

% covar 
P = sym('P', [9,9], 'real');

gyro_covar = diag(gc);
accel_covar = diag(ac);
vel_covar = diag(vc);
Q= blkdiag(gyro_covar, accel_covar, vel_covar);

gvec = [0;0;g];
A = [zeros(3), zeros(3,3), zeros(3,3);
        hat(gvec), zeros(3,3), zeros(3,3);
        zeros(3,3), eye(3), zeros(3,3);];
            
% computing rotation
eta = w*dt;
K = hat(eta/th);
rot = simplify( eye(3)+ sin(th)*K + (1-cos(th))*K*K);

% net acceleration in inertial-frame
net_accel = R*a + gvec;

% time-propagating the IMU motion model
new_p = p + v*dt + 0.5*dt*dt*net_accel;
new_v = v + net_accel*dt;
new_R = R*rot;


% discretization of the linear dynamics
Ak = eye(9)+A*dt;

% covariance update
new_P = simplify(Ak * P * Ak'+ Q);

vars_ = {dt, R, v, p, a, w, P, gc, ac, vc, g, th};

matlabFunction(new_R, new_v, new_p, new_P, 'File','timeUpdate', ...
    'Vars', vars_);








