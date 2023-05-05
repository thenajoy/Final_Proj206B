

mQ = 2;
g = 9.80665;
J = diag([0.0483333 , 0.0483333 , 0.0833333]);

iJ = inv(J);
Js = J/min(eig(J));
dt = 1/500;

%% set environment
addpath('../trajectories/');
addpath('./controllers/');

ctrl = SO3VariationMPC();

ctrl.mQ = mQ;
ctrl.updateInertia(J);
ctrl.trajectory = @setpoint; % @circular_trajectory;
ctrl.flat2state = @qflat2state;

[A, B] = ctrl.variation_lin_dyn(1);
Ak = eye(6)+A*dt;
Bk = B*dt;

Q1 = diag([10, 10, 10, 0.1, 0.1, 0.1]);
Q2 = 0.1*eye(3);
[~, S, ~] = dlqr(Ak,Bk,Q1,Q2);

N = 10;
umin = ctrl.Mmin;
umax = ctrl.Mmax;

% cmd = ctrl.getCmdTraj(1);
% Rd = cmd.R;
% Omd = cmd.Omega;
% x
% 
% rotation:   0.981404 -0.0980301  -0.165036
%    0.13242   0.968178    0.21236
%   0.138967  -0.230265   0.963154
% angular velocity:  0.491356  0.713022 -0.344487
% xd
% 
% rotation: 1 0 0
% 0 1 0
% 0 0 1
% angular velocity: 0 0 0
% x0: -0.221313 -0.152002  0.115225  0.491356  0.713022 -0.344487
% uOpt        -2        -2         2        -2        -2         2  -1.48515        -2         2 -0.848278        -2   1.68385 -0.320592        -2   1.20418   0.18814        -2  0.783098  0.764908        -2  0.396393   1.50835        -2 0.0218331         2 -0.810672 -0.362113         2  0.715154 -0.777518
% input: torque: -2 -2  2

R = [0.981404 -0.0980301  -0.165036;
   0.13242   0.968178    0.21236;
  0.138967  -0.230265   0.963154];
Om = [0.491356  0.713022 -0.344487]';

Rd = eye(3);
Omd = zeros(3,1);

eR = errorR(R, Rd);
eOmega = errorOm(R, Om, Rd, Omd);

x0 = [eR; eOmega];
u0 = zeros(N*3,1);

[u, uOpt, uNext, fval, exitflag] = compute_mpc(N, x0, Ak, Bk, Q1, S, Q2, u0, umin, umax);

N = 5;
u0 = zeros(N*3,1);
[u, uOpt, uNext, fval, exitflag] = compute_mpc(N, x0, Ak, Bk, Q1, S, Q2, u0, umin, umax);


