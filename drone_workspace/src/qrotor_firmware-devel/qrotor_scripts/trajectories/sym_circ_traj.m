clear
close all

%% circular trajectory
% syms t real
% syms yaw yaw_rate real
% p0  = sym('p0',     [3,1], 'real');
% r   = sym('r',      [3,1], 'real');
% phi = sym('phi',    [3,1],'real');
% w   = sym('w',      [3,1], 'real');
% 
% p  = [p0(1) + r(1) .* cos(phi(1) + w(1) .* t); ...
%             p0(2) + r(2) .* sin(phi(2) + w(2) .* t); ...
%             p0(3) + r(3) .* sin(phi(3) + w(3) .* t)];
% v  = diff(p,t);
% a  = diff(v,t);
% da = diff(a,t);
% d2a = diff(da,t);
% 
% psi = yaw + yaw_rate*t;
% b1 = [cos(psi); sin(psi); 0];
% db1 = diff(b1,t);
% d2b1 = diff(db1,t);
% 
% traj.p = p;
% traj.v = v;
% traj.a = a;
% traj.da = da;
% traj.d2a = d2a;
% traj.b1 = b1;
% traj.db1 = db1;
% traj.d2b1 = d2b1;
% 
% matlabFunction(p,v,a,da,d2a,b1,db1,d2b1, 'File','circ_traj', ...
%                 'Vars', {t, p0, r, phi, w, yaw, yaw_rate});


%%
clear
t = 0.1;
p0 = ones(3,1);
r = [1;1;0];
phi = zeros(3,1);
w = 0.1*ones(3,1);
yaw = deg2rad(20)
yaw_rate = 0.1

[traj] = circ_traj(t, p0, r, phi, w, yaw, yaw_rate);

%% 
% [x0+r*np.cos(2*a*(b+np.exp(1)**((-1)*c*(t+(-1)*t0)))**(-1)*np.pi),
% y0+r*np.sin(2*a*(b+np.exp(1)**((-1)*c*(t+(-1)*t0)))**(-1)*np.pi),
% z0])

