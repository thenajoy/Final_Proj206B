
t = 0.1;
p0 = ones(3,1);
r = [1;1;0];
phi = zeros(3,1);
w = 0.1*ones(3,1);
yaw = deg2rad(20)
yaw_rate = 0.1

[traj] = circ_traj(t, p0, r, phi, w, yaw, yaw_rate);

JQ  =  [0.0023, 1e-7, 1e-7;
        1e-7, 0.0023, 1e-7;
        1e-7, 1e-7, 0.0076];
mQ = 0.85;

[state] = qflat2state(mQ, JQ, traj)

