function flats = circular_trajectory(t)

p0 = ones(3,1);
r = [1;1;0];
phi = zeros(3,1);
w = pi*ones(3,1);
yaw = 0;
yaw_rate = 0.1;

flats = circ_traj(t, p0, r, phi, w, yaw, yaw_rate);

end