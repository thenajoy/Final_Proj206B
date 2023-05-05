function traj = setpoint(t)

traj.p = zeros(3,1);
traj.v =  zeros(3,1);
traj.a =  zeros(3,1);
traj.da = zeros(3,1);
traj.d2a =  zeros(3,1);

traj.b1 = [1;0; 0.0];
traj.db1 =  zeros(3,1);
traj.d2b1 =  zeros(3,1);

end