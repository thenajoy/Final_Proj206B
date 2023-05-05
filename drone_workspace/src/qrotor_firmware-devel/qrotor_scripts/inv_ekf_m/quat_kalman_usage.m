

R = Rxd(5)*Ryd(4);
Rm= eye(3);

[q2] = quat_kalman_update(rotm2quat(R), rotm2quat(Rm));


