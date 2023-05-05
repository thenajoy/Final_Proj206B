

R = Rxd(5)*Ryd(4);
v = zeros(3,1);
p = zeros(3,1);

Rm= eye(3);
vm = 0.01*ones(3,1);
pm = 0.01*ones(3,1);

covar = 0.1*eye(9);

[new_R, new_v, new_p, covar] = meas_update_tcam(R, v, p, Rm, vm, pm, covar);


