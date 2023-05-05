
J = [0.0049, 0.0000055, 0.0000054;
                   0.0000055, 0.0053, 0.000021;
                   0.0000054, 0.000021, 0.0098];

Omegad = zeros(3,1);
Rd = eye(3);
I = eye(3);
O = zeros(3,3);


A = [[-hat(Omegad), I];
        [O, pinv(J)*(hat(J*Omegad)-hat(Omegad)*J)]];
B = [O; pinv(J)];

Ts = 1/500;

Ak = eye(6)+A*Ts;
Bk = B*Ts;

[Ad, Bd] = c2d(A,B,Ts);

Q = blkdiag(100*eye(3), 10*eye(3));
R = eye(3);

[K, Pss, CLP] = dlqr(A, B, Q, R);
[K,S,E] = lqrd(A,B,Q,R,Ts);


