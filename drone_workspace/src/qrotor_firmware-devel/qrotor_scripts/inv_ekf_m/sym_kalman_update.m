
syms k1 k2 real
H = eye(3);
P = k1*eye(3);
N = k2*eye(3);

S = H * P * H' + N;
K = (P * H')* inv(S);


