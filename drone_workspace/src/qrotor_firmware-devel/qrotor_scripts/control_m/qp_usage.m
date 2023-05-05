Q = diag([1,1,1,4e2]);
f = zeros(4,1);

A = [1,1,1,-1];
b = -1;

lb = -ones(4,1);
ub = ones(4,1);

x0 = zeros(4,1);

[xOpt, x, fval, exitflag] = qp(Q, f, A, b, lb, ub, x0);

