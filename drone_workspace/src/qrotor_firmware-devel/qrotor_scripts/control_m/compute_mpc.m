function [u, uOpt, uNext, fval, exitflag] = compute_mpc(N, x0, A, B, Q, P, R, u0, umin, umax)

[H, f, c] = generate_mpc_qp(N, x0, A, B, Q, P, R);
LB = repmat(umin, N, 1);
UB = repmat(umax, N, 1);
        

u = uOpt(1:3);

uNext = [uOpt(4:end,:); zeros(3,1)];

end