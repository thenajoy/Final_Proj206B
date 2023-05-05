function [xOpt, fval, exitflag] = qp(H, f, A, b, LB, UB, x0)

options = optimoptions('quadprog', ...
    'Display', 'off', ...
    'Algorithm', 'active-set');
% ws = optimwarmstart(x0,options);
[xOpt, fval, exitflag] = quadprog(H, f, A, b, [], [], LB, UB, x0, options);

% xOpt = x.X;
end
