function [u, delta, V, LgV, LfV] = SO3_DiscreteClf_QCQP(dt, R, Om, Rc, Omc, Mff)
% parameters
% ----------


% cost function
% -------------


% constraint
% ----------
[H, k, d] = quadratic_dclf_const(dt, R, Om, Rc, Omc);

% bounds
% ------
LB = [Mff+Mmin; -inf];
UB = [Mff+Mmax; inf];

end