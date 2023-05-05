% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(x, H) + g'*x)
%   subject to
%     lbA <= A*x
%     A*x <= ubA
%     lb <= x
%     x <= ub
%
% with variables
%        x   4 x 1
%
% and parameters
%        A   1 x 4
%        H   4 x 4    PSD
%        g   4 x 1
%       lb   4 x 1
%      lbA   1 x 1
%       ub   4 x 1
%      ubA   1 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.A, ..., params.ubA, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2021-01-20 17:22:48 -0500.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
