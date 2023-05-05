function [H, f, c] = generate_mpc_qp(N, x0, A, B, Q, P, R)

%%
% cost:  0.5*x'*H*x + f'*x + c
% constraints: Ain*x <= bin
%               Ain = G0
%               bin = w0 + E0*x0

%% cost
nx = size(B, 1);
nu = size(B, 2);

Sx = zeros((N+1)*nx, nx);
Su = zeros((N+1)*nx, N*nu);

Sx(1:nx, :) = eye(nx);
for i = 1:N
    Sx(nx*i+1:nx*(i + 1), :) = A * Sx(nx*(i - 1)+1:nx*i, :);
end
for i = 1:N
    if (i == 1)
        Su(nx*i+1:nx*(i + 1), 1:nu) = B;
    else
        tmp1 = A * Su(nx*(i - 1)+1:nx*i, 1:nu);
        tmp2 = Su(nx*(i - 1)+1:nx*(i), 1:nu*(i - 1));
        Su(nx*i+1:nx*(i + 1), 1:nu*i) = [tmp1, tmp2];
    end
end

% state cost weights
Qbar = blkdiag(kron(eye(N), Q), P);
% input cost weights
Rbar = blkdiag(kron(eye(N), R));


% Cost function
H = Su' * Qbar * Su + Rbar;
H = (H+H')*0.5;
F = Sx' * Qbar * Su;
f = (2 * x0' * F)';
c = x0' * Sx' * Qbar * Sx * x0;
end