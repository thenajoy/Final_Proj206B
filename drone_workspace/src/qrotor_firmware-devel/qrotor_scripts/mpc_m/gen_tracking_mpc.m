function [H, f, c] = gen_tracking_mpc(A, B, Q, P, R, Xd, Ud)
% 
% cost function 
% cost = 0.5*U'*H*U + f'*U + c
%
%%
nx = size(B,1);
nu = size(B,2);
N = size(B,3);

Sx = [];
Su = [];


sx = eye(nx);

Sx = zeros(N*nx,nx);
Su = zeros(N*nx, N*nu);
for i = 1 : N
    sx = A(:,:,i)*sx;
    Sx(nx*(i-1)+1:nx*i,:) = sx;
end



end