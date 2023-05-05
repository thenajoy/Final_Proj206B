clear;

%%

dt = 1/100;
nx = 6;
nu = 3;

Ak = [eye(3), eye(3)*dt; zeros(3,3), eye(3)];
Bk = [zeros(3,3); eye(3)*dt];

N = 10;
A = zeros(nx,nx,N);
B = zeros(nx,nu,N);
for i = 1:N
    A(:,:,i) = Ak;
    B(:,:,i) = Bk;
end


gen_tracking_mpc(A, B)



