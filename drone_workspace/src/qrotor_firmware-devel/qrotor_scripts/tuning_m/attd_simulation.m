clear; close all;

J = [0.0049, 0.0000055, 0.0000054; ...
    0.0000055, 0.0053, 0.000021; ...
    0.0000054, 0.000021, 0.0098];
iJ = pinv(J);

Omegad = zeros(3, 1);
Rd = eye(3);
I = eye(3);
O = zeros(3, 3);

%%
A = [[-hat(Omegad), I]; ...
    [O, pinv(J) * (hat(J * Omegad) - hat(Omegad) * J)]];
B = [O; pinv(J)];

Ts = 1 / 500;

Ak = eye(6) + A * Ts;
Bk = B * Ts;

[Ad, Bd] = c2d(A, B, Ts);

Q = blkdiag(diag([500, 500, 10]), diag([20, 20, 5]));
R = 1*eye(3);

% [K, Pss, CLP] = dlqr(A, B, Q, R);
[K, S, E] = lqrd(A, B, Q, R, Ts);
fprintf('{%f, %f, %f,\n%f, %f, %f,\n%f, %f, %f}\n\n',...
    K(1,1), K(1,2), K(1,3), ... 
    K(2,1), K(2,2), K(2,3), ...
    K(3,1), K(3,2), K(3,3));
fprintf('{%f, %f, %f,\n%f, %f, %f,\n%f, %f, %f}\n\n',...
    K(1,4), K(1,5), K(1,6), ... 
    K(2,4), K(2,5), K(2,6), ...
    K(3,4), K(3,5), K(3,6));

%% simulation

params.J = J;
params.iJ = iJ;

R0 = eul2rotm([pi / 3, pi / 4, pi / 2]);
Om0 = zeros(3, 1);
x0 = [reshape(R0, 9, 1); Om0];

N = 1000;
x = zeros(12,N);
u = zeros(3,N);
t = zeros(1,N);
error = zeros(6,N);

x(:,1) =x0;
for i = 2:N
    
    R = reshape(x(1:9,i-1),3,3);
    Om = x(10:12,i-1);
    eR = 0.5*vee(Rd'*R - R'*Rd);
    eOm = Om-R'*Rd*Omegad;
    params.u = -K*[eR; eOm];
    params.u = max(min([2;2;2], params.u),[-2;-2;-2]);
    
    
    sol = ode15s(@att_dyn, [t(i-1), t(i-1)+Ts], x(:,i-1), [], params);
    
    x(:,i) = sol.y(:,end);
    u(:,i-1) = params.u;
    t(i) = t(i-1)+Ts;
    error(:,i-1) = [eR; eOm];
end
R = reshape(x(1:9,end),3,3);
Om = x(10:12,end);
error(:,end) = [[0.5*vee(Rd'*R - R'*Rd)]; [Om-R'*Rd*Omegad]];


%%
% close all;
% figure;
subplot(2,2,1); hold on;
plot(t, error(1,:), 'r', 'linewidth', 1);
plot(t, error(2,:), 'g', 'linewidth', 1);
plot(t, error(3,:), 'b', 'linewidth', 1);

subplot(2,2,2); hold on
plot(t, error(4,:), 'r', 'linewidth', 1);
plot(t, error(5,:), 'g', 'linewidth', 1);
plot(t, error(6,:), 'b', 'linewidth', 1);


subplot(2,2,3); hold on
plot(t(1:end-1), u(1,1:end-1), 'r', 'linewidth', 1);
plot(t(1:end-1), u(2,1:end-1), 'g', 'linewidth', 1);
plot(t(1:end-1), u(3,1:end-1), 'b', 'linewidth', 1);

%%
function [x] = att_dyn(t, x, params)

R = reshape(x(1:9), 3, 3);
Om = x(10:12);

u = params.u;
dR = R * hat(Om);
dOm = params.iJ * (u - cross(Om, params.J * Om));

x = [dR(:); dOm];
end
