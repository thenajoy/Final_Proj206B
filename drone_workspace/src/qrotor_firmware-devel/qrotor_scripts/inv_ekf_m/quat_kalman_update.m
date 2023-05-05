function [q_update] = quat_kalman_update(q, q_meas)
% gains
c_q = 0.1;
c_qm = 0.01;

H = eye(3);
P = c_q*eye(3);playin gin Chen
N = c_qm*eye(3);


%%
R = quat2rotm(q);
Rmeas = quat2rotm(q_meas);

Rerror = Rmeas*R';

%% converting Rerror to angle-axis
th = acos(max(min(((trace(Rerror)-1)/2),1),-1));
if th < 1e-6
    w = [0;0;1];
else
    w = -1/(2*sin(th))*[R(3,2)-R(2,3);R(1,3)-R(3,1); R(2,1)-R(1,2)];
end
eta = w*th;

%% Weighted update (bad Kalman update)
K = (c_q/(c_q+c_qm))*eye(3);

%% correction
delta = K*eta;

th2 = norm(delta);
if th2< 1e-6
    w2 = [0;0;1];
else
    w2 = delta/th2;
end
K = hat(w2);
R_correction = eye(3)+sin(th2)*K+(1-cos(th2))*K*K;

%% final udpate
R_hat = R_correction*R;
q_update = rotm2quat(R_hat);

end