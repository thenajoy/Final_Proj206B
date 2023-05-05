function [R, v, p, covar] = meas_update_mocap(R, v, p, Rm, vm, pm, covar)

H = [[eye(3), zeros(3,6)];[zeros(3,6), eye(3)]];
I = eye(9);
N =  blkdiag(0.001*eye(3), 0.01*eye(3));

% kalman gain
% -----------
S = H * covar * H' + N;
K = (covar * H')* inv(S);



% inv_state = [[R', -R'*v, -R'*p];[zeros(2,3), eye(2)]];
% meas= [[Rm, vm, pm];[zeros(2,3), eye(2)]];               
% eta = meas*inv_state;
eta = [[Rm, vm, pm];[zeros(2,3), eye(2)]]*[[R', -R'*v, -R'*p];[zeros(2,3), eye(2)]];

% performing logm
eta_log = matrix_log(eta);

error = [eta_log(3,2) ;
         eta_log(1,3) ;
         eta_log(2,1); % rotation
         eta_log(1:3,5)]; % position

delta = K * error;

hat_delta = [[[0 -delta(3) delta(2) ; delta(3) 0 -delta(1) ; -delta(2) delta(1) 0], delta(4:6), delta(7:9)];
                zeros(2,5)];
% performing matrix exponential
exp_hat_delta = matrix_exp(hat_delta);
            
new_state = exp_hat_delta*[[R,v,p];[zeros(2,3), eye(2)]];

R = new_state(1:3,1:3);
v = new_state(1:3,4);
p = new_state(1:3,5);

%Joseph update form
covar = (I - K * H) * covar * (I - K * H)' + K * N * K'; 

end


