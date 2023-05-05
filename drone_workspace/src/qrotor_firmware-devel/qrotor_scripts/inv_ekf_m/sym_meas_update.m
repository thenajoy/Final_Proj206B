<<<<<<< HEAD
% sym_meas_update

% measurement matrix
H = eye(9);
% meas covar
n = sym('n', [9,1], 'real');
N = diag(n);
% state covar
P = sym('P', [9,9], 'real');
% state variables
R = sym('R', [3,3], 'real');
v = sym('v', [3,1], 'real');
p = sym('p', [3,1], 'real');

Rm = sym('Rm', [3,3], 'real');
vm = sym('vm', [3,1], 'real');
pm = sym('pm', [3,1], 'real');

% S = H*P*H'+N;
% K = P*H'*inv(S);
% 
% save('sym_meas_update.mat');
t
% inv_state = [[R', -R'*v, -R'*p];
%                     [zeros(2,3), eye(2)]];
%                 
% eta = meas*inv_state;
% 
% % performing logm
% eta_log = matrix_log(eta);
% 
% error = [eta_log(3,2) ;
%          eta_log(1,3) ;
%          eta_log(2,1);
%          eta_log(1:3,4);
%          eta_log(1:3,5)];
% 
% delta = K * error;
% 
% hat_delta = [[[0 -delta(3) delta(2) ; delta(3) 0 -delta(1) ; -delta(2) delta(1) 0], delta(4:6), delta(7:9)];
%                 zeros(2,5)];
% % performing matrix exponential
% exp_hat_delta = matrix_exp(hat_delta);
%             

cd % %Joseph update form
% new_covar = (I - K * H) * covar * (I - K * H)' + K * N * K'; 



=======
%
% 

xi_R = sym('xi_R', [3,1],'real');
xi_v = sym('xi_v', [3,1],'real');
xi_x = sym('xi_x', [3,1],'real');


Lg_xi = [[hat(xi_R), xi_v, xi_x]; zeros(2,5)]
matrix_exp(Lg_xi)
>>>>>>> 1380104d74f8317d804736cdf194d830e5c9151d


