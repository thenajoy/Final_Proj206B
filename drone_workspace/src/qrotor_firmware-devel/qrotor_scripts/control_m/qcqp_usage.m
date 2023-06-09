clear;
%%
% Q = diag([1, 1, 1, 4e2]);
% f = zeros(4, 1);
% c = 0;
% LB = [-2;-2;-2;-1e9];
% UB = -LB;
% 
% % H = [	4.0000655e-06	4.4898698e-09	4.4082356e-09	       0;
% % 	4.4898698e-09	4.3266014e-06	1.7143137e-08	       0;
% % 	4.4082356e-09	1.7143137e-08	8.0001309e-06	0;
% %     0 0 0 0];
% % 
% % k = [-6.5548273e-05;
% % 	-1.0184672e-05;
% % 	-9.7483193e-05;
% % 	      -1];
% % 
% % 
% % d = 0.001167;
% x0 = zeros(4,1);
% 
% R = Rxd(5);
% Om = 0.1*ones(3,1);
% Rd = eye(3);
% Omd = zeros(3,1);
% [H, k, d] = quadratic_dclf_const(1/500, R, Om, Rd, Omd);
% 
% 
% [xOpt, fval, eflag, output, lambda]= qcqp(Q, f, c, H, k, d, LB, UB, x0);

% lb = -ones(4,1);
% ub = ones(4,1);
% 
% A = [1,1,1,-1];
% b = -1;
% % [xOpt2, x2, fval2, exitflag] = qp(Q, f, A, b, lb, ub, x0);

%%
% config = coder.config()
% codegen -c -lang:c++ -d codegen qp -args {Q, f, A, b, lb, ub, x0}
% 
dt = 0.002;
R = [ [ 0.945993, -0.177685, 0.271154];
[ 0.023448, 0.871727, 0.489430];
[ -0.323337, -0.456640, 0.828814]];

 
Rc= [ [ 0.960686, 0.000000, -0.277638];
[ 0.078093, 0.959626, 0.270220];
[ 0.266429, -0.281278, 0.921899]];

Om = [-1.326504, -1.960635, -0.657170]';
Omc = [0.000000, 0.000000, 0.000000]';

inertia= [0.004900, 0.000006, 0.000005;
    0.000006, 0.005300, 0.000021;
    0.000005, 0.000021, 0.009800];
inertia_inv=[204.081955, -0.211339, -0.112001;
    -0.211340, 188.681046, -0.404200;
    -0.112001, -0.404201, 102.041748];
inertia_scaled=[1.000016, 0.001122, 0.001102;
    0.001122, 1.081650, 0.004286;
    0.001102, 0.004286, 2.000033];

gamma = 0.9; 
epsilon2 = 8; % original value: 4
c2 = 30;
    
[H, k, d, V] = quadratic_dclf_const(dt, R, Om, Rc, Omc, gamma, c2, epsilon2,  inertia, inertia_inv, inertia_scaled)
% d =-0.373574
Q = diag([1, 1, 1, 4e2]);
f = zeros(4, 1);
c = 0;
LB = [-2;-2;-2;-1e9];
UB = -LB;
x0 = zeros(4,1);
[xOpt, fval, eflag, output, lambda]= qcqp(Q, f, c, H, k, d, LB, UB, x0)

% ---
% H_array[16] = 
% [ 0.166600, -0.000173, -0.000091, 0.000000]
% [ -0.000173, 0.154028, -0.000330, 0.000000]
% [ -0.000091, -0.000330, 0.083301, 0.000000]
% [ 0.000000, 0.000000, 0.000000, 0.000000]
% k = [-0.364417, -0.387497, -0.086901, -1.000000]
% d =6.273497
% ***************************************
% time: 0.000264
% x[0] = 2.000000, x[1] = 2.000000, x[2] = 1.000000, x[3] = 5.364141, 
% CLFQP:mx: 2.000000	 my: 2.000000	 mz: 1.000000 f 8.816418
% 
% 
% 

%%
% F: -3.438886, 3.346997, 11.418840
% e3_cmd: -0.277638, 0.270220, 0.921899
% *************** :QCQP: ***************
% R[9] = 
% [ 0.945993, -0.177685, 0.271154]
% [ 0.023448, 0.871727, 0.489430]
% [ -0.323337, -0.456640, 0.828814]
% Rc[9] = 
% [ 0.960686, 0.000000, -0.277638]
% [ 0.078093, 0.959626, 0.270220]
% [ 0.266429, -0.281278, 0.921899]
% Om.T = [ -1.326504, -1.960635, -0.657170]
% Omc.T = [0.000000, 0.000000, 0.000000]
% H_array[16] = 
% [ 0.166600, -0.000173, -0.000091, 0.000000]
% [ -0.000173, 0.154028, -0.000330, 0.000000]
% [ -0.000091, -0.000330, 0.083301, 0.000000]
% [ 0.000000, 0.000000, 0.000000, 0.000000]
% k = [-1.161100, 0.810057, 0.004229, -1.000000]
% d =-0.311076
% ***************************************
% eflag: 1.000000, comp_time: 0.000152
% x[0] = 0.000000, x[1] = 0.000000, x[2] = 0.000000, x[3] = 0.000000, 
% 
