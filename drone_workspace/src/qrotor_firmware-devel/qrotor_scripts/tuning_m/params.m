
%% dynamics parameters
g = 9.80665;
e1 = [1;0;0];
e2 = [0;1;0];
e3 = [0;0;1];

m = 0.775;
J = [0.005315307431627, 0.000005567447099, 0.000005445855427;
0.000005567447099, 0.004949258422243, 0.000020951458431;
0.000005445855427, 0.000020951458431, 0.009806225007686;];

invJ = [188.1362   -0.2112   -0.1040;
   -0.2112  202.0525   -0.4316;
   -0.1040   -0.4316  101.9770];

tau = 0.002;

%% inner-loop parameters
N = 4;
th_ = [pi/4, (5*pi/4), (3*pi/4), (7*pi/4)];
d_ = [-1, -1, 1, 1];
mass_ = 0.775;
l_ = 0.168275;

kf_ = 7.64e-06;    % //6.656300353404825e-06;
cf_ = 0;            % //-0.057668040811899;
km_ = 1.026e-07;    % //1.026148112683309e-07;
cm_ = 7.68e-04;       % //0; //7.688264352189356e-04;

pwm2w  =   1166.3755960945295556;
pwm2w_offset =   -1156.5120044647383111;
w2pwm  =   0.0008514439377261;
w2pwm_offset = 0.9946035424300900;

% allocation matrix
A = zeros(4,4);
A_offset = zeros(4,1);
for i = 1:N
    
    A(1,i) =  kf_;
    A(2,i) =  l_*sin(th_(i))*kf_;
    A(3,i) =  -l_*cos(th_(i))*kf_;
    A(4,i) =  d_(i)*km_;
   
    A_offset = A_offset+[cf_, l_*sin(th_(i))*cf_, -l_*cos(th_(i))*cf_, d_(i)*cm_]';
end

invA = pinv(A);

PWM_MIN = 1.1;
PWM_MAX = 1.9;

rotor_MIN = pwm2w*PWM_MIN + pwm2w_offset;
rotor_MAX = pwm2w*PWM_MAX + pwm2w_offset;

tmp_f_min = A*(rotor_MIN^2*ones(4,1))+A_offset;
tmp_f_max = A*(rotor_MAX^2*ones(4,1))+A_offset;

tmp_mx_min = A*[rotor_MIN; rotor_MAX; rotor_MIN; rotor_MAX].^2+A_offset;
tmp_mx_max = A*[rotor_MAX; rotor_MIN; rotor_MAX; rotor_MIN].^2+A_offset;

tmp_my_min = A*[rotor_MAX; rotor_MIN; rotor_MIN; rotor_MAX].^2+A_offset;
tmp_my_max = A*[rotor_MIN; rotor_MAX; rotor_MAX; rotor_MIN].^2+A_offset;

tmp_mz_max = A*[rotor_MIN; rotor_MIN; rotor_MAX; rotor_MAX].^2+A_offset;
tmp_mz_min = A*[rotor_MAX; rotor_MAX; rotor_MIN; rotor_MIN].^2+A_offset;


%% input bounds
F_MIN = tmp_f_min(1);
F_MAX = tmp_f_max(1);

M_MIN = [tmp_mx_min(2); tmp_my_min(3); tmp_mz_min(4)];
M_MAX = [tmp_mx_max(2); tmp_my_max(3); tmp_mz_max(4)];

%% control parameters

% position
% --------
% decently working
% kp_pos = [2; 2; 6];
% kd_pos = [4; 4; 4];
% ki_pos = [0.0; 0.0; 0.0];
% 
% kp_pos = [4; 4; 6];
% kd_pos = [5; 5; 4];
% ki_pos = [0.0; 0.0; 0.0];

kp_pos = 1*[1,1,5]';
kd_pos = 5*[0.5, 0.5, 0.5]';
ki_pos = 0*[1,1,2]';


% attitude
%---------
% decently working
% kp_att = [0.52; 0.5; 0.24];
% kd_att = [0.22; 0.18; 0.18];
% ki_att = [0.0; 0.0; 0.0;];

% kp_att = [0.52; 0.5; 0.24];
% kd_att = [0.22; 0.18; 0.18];
% ki_att = [0.0; 0.0; 0.0;];

kp_att = 1.8*[0.7, 0.7, 0.25*0.7943]';
kd_att = 1.5*[0.12, 0.12, 0.15]';
ki_att = 0*[1,1,2]';

%% Initial condition
% x0  = [-0.2; -0.2; -0.2];
x0  = [-0.5; -0.5; -0.5];


% R0 = RPY2Rot_ZXY((pi/180)*[60;60;90]);
R0 = RPY2Rot_ZXY((pi/180)*[0;0;0]);

xInit = [x0;
            zeros(3,1);
            reshape(R0,9,1);
            zeros(3,1);];


        