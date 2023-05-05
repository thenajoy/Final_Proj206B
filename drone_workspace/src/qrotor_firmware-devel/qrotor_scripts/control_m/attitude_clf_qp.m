function attitude_clf_qp()
% close all


%%

obj.m = 1;
obj.g = 9.81;
obj.e1 = [1;0;0];
obj.e3 = [0;0;1];
obj.Mmax = [1.3149; 1.3149; 0.1704];
obj.Mmin = -obj.Mmax;
obj.J =  [0.0023, 1e-7, 1e-7;
        1e-7, 0.0023, 1e-7;
        1e-7, 1e-7, 0.0076];
obj.iJ = inv(obj.J);
obj.Js = obj.J/min(eig(obj.J));

obj.controller = [];
obj.FREQ = 500;

obj = computeGains(obj);
obj = computeGains2(obj);
obj.x0 = zeros(4,1);

%
R0 = RPYtoRot_ZXY(20*pi/180, 20*pi/180, 0*pi/180);
y0 = [reshape(R0,9,1); zeros(3,1)];

tf = 15;
% 
% tic
% obj.controller = @pd_ctrl;
% log1 = dsimulate([0,tf], y0, obj);
% toc

sim_strart = tic;
% obj.controller = @pd_ctrl;
% obj.controller = @clf_qp;
obj.controller = @dclf_qp;
% obj.controller = @variation_clf_qp;
% obj.controller = @variation_dclf_qp; 
log2 = dsimulate([0,tf], y0, obj);
sim_end = toc(sim_strart);

fprintf("simulation time %f\n", sim_end);

% tic
% obj.controller = @pd_ctrl;
% log3 = csimulate([0,tf], y0, obj);
% toc
% 
% tic
% obj.controller = @clf_qp;
% log4 = csimulate([0,tf], y0, obj);
% toc

% keyboard;
%%
close all
% plot_log(obj,log1);
plot_log(obj, log2);

figure('name', strcat('lyap-',func2str(obj.controller))); hold on;
% plot(log1.t, log1.V, 'linewidth', 2);
plot(log2.t, log2.V, 'linewidth', 2)
% plot(log3.t, log3.V, 'linewidth', 2);
% plot(log4.t, log4.V, 'linewidth', 2)
% 
% plot(log2.t, log2.LfV, 'linewidth', 2);
% for i = 1:3
%     plot(log2.t, log2.LgV(i,:), 'linewidth', 1);
% end

grid on; grid minor;
% latex_legend({'pd-disc', 'clf-disc', 'clf-cont'},20);
% xlim([0,1])
% latex_title('$$clf-vs-pd~@1000Hz$$');


keyboard
end

function plot_log(obj, log)

%     xlim_ = [0, 10];d
    % plots
    % -----
    L = length(log.t);
    figure('name', strcat('input-',func2str(obj.controller))); 
    hold on;
    for it = 1:3
        subplot(3,1,it);
        hold on;
        stairs(log.t, log.u(it,:), 'linewidth', 2);
        l = plot(log.t, obj.Mmin(it)*ones(1, L),'k','linewidth',2); l.Color(4)=0.4;
        l = plot(log.t, obj.Mmax(it)*ones(1, L),'k','linewidth',2); l.Color(4)=0.4;

        grid on; grid minor;
%         xlim(xlim_);
        latex_title(strcat('$$f_',num2str(it),'$$'));
    end

    figure('name', strcat('states-',func2str(obj.controller))); 
    for it = 1:3
        subplot(3,2,it*2-1);
        hold on;
        stairs(log.t, rad2deg(log.RPY(it,:)), 'linewidth', 2);
        grid on; grid minor;
        latex_title(strcat('$$p_',num2str(it),'$$'));
%         xlim(xlim_);


        subplot(3,2,it*2);
        hold on;
        stairs(log.t, log.x(it+9,:), 'linewidth', 2);
        grid on; grid minor;
        latex_title(strcat('$$v_',num2str(it),'$$'));
%         xlim(xlim_);
    end

end


%% dynamics
function [dx] = ode_fun(t,x,obj)
    R   = reshape(x(1:9),3,3);
    Om  = x(10:12);
    dR  = R*hat(Om);
    dOm = obj.iJ*(obj.controller(t,x, obj)-hat(Om)*obj.J*Om);
    dx = [reshape(dR,9,1);
            dOm];
end

function [dx] = ode_fun1(t,x,obj)
    R   = reshape(x(1:9),3,3);
    Om  = x(10:12);
    dR  = R*hat(Om);
    dOm = obj.iJ*(obj.u-hat(Om)*obj.J*Om);
    dx = [reshape(dR,9,1);
            dOm];
end

function [log] = csimulate(tspan,x0,obj)
    fprintf("running continuous simulation of: %s\n",func2str(obj.controller));

    options = odeset('RelTol', 1e-4, 'AbsTol', 1e-8);
    sol = ode15s(@ode_fun,tspan,x0,options, obj);
    
    log.t = sol.x;
    log.x = sol.y;
    L = size(log.t,2);
    log.u = zeros(3,L);
    log.RPY = zeros(3,L);
    for i = 1:L
        log.RPY(:,i) = Rot2RPY_ZXY(reshape(log.x(1:9,i),3,3));
        [u, delta, V, LgV, LfV] = obj.controller(log.t(i), log.x(:,i), obj);
        log.u(:,i) = u;
        log.V(:,i) = V;
        log.LfV(:,i) = LfV;
        log.LgV(:,i) = LgV';
        log.delta(:,i) = delta;
    end
end

function [log] = dsimulate(tspan,x0,obj)
    fprintf("running discrete simulation of: %s\n",func2str(obj.controller));
    t = tspan(1):1/obj.FREQ:tspan(2);
    N = length(t);
% 	obj.options= odeset('MaxStep', 1/obj.FREQ);
    obj.options = odeset();

    log.t = t;
    log.u = zeros(3,N);
    log.x = zeros(12,N);
    log.RPY = zeros(3,N);
    log.V = zeros(1,N);
    log.LfV = zeros(1,N);
    log.delta = zeros(1,N);
    log.LgV = zeros(3,N);
    log.comp_time = zeros(1,N);
    
    % initial condition
    log.x(:,1) = x0;
    log.RPY(:,1) = Rot2RPY_ZXY(reshape(x0(1:9),3,3));

    progressbar
    for k = 2:N
        tic
        [obj.u, delta, V, LgV, LfV] = obj.controller(t(k-1), x0, obj);
        log.comp_time(k) = double(toc);
        % continuous integration
        % ----------------------
        % obj.options = odeset('RelTol', 1e-4, 'AbsTol', 1e-8, 'MaxStep', 1/obj.FREQ);
        sol = ode15s(@ode_fun1,[t(k-1), t(k)], x0, obj.options, obj);
        % obj.options = odeset(obj.options, 'InitialStep', sol.x(end)-sol.x(end-4));

        % discrete integration
%         %---------------------
%         R = reshape(x0(1:9),3,3);
%         Omega = x0(10:12);
%         Rnew = R*expm(hat(Omega)*1/obj.FREQ);
%         dOmega = obj.iJ*(obj.u-hat(Omega)*obj.J*Omega);
%         Omega_new = Omega + dOmega*1/obj.FREQ;
%         sol.y = [reshape(Rnew, 9,1); Omega_new];

        
        % logging
        % -------
        x0 = sol.y(:,end);
        obj.x0 = [obj.u; delta];
        % fprintf("%.4f\n",t(k));

        log.u(:,k-1) = obj.u;
        log.V(:,k-1) = V;
        log.LfV(:,k-1) = LfV;
        log.LgV(:,k-1) = LgV';
        log.delta(:,k-1) = delta;

        log.x(:,k) = x0;
        log.RPY(:,k) = Rot2RPY_ZXY(reshape(x0(1:9),3,3));

        progressbar(k/N);
    end
	[u, delta, V, LgV, LfV] = obj.controller(t(k), x0, obj);
    log.u(:,k) = u;
    log.V(:,k) = V;
    log.LfV(:,k) = LfV;
    log.LgV(:,k) = LgV';
    log.delta(:,k) = delta;

end

%% controllers

function [u, delta, V, LgV, LfV] = pd_ctrl(t,x, obj)
    trajd = trajectory(obj, t);
    
    R = reshape(x(1:9),3,3);
    Om = x(10:12);
    Om_hat = hat(Om);
    
	% compute SO(3) error
    eR = 0.5*vee(trajd.R'*R-R'*trajd.R);
    eOmega = Om-R'*trajd.R*trajd.Om;
    dR = R * Om_hat;
    dRc = trajd.R * hat(trajd.Om);
    deR = (vee(dRc' * R - R' * dRc) + vee(trajd.R' * dR - dR' * trajd.R)) / 2;

    eta2 = 150;
    J_scale = obj.J / min(eig(obj.J));
    epsilon2 = 4;
    c2 = 20;
    
    % lyapunov functions
    V = eOmega' * J_scale * eOmega / 2 + epsilon2 * (eR' * eOmega) ...
        +c2 * (eR' * eR) / 2;
    LgV = (eOmega' * J_scale + epsilon2 * eR');
    LfV = (epsilon2 * eOmega' + c2 * eR') * deR ...
        -LgV * (dR' * trajd.R * trajd.Om + R' * trajd.R * trajd.dOm);
    delta = 0;

    % compute the moment and thrust
    kR = diag([10,10,10]);
    kOmega = diag([1,1,1]);
    
    Omega_hat = hat(Om);
    Mfb = (-kR * eR - kOmega * eOmega);
    Mff = Omega_hat * obj.J * Om ...
        +obj.J * (R' * trajd.R * trajd.dOm - Omega_hat * R' * trajd.R * trajd.Om);
    M = Mff + Mfb;
    u = max(min(M,obj.Mmax),obj.Mmin);
end

function [u, delta, V, LgV, LfV] = clf_qp(t,x, obj)
    trajd = trajectory(obj, t);
    
    R = reshape(x(1:9),3,3);
    Om = x(10:12);
    Om_hat = hat(Om);
    
	% compute SO(3) error
    eR = 0.5*vee(trajd.R'*R-R'*trajd.R);
    eOmega = Om-R'*trajd.R*trajd.Om;
    dR = R * Om_hat;
    dRc = trajd.R * hat(trajd.Om);
    deR = (vee(dRc' * R - R' * dRc) + vee(trajd.R' * dR - dR' * trajd.R)) / 2;

    eta2 = 150;
    J_scale = obj.J / min(eig(obj.J));
    epsilon2 = -4;
    c2 = 20;
    
    % lyapunov functions
    V = eOmega' * J_scale * eOmega / 2 + epsilon2 * (eR' * eOmega) ...
        +c2 * (eR' * eR) / 2;
    LgV = (eOmega' * J_scale + epsilon2 * eR');
    LfV = (epsilon2 * eOmega' + c2 * eR') * deR ...
        -LgV * (dR' * trajd.R * trajd.Om + R' * trajd.R * trajd.dOm);

    % QP
    H2 = diag([1, 1, 1, 4e2]);
    f2 = zeros(4, 1);
    A2 = [LgV, -1];
    b2 = -LfV - eta2 * V;
    optoption_2 = optimset('Display', 'off', 'TolFun', 1e-5);
	LB = [obj.iJ * (obj.Mmin - Om_hat * obj.J * Om); -inf];
    UB = [obj.iJ * (obj.Mmax - Om_hat * obj.J * Om); inf];
    
    options = optimoptions('quadprog',...
                            'Display', 'off',...
                            'Algorithm','active-set');
%     ws = optimwarmstart(obj.x0,options);
    x = quadprog(H2, f2, A2, b2, [], [], LB, UB, obj.x0, options);
    
    delta = x(4);
    dOmega = x(1:3);
    u = obj.J * dOmega + Om_hat * obj.J *Om;

end

function [u, delta, V, LgV, LfV] = dclf_qp(t,x, obj)
    trajd = trajectory(obj, t);

    R = reshape(x(1:9),3,3);
    Om = x(10:12);
    Om_hat = hat(Om);
    
    Q = diag([1, 1, 1, 4e2]);
    f = zeros(4, 1);
    c = 0;    
    
    
    % %% constants
    eta2 = 150;
    gamma = 0.9; 
    epsilon2 = 8; % original value: 4
    c2 = 20;
    
    [H, k, d, ~] = quadratic_dclf_const_mex(1/obj.FREQ, R, Om, trajd.R, trajd.Om,...
                                        gamma, c2, epsilon2,  obj.J, obj.iJ, obj.Js);

    LB = [(trajd.M+obj.Mmin); -inf];
    UB = [(trajd.M+obj.Mmax); inf];
    
	y = qcqp_mex(Q, f, c, H, k, d, LB, UB, obj.x0);
    u = y(1:3);
    delta = y(4);

    % compute SO(3) error
    eR = 0.5*vee(trajd.R'*R-R'*trajd.R);
    eOmega = Om-R'*trajd.R*trajd.Om;
    dR = R * Om_hat;
    dRc = trajd.R * hat(trajd.Om);
    deR = (vee(dRc' * R - R' * dRc) + vee(trajd.R' * dR - dR' * trajd.R)) / 2;

    eta2 = 150;
    gamma = 0.9; 
    J_scale = obj.J / min(eig(obj.J));
    epsilon2 = 1; 4;
    c2 = 20;

    % lyapunov functions
    V = eOmega' * J_scale * eOmega / 2 + epsilon2 * (eR' * eOmega) ...
    +c2 * (eR' * eR) / 2;
    LgV = (eOmega' * J_scale + epsilon2 * eR');
    LfV = (epsilon2 * eOmega' + c2 * eR') * deR ...
    -LgV * (dR' * trajd.R * trajd.Om + R' * trajd.R * trajd.dOm);
%     delta = 0;
%     %     
%     % integrated dynamics
%     trajd2 = trajectory(obj, t+1/obj.FREQ);
%     R2 = R*expm(hat(Om*1/obj.FREQ));
%     eR2 =  0.5*vee(trajd2.R'*R2-R2'*trajd2.R);
% 
%     dt = 1/obj.FREQ;
%     b = 0.5*c2*(eR2'*eR2);
%     rho = Om - R2'*trajd2.R*trajd2.Om;
%     tmp = epsilon2*eR2';
% 
%     Q = diag([1, 1, 1, 4e2]);
%     f = zeros(4, 1);
%     c = 0;    
% 
%     LB = [(trajd.M+obj.Mmin); -inf];
%     UB = [(trajd.M+obj.Mmax); inf];
%     
%     
% %     Ht = J_scale*dt*dt;
% %     kt = (dt*rho'*J_scale+ dt*tmp);
% %     dt = b+ 0.5*rho'*J_scale*rho + tmp*rho + (gamma-1)*V;
% %     
%     % setup one!
% %     H = blkdiag(Ht,0);
% %     k = [kt', -1]';
% %     d = dt;
% %     
% %     y = qcqp_mex(Q, f, c, H, k, d, LB, UB, obj.x0);
% 
% %     delta= y(4);
% %     dOmega = y(1:3);
% %     u = obj.J * dOmega + Om_hat * obj.J *Om;
%     
%     % setup two!
% %     H2 = obj.iJ'*Ht*obj.iJ;
% %     OmJOm = Om_hat*obj.J*Om;
% %     k2 = kt*obj.iJ -OmJOm'*H2;
% %     d = 0.5*OmJOm'*H2*OmJOm -kt*obj.iJ*OmJOm +dt;
% %     
% %     H = blkdiag(H2,0);
% %     k = [k2, -1]';
% 
%     y = qcqp_mex(Q, f, c, H, k, d, LB, UB, obj.x0);
%     u = y(1:3);
%     delta = y(4);

end

function [y,grady] = quadobj(x,Q,f,c)
    y = 1/2*x'*Q*x + f'*x + c;
    if nargout > 1
        grady = Q*x + f;
    end
end
function [y,yeq,grady,gradyeq] = quadconstr(x,H,k,d)
    jj = length(H); % jj is the number of inequality constraints
    y = zeros(1,jj);
    for i = 1:jj
        y(i) = 1/2*x'*H{i}*x + k{i}'*x + d{i};
    end
    yeq = [];

    if nargout > 2
        grady = zeros(length(x),jj);
        for i = 1:jj
            grady(:,i) = H{i}*x + k{i};
        end
    end
    gradyeq = [];
end
function hess = quadhess(x,lambda,Q,H)
    hess = Q;
    jj = length(H); % jj is the number of inequality constraints
    for i = 1:jj
        hess = hess + lambda.ineqnonlin(i)*H{i};
    end
end

function [u, delta, V, LgV, LfV] = variation_clf_qp(t,x, obj)
    [A, B] = variation_lin_dyn(obj, t);
	trajd = trajectory(obj, t);
    
    R = reshape(x(1:9),3,3);
    Om = x(10:12);
    Om_hat = hat(Om);
    
	% compute SO(3) error
    eR = 0.5*vee(trajd.R'*R-R'*trajd.R);
    eOmega = Om-R'*trajd.R*trajd.Om;
    
    x = [eR; eOmega];
    V  = x'*obj.P*x;
    LfV = 2*x'*obj.P*A*x;
    LgV = 2*x'*obj.P*B;
    
	eta2 = 50;
    
	% QP
    H2 = diag([1, 1, 1, 4e2]);
    f2 = zeros(4, 1);
    A2 = [LgV, -1];
    b2 = -LfV - eta2 * V;
    optoption_2 = optimset('Display', 'off', 'TolFun', 1e-5);
	LB = [(obj.Mmin); -inf];
    UB = [(obj.Mmax); inf];
    
    x = quadprog(H2, f2, A2, b2, [], [], LB, UB, [], optoption_2);
    
    delta = x(4);
    dM = x(1:3);
    u = trajd.M + dM;
end

function [u, delta, V, LgV, LfV] = variation_dclf_qp(t,x, obj)
    [A, B] = variation_lin_dyn(obj, t);
    A = A*1/obj.FREQ+eye(6);
    B = B*1/obj.FREQ;
    
	trajd = trajectory(obj, t);
    
    R = reshape(x(1:9),3,3);
    Om = x(10:12);
    Om_hat = hat(Om);
    
	% compute SO(3) error
    eR = 0.5*vee(trajd.R'*R-R'*trajd.R);
    eOmega = Om-R'*trajd.R*trajd.Om;
    
    eta2 = 150;
    J_scale = obj.J / min(eig(obj.J));
    epsilon2 = 4;
    c2 = 20;	
    gamma = eta2*1/obj.FREQ;

    x = [eR; eOmega];
    
%     P = obj.Pk;
    P = 0.5*[[c2*eye(3), epsilon2*eye(3)]; [epsilon2*eye(3), J_scale]];
    
    V  = x'*P*x;
    LfV = 2*x'*P*A*x;
    LgV = 2*x'*P*B;
    
    
	% QP
    Q = diag([1, 1, 1, 4e2]);
    f = zeros(4, 1);
    c = 0;

    H = blkdiag(2*B'*P*B, 0);
    k = [2*x'*A'*P*B, -1]';
    d = (gamma-1)*V + x'*A'*P*A*x;
    
    LB = [(trajd.M+obj.Mmin); -inf];
    UB = [(trajd.M+obj.Mmax); inf];
    y = qcqp_mex(Q, f, c, H, k, d, LB, UB, obj.x0);
    
    delta = y(4);
    dM = y(1:3);
    u = trajd.M + dM;
end

%% trajectory
function[traj] = trajectory(obj,t)
    traj.R = eye(3);
    traj.Om = zeros(3,1);
    traj.dOm = zeros(3,1);
    traj.M = obj.J*traj.dOm + hat(traj.Om)*obj.J*traj.Om;
end

function[A, B] = variation_lin_dyn(obj, t)
    trajd = trajectory(obj, t);
    
    A66 = obj.iJ*(hat(obj.J*trajd.Om)-hat(trajd.Om)*obj.J);
    A = [[-hat(trajd.Om), eye(3)];
            [zeros(3,3), A66]];
    B = [zeros(3,3); obj.iJ];
end

function obj = computeGains(obj)
    [A, B] = variation_lin_dyn(obj, 0);
    Q = diag([100, 100, 100, 10, 10, 10]);
    R = diag([1,1,1]);
    
    [K,S,CLP] = lqr(A,B,Q,R);
    obj.P = S;
    
end
function obj = computeGains2(obj)
    [A, B] = variation_lin_dyn(obj, 0);
    A = A*1/obj.FREQ+eye(6);
    B = B*1/obj.FREQ;
    
    Q = diag([100, 100, 100, 10, 10, 10]);
    R = 1*diag([1,1,1]);
    
    [K,S,CLP] = dlqr(A,B,Q,R);
    obj.Pk = S;
    
end

%%
% 
% inertia = [0.0023, 1e-7, 1e-7;
%         1e-7, 0.0023, 1e-7;
%         1e-7, 1e-7, 0.0076];
% inertia_inv = [434.7826   -0.0189   -0.0057;
%    -0.0189  434.7826   -0.0057;
%    -0.0057   -0.0057  131.5789];
% 
% inertia_scaled = [    1.0000    0.0000    0.0000;
%     0.0000    1.0000    0.0000;
%     0.0000    0.0000    3.3045];

% inertia= [0.004900, 0.000006, 0.000005;
%     0.000006, 0.005300, 0.000021;
%     0.000005, 0.000021, 0.009800];
% inertia_inv=[204.081955, -0.211339, -0.112001;
%     -0.211340, 188.681046, -0.404200;
%     -0.112001, -0.404201, 102.041748];
% inertia_scaled=[1.000016, 0.001122, 0.001102;
%     0.001122, 1.081650, 0.004286;
%     0.001102, 0.004286, 2.000033];


