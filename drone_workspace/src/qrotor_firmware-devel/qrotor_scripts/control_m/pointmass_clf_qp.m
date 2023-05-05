function pointmass_clf_qp()
close all


%%

obj.m = 1;
obj.g = 9.81;
obj.e3 = [0;0;1];

obj.controller = @pd_ctrl;
obj.controller = @clf_qp;
obj.FREQ = 100;

%
y0 = [ones(3,1); zeros(3,1)];

tic
% log = csimulate([0,5], y0, obj);
log = dsimulate([0,5], y0, obj);
toc

% plots
% -----
figure; 
for it = 1:3
    subplot(3,1,it);
    hold on;
    stairs(log.t, log.u(it,:), 'linewidth', 2);
    grid on; grid minor;
    latex_title(strcat('$$f_',num2str(it),'$$'));
end

figure; 
for it = 1:3
    subplot(3,2,it*2-1);
    hold on;
    plot(log.t, log.x(it,:), 'linewidth', 2);
    grid on; grid minor;
    latex_title(strcat('$$p_',num2str(it),'$$'));

    
	subplot(3,2,it*2);
    hold on;
    plot(log.t, log.x(it+3,:), 'linewidth', 2);
    grid on; grid minor;
    latex_title(strcat('$$v_',num2str(it),'$$'));
end



end


%% dynamics
function [dx] = ode_fun(t,x,obj)
    dx = [x(4:6);
            obj.controller(t,x, obj)/ obj.m - obj.g * obj.e3;];
end
function [dx] = ode_fun1(t,x,obj)
    dx = [x(4:6);
            obj.u/ obj.m - obj.g * obj.e3;];
end

function [log] = csimulate(tspan,x0,obj)
    options = odeset('RelTol', 1e-4, 'AbsTol', 1e-8);
    sol = ode15s(@ode_fun,tspan,x0,options, obj);
    
    log.t = sol.x;
    log.x = sol.y;
    L = size(log.t,2);
    log.u = zeros(3,L);
    for i = 1:L
        log.u(:,i) = obj.controller(log.t(i), log.x(:,i), obj);
    end

end

function [log] = dsimulate(tspan,x0,obj)
t = tspan(1):1/obj.FREQ:tspan(2);
N = length(t);

log.t = t;
log.u = zeros(3,N);
log.x = zeros(6,N);
log.x(:,1) = x0;

progressbar
for k = 2:N
   obj.u = obj.controller(t(k-1), x0, obj);
   options = odeset('RelTol', 1e-4, 'AbsTol', 1e-8, 'MaxStep', 1/obj.FREQ);
   sol = ode15s(@ode_fun1,[t(k-1), t(k)], x0, options, obj);
   
   x0 = sol.y(:,end);
   log.u(:,k-1) = obj.u;
   log.x(:,k) = x0;
	progressbar(k/N);
end
log.u(:,k) = obj.u;

end

%% controllers

function [u] = pd_ctrl(t,x, obj)
    trajd = trajectory(t);
    
    ex = x(1:3)-trajd.x;
    ev = x(4:6)-trajd.v;
    
    kx = 10*eye(3);
    kv = 5*eye(3);
    
    u = -kx*ex-kv*ev;
end

function [u] = clf_qp(t,x, obj)
    trajd = trajectory(t);

    % compute the position and velocity error
    ex = x(1:3) - trajd.x;
    ev = x(4:6) - trajd.v;

    % mass property
    m = obj.m;
    g = obj.g;
    e3 = [0; 0; 1];

    % first level of QP : compute the virtual force
    eta1 = 4;
    epsilon1 = 2;
    c1 = 6;

    % scale the mass properties to
    V1 = (ev' * ev) / 2 + epsilon1 * (ex' * ev) + c1 * (ex' * ex) / 2;
    LgV1 = (ev + epsilon1 * ex)';
    LfV1 = epsilon1 * (ev' * ev) + c1 * (ev' * ex) - LgV1 * trajd.a;

    A1 = LgV1;
    b1 = -LfV1 - eta1 * V1;
    H1 = diag([5, 5, 5]);
    f1 = zeros(3, 1);
    optoption_1 = optimset('Display', 'off', 'TolFun', 1e-10); % the first level has more accurate
    accd = quadprog(H1, f1, A1, b1, [], [], [], [], [], optoption_1);
    u = m * (accd + g * e3);

end


%% trajectory
function[traj] = trajectory(t)
    traj.x = zeros(3,1);
    traj.v = zeros(3,1);
    traj.a = zeros(3,1);
end
