classdef QrotorHardwareSimulation < handle
% QrotorHardwareSimulation

%%
properties
    
    type
    mQ double
    JQ double
    
    g = 9.80665;
    
    e1 = [1; 0; 0];
    e2 = [0; 1; 0];
    e3 = [0; 0; 1];


    fmin = 1.5740;
    fmax = 23.6746;
    Mmax = [1.3149; 1.3149; 0.1704];
    Mmin = [-1.3149; -1.3149; -0.1704];
    
    % bounds
    bounds struct

    controller 
    controlParams struct
    odeopts = odeset();
    
    include_noise = 0;

    % noise covariances
	xVar = diag([0.25e-9, 0.25e-9, 0.25e-9]);
	vVar = diag([0.25e-6, 0.25e-6, 0.25e-6]);
	eVar = diag([0.25e-6, 0.25e-6, 0.25e-6]);
	bVar = diag([0.000012341280121e-6, 0.000013413255965e-6, 0.000002635158647e-6]);

    ATT_FREQ = 500;
    POS_FREQ = 100;

end

properties (Constant = true)
    nDof = 6;
    nAct = 2;
end

methods
	
	% class constructor
	function obj = QrotorHardwareSimulation(params,varargin)
        
        if isfield(params, 'mQ')
            obj.mQ = params.mQ;
        else
            obj.mQ = 0.85;
        end

        if isfield(params, 'JQ')	
            obj.JQ = params.JQ;
        else
            obj.JQ = [0.005315307431627, 0.000005567447099, 0.000005445855427;
                        0.000005567447099, 0.004949258422243, 0.000020951458431;
                        0.000005445855427, 0.000020951458431, 0.009806225007686;];
        end

        if isfield(params,'fmin')
            obj.fmin = params.fmin;
        else
            obj.fmin = 1.5740;
        end

        if isfield(params,'fmax')
            obj.fmax = params.fmax;
        else
            obj.fmax = 23.6746;
        end
        
        obj.type = 'quadrotor';

    end     
%%    
    % set property values
    function obj = setProperty(obj, propertyName, value)
        if isprop(obj, propertyName)
            set(obj, propertyName, value);
        else
            error([propertyName ' not a property of class ',class(obj)]);
        end
    end

    function sol = simulate(obj, tspan, x0, u, solver)
        odefun = @(t,x)systemDynamics(obj, t, x, u);
        sol = solver(odefun, tspan, x0, obj.odeopts);
    end
    
    function [log] = discrete_simulate(obj, tspan, x0, solver)
        odeset('MaxStep', 1/obj.ATT_FREQ);
        % logs
        xLog = struct('ideal',[],'actual',[]);
        eulerLog = struct('ideal',[],'actual',[]);
        uLog = struct('ideal',[],'actual',[]);

        xLog.ideal = x0;
        xLog.actual = x0;
        [r,p,y] =  RotToRPY_ZXY(reshape(x0(7:15,end),3,3));
        eulerLog.ideal = [r;p;y];
        eulerLog.actual = [r;p;y];

        t0 = tspan(1);
        tf = tspan(end);
        tPOS = t0:1/obj.POS_FREQ:tf;
        tTOTAL = t0:1/obj.ATT_FREQ:tf;
        
        progressbar
        N = length(tPOS);
        for it = 2:N
            
            t = tPOS(it-1);
            F = obj.controller.position_control(t, x0);

            % attitude control
            tATT = tPOS(it-1):1/obj.ATT_FREQ:tPOS(it);
            for jt = 2:length(tATT)
                [u, M_] = obj.controller.apply(tATT(jt-1), F, reshape(x0(7:15),3,3), x0(16:18));
                f = u(1);
                M = u(2:4);
                % simulate
                sol = obj.simulate([tATT(jt-1),tATT(jt)], x0, [f; M], solver);

                [r,p,y] = RotToRPY_ZXY(reshape(sol.y(7:15,end),3,3));
                if (obj.include_noise)
                    dEuler = chol(obj.eVar)*randn(3,1);
                    x0 = [sol.y(1:3,end)+chol(obj.xVar)*randn(3,1);
                            sol.y(4:6,end)+chol(obj.vVar)*randn(3,1);
                            reshape(RPYtoRot_ZXY(r+dEuler(1), p+dEuler(2), y+dEuler(3)),9,1);
                            sol.y(16:18,end)+chol(obj.bVar)*randn(3,1)];
                else
                    dEuler = zeros(3,1);
                    x0 = sol.y(:,end);
                end

                xLog.ideal = [xLog.ideal, sol.y(:,end)];
                xLog.actual = [xLog.actual, x0];
                eulerLog.ideal = [eulerLog.ideal, [r;p;y]];
                eulerLog.actual = [eulerLog.actual, [[r;p;y]+dEuler]];
                uLog.ideal = [uLog.ideal, [F(3); M_]];
                uLog.actual = [uLog.actual, [f;M]];
            end
        % 	fprintf("simulated upto %.4f\n",tATT(end));
            progressbar(it/N);
        end
        uLog.ideal = [uLog.ideal, [F(3); M_]];
        uLog.actual = [uLog.actual, [f;M]]; 
        
        log.tTOTAL = tTOTAL;
        log.xLog = xLog;
        log.uLog = uLog;
        log.eulerLog = eulerLog;
    end

    function [log] = continuous_simulate(obj, tspan, x0, solver)
        ode_fun = @(t,x) ode_cont_dyn(obj, t, x);
        obj.odeopts = odeset();
        sol = solver(ode_fun, tspan, x0, obj.odeopts);
        
        log.tTOTAL = sol.x;
        log.xLog.ideal = sol.y;
        log.xLog.actual = sol.y;
        L = size(sol.x,2);
        log.uLog.ideal = zeros(4,L);
        log.uLog.actual = zeros(4,L);
        log.eulerLog.ideal = zeros(3,L);
        for i =1:L
            [F, u, M_] = obj.compute_input(sol.x(i), sol.y(:,i));
            log.uLog.ideal(:,i) = [F(3); M_];
            log.uLog.actual(:,i) = u;
            log.eulerLog.actual(:,i) = Rot2RPY_ZXY(reshape(sol.y(7:15,i),3,3));
        end
        log.eulerLog.ideal = log.eulerLog.actual;
    end
    
    function [F, u, M_] = compute_input(obj, t, x)
        F = obj.controller.position_control(x);
        [u, M_] = obj.controller.apply(F, reshape(x(7:15),3,3), x(16:18));       
    end
    function [dx] = ode_cont_dyn(obj, t, x)
        [~, u, ~] = obj.compute_input(t, x);
        dx = obj.systemDynamics(t,x,u);
        fprintf("%.4f\n",t);
    end
    
    function dx = systemDynamics(obj, t, x, u)
        
        vQ = x(4:6); R = reshape(x(7:15),3,3); Om = x(16:18);
        f = u(1);
        M = u(2:4);
        dx = [vQ; 
                -obj.g*obj.e3+(f*R*obj.e3)/obj.mQ;
                reshape(R*hat(Om),9,1);
                obj.JQ\(M-cross2(Om,obj.JQ*Om))];
        
    end
    
    function u = calcControlInput(obj, t, x)
        u = obj.controller(obj, t,x);
    end
 
%%  methods defined externally
%     % discrete dynamics
%     % dx = f(xref,uref) + A(x-xref)+B(u-uref)
%     [A, B] = discretizeLinearizeQuadrotor(obj, Ts, xk, uk);
%     
%     % differential flatness
%     [ref] = flat2state(obj,flats);
    
    % animation
    animateQuadrotor(obj,opts_in);
    
    
end

end