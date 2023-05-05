classdef SO3VariationLQR < Controller
properties
    kp_att = [8; 8; 3];
    kp_Om = [0.3; 0.3; 0.225];
    kd_Om = zeros(3, 1);
    ki_Om = [0.15; 0.15; 0.05];

    % clf-qp
    moment = zeros(3, 1);

    yaw_w = 0;
    eta2 = 150;
    epsilon2 = 2*4;
    c2 = 20;

    C2 = eye(3);
    E2 = eye(3);
    KJ = eye(3);
    
    kpPos = 10* diag([1, 1, 1]);
    kdPos = 6 * diag([1, 1, 1]);
    Kpos = [eye(3), eye(3)];
    
    x0 = zeros(4,1);
end
%%
methods
    function obj = SO3VariationLQR(varargin)
        
        % computing position gains
        Apos = [zeros(3,3), eye(3); zeros(3,6)]; 
        Bpos = [zeros(3,3); eye(3)/obj.mQ];
        dt_pos = 1/obj.POS_FREQ;

        Aposk = eye(6) + Apos*dt_pos;
        Bposk = Bpos*dt_pos;
        
        Q = diag([100, 100, 100, 10, 10, 10]); 
        R = diag([1,1,1]);
        obj.Kpos = dlqr(Aposk, Bposk, Q, R);
        
        
    end
    function [F] = position_control(obj,t, x)
        xQ = x(1:3);
        vQ = x(4:6);
        
        cmd = obj.getCmdTraj(t);
        eta = [xQ-cmd.p; vQ-cmd.v];
        
        F = - obj.Kpos*eta + obj.mQ * obj.g * obj.e3;

    end
    function [u, moment] = apply(obj,t, F, R, Omega)
        cmd = obj.getCmdTraj(t);

        % compute the controlled rotation matrix
        e3c = F / norm(F);
        e3c_hat = hat(e3c);
        e1c = -e3c_hat^2 * cmd.R(:,1);
        e1c = e1c / norm(e1c);
        e2c = cross(e3c, e1c);
        Rc = [e1c, e2c, e3c];

        % compute SO(3) error
        eR = vee(Rc'*R-R'*Rc) / 2;
        eOmega = Omega - R' * Rc * cmd.Omega;
        
        dt = 1/obj.ATT_FREQ;
        [A, B] = obj.variation_lin_dyn(t);
        Ak = eye(6)+A*dt;
        Bk = B*dt;
        
        Q1 = diag([10, 10, 10, 0.1, 0.1, 0.1]);
        Q2 = 1*eye(3);
        [K, ~, ~] = dlqr(Ak, Bk, Q1, Q2);
        M = -K*[eR; eOmega];
        moment = M + cmd.M;
        
        f = F' * R(:, 3);
        M = max(min(moment,obj.Mmax),obj.Mmin);
        u = [f; M];
        
    end
    function[A, B] = variation_lin_dyn(obj, t)
        trajd = obj.getCmdTraj(t);

        A66 = obj.iJ*(hat(obj.J*trajd.Omega)-hat(trajd.Omega)*obj.J);
        A = [[-hat(trajd.Omega), eye(3)];
                [zeros(3,3), A66]];
        B = [zeros(3,3); obj.iJ];
    end
end


end