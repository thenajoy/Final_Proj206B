classdef SO3GeometricCLFQP < Controller
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
    
    kpPos = 10 * diag([1, 1, 1]);
    kdPos = 6 * diag([1, 1, 1]);
    
    x0 = zeros(4,1);
end

%%
methods
    function obj = SO3GeometricCLFQP(varargin)
        obj.J_scale = obj.J / min(eig(obj.J));
        obj.iJ = inv(obj.J);

        obj.C2 = diag([20, 20, 10]); % 8.4, 8.4, 5.1
        obj.E2 = diag([4, 4, 1]);
        obj.KJ = diag([1, 1, 1]);

    end
    function [F] = position_control(obj,t, x)
        xQ = x(1:3);
        vQ = x(4:6);

        F = -obj.kpPos*(xQ-zeros(3, 1)) - obj.kdPos*(vQ-zeros(3, 1))...
            + (obj.mQ * obj.g * obj.e3);

%         % compute the position and velocity error
%         ex = x(1:3) - zeros(3,1);
%         ev = x(4:6) - zeros(3,1);
% 
%         % mass property
%         m = obj.mQ;
%         J = obj.J;
%         g = obj.g;
%         e3 = [0; 0; 1];
% 
%         % first level of QP : compute the virtual force
%         eta1 = 2.5;
%         epsilon1 = 2;
%         c1 = 6;
%         
%         accd = zeros(3,1);
% 
%         % scale the mass properties to
%         V1 = (ev' * ev) / 2 + epsilon1 * (ex' * ev) + c1 * (ex' * ex) / 2;
%         LgV1 = (ev + epsilon1 * ex)';
%         LfV1 = epsilon1 * (ev' * ev) + c1 * (ev' * ex) - LgV1 * accd;
% 
%         A1 = LgV1;
%         b1 = -LfV1 - eta1 * V1;
%         H1 = diag([5, 5, 1]);
%         f1 = zeros(3, 1);
%         optoption_1 = optimset('Display', 'off', 'TolFun', 1e-10); % the first level has more accurate
%         accd = quadprog(H1, f1, A1, b1, [], [], [], [], [], optoption_1);
%         F = m * (accd + g * e3);
    end
    function [u, moment] = apply(obj,t, F, R, Omega)
%         [u, moment] = obj.guofan_clf_qp(F, R, Omega);
        [u, moment] = obj.discrete_clf_qp(F, R, Omega);
%           [u, moment] = obj.variation_dclf_qp(F, R, Omega);
    end
    function [u, moment] = guofan_clf_qp(obj, F, R, Omega)
        J = obj.J;
        g = obj.g;
        e3 = [0; 0; 1];
        % second level of QP : orientation adjustment without considering
        % the external obstacle

        Rd = eye(3);
        % compute the controlled rotation matrix
        e3c = F / norm(F);
        e3c_hat = hat(e3c);
        e1b = -e3c_hat^2 * Rd(:, 1);
        e1c = e1b / norm(e1b);
        Rc = [e1c, cross(e3c, e1c), e3c];

        Omegad = zeros(3,1);
        dOmegad = zeros(3,1);

        % compute the orientation error
        %         R = reshape(x(7:15), 3, 3);
        Omega_hat = hat(Omega);
        eR = vee(Rc'*R-R'*Rc) / 2;
        eOmega = Omega - R' * Rc * Omegad;
        dR = R * Omega_hat;
        dRc = Rc * hat(Omegad);
        deR = (vee(dRc' * R - R' * dRc) + vee(Rc' * dR - dR' * Rc)) / 2;

        eta2 = 150;
        J_scale = J / min(eig(J));
        epsilon2 = 4;
        c2 = 20;
        V2 = eOmega' * J_scale * eOmega / 2 + epsilon2 * (eR' * eOmega) ...
        +c2 * (eR' * eR) / 2;
        LgV2 = (eOmega' * J_scale + epsilon2 * eR');
        LfV2 = (epsilon2 * eOmega' + c2 * eR') * deR ...
        -LgV2 * (dR' * Rc * Omegad + R' * Rc * dOmegad);

        H2 = diag([1, 1, 1, 4e2]);
        f2 = zeros(4, 1);
        A2 = [LgV2, -1];
        b2 = -LfV2 - eta2 * V2;
        LB = [obj.iJ * (obj.Mmin - Omega_hat * obj.J * Omega); -inf];
        UB = [obj.iJ * (obj.Mmax - Omega_hat * obj.J * Omega); inf];
%         LB = [];
%         UB = [];

        optoption_2 = optimset('Display', 'off', 'TolFun', 1e-5);
        uOpt = quadprog(H2, f2, A2, b2, [], [], LB, UB, [], optoption_2);
        dOmega = uOpt(1:3);
        M = J * dOmega + Omega_hat * J * Omega;
        f = F' * R(:, 3);
        moment = M;
        M = max(min(M, obj.Mmax), obj.Mmin);

        u = [f; M];
    end
    
    function [u, moment] = discrete_clf_qp(obj, F, R, Om)
        t = 0;
        trajd = obj.trajectory(t);
        Om_hat = hat(Om);
        
        e3 = [0; 0; 1];
        % second level of QP : orientation adjustment without considering
        % the external obstacle

        % compute the controlled rotation matrix
        e3c = F / norm(F);
        e3c_hat = hat(e3c);
        e1b = -e3c_hat^2 * trajd.R(:, 1);
        e1c = e1b / norm(e1b);
        Rc = [e1c, cross(e3c, e1c), e3c];
        trajd.R = Rc;
          
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
    %     
        % integrated dynamics
        trajd2 = trajectory(obj, t+1/obj.ATT_FREQ);
        trajd2.R = Rc;
        R2 = R*expm(hat(Om*1/obj.ATT_FREQ));
        eR2 =  0.5*vee(trajd2.R'*R2-R2'*trajd2.R);

        dt = 1/obj.ATT_FREQ;
        gamma =dt*eta2;
        b = 0.5*c2*(eR2'*eR2);
        rho = Om - R2'*trajd2.R*trajd2.Om;
        tmp = epsilon2*eR2';

        const.H{1} = blkdiag(J_scale*dt*dt,0);
        const.k{1} = [(dt*rho'*J_scale+ dt*tmp), -1]';
        const.d{1} = b+ 0.5*rho'*J_scale*rho + tmp*rho + (gamma-1)*V;

        cost.Q = diag([1, 1, 1, 4e2]);
        cost.f = zeros(4, 1);
        cost.c = 0;   
        y0 = zeros(4,1);
%         [y] = qcquadprog(cost, const, [],[], y0);

        LB = [obj.iJ * (obj.Mmin - Om_hat * obj.J * Om); -inf];
        UB = [obj.iJ * (obj.Mmax - Om_hat * obj.J * Om); inf];
    
        y = qcqp_mex(cost.Q, cost.f, cost.c, const.H{1}, const.k{1}, const.d{1}, LB, UB, y0);
        
        dOmega = y(1:3);
%         moment = obj.J*dOmega + Om_hat*obj.J*Om;
        f = F' * R(:, 3);
%         f = F(3);
        M = dOmega;
        u = [f;M];

    end
    
    function [u, moment] = variation_dclf_qp(obj, F, R, Om)
        t = 0;
        [A, B] = variation_lin_dyn(obj, t);
        A = A*1/obj.ATT_FREQ+eye(6);
        B = B*1/obj.ATT_FREQ;

        trajd = trajectory(obj, t);
        
        % compute the controlled rotation matrix
        e3c = F / norm(F);
        e3c_hat = hat(e3c);
        e1b = -e3c_hat^2 * trajd.R(:, 1);
        e1c = e1b / norm(e1b);
        Rc = [e1c, cross(e3c, e1c), e3c];
        trajd.R = Rc;

        % compute SO(3) error
        eR = 0.5*vee(trajd.R'*R-R'*trajd.R);
        eOmega = Om-R'*trajd.R*trajd.Om;

        eta2 = 50;
        J_scale = obj.J / min(eig(obj.J));
        epsilon2 = 4;
        c2 = 20;	
        gamma = eta2*1/obj.ATT_FREQ;

        x = [eR; eOmega];

    %     P = obj.Pk;
        P = 0.5*[[c2*eye(3), epsilon2*eye(3)]; [epsilon2*eye(3), J_scale]];

        V  = x'*P*x;
        LfV = 2*x'*P*A*x;
        LgV = 2*x'*P*B;


        % QP
        cost.Q = diag([1, 1, 1, 4e2]);
        cost.f = zeros(4, 1);
        cost.c = 0;

        const.H{1} = blkdiag(2*B'*P*B, 0);
        const.k{1} = [2*x'*A'*P*B, -1]';
        const.d{1} = (gamma-1)*V + x'*A'*P*A*x;

        LB = [(trajd.M+obj.Mmin); -inf];
        UB = [(trajd.M+obj.Mmax); inf];
        y = qcquadprog(cost, const, LB, UB, obj.x0);

        delta = y(4);
        dM = y(1:3);
        moment = trajd.M + dM;
        f = F'*R(:,3);
        u = [f;moment];
        
    end
    

end
end