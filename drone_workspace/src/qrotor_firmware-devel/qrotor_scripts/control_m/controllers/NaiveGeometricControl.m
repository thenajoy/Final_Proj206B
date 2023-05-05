classdef NaiveGeometricControl < Controller
properties
%         kR = 1e2;
%         kOmega = 8;
    kR = 0.1*diag([10,10,10]);
    kOmega =0.1* diag([1,1,1]);

    kpPos = 10 * diag([1, 1, 1]);
    kdPos = 6 * diag([1, 1, 1]);


end
methods
    function obj = NaiveGeometricControl(varargin)

    end
    function [F] = position_control(obj, t, x)
        xQ = x(1:3);
        vQ = x(4:6);
        cmd = obj.getCmdTraj(t);

        F = -obj.kpPos * (xQ - cmd.p) - obj.kdPos * (vQ - cmd.v) ...
            + cmd.a +  (obj.mQ * obj.g * obj.e3);
    end
    function [u, M_] = apply(obj, t, F, R, Omega)
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

        % compute the moment and thrust
        Omega_hat = hat(Omega);
        f = F' * R(:, 3);
        Mfb = (-obj.kR * eR - obj.kOmega * eOmega);
        Mff = Omega_hat*obj.J*Omega ...
            +obj.J*(R'*Rc*cmd.dOmega-Omega_hat*R'*Rc*cmd.Omega);
        M_ = Mff + Mfb;
%         M = M_;
        M = max(min(M_,obj.Mmax),obj.Mmin);
        u = [f; M];
    end
end
end