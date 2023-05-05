classdef EulerPD < Controller
properties

kpEuler = diag([1, 1, 0.7943]);
kdEuler = diag([0.12, 0.12, 0.15]);
kiEuler = 0*diag([1,1,1]);

kpPos = 10*diag([1,1,1]);
kdPos = 6*diag([1, 1, 1]);

end
%%
methods
function obj = EulerPD(varargin)

end
function [F] = position_control(obj, t, x)
    xQ = x(1:3);
    vQ = x(4:6);
    cmd = obj.getCmdTraj(t);

    F = -obj.kpPos*(xQ-cmd.p) -obj.kdPos*(vQ-cmd.v)...
        +(obj.mQ*obj.g*obj.e3) + cmd.a; 
end
function [u, M_] = apply(obj, t, F, R, Omega)
    cmd = obj.getCmdTraj(t);

    % control input
    f = max(min(F(3), obj.fmax), obj.fmin);

    [r, p, y] = RotToRPY_ZXY(R);   
    cmd_rpy = [((F(1)/obj.mQ)*sin(y) - (F(2)/obj.mQ)*cos(y))/obj.g;
                ((F(1)/obj.mQ)*cos(y) + (F(2)/obj.mQ)*sin(y))/obj.g;
                0.0];

    err_rpy = ([r;p;y]-cmd_rpy);
    err_body_rates =Omega-zeros(3,1);
    obj.rates_integral =  obj.rates_integral+err_rpy*obj.dt;

    M_ = -obj.kpEuler*err_rpy -obj.kdEuler*err_body_rates-obj.kiEuler*obj.rates_integral ;
    M_ = M_ + cmd.M;
    
    M = max(min(M_,obj.Mmax),obj.Mmin);
    u = [f;M];

end
end
% end of classdef
end