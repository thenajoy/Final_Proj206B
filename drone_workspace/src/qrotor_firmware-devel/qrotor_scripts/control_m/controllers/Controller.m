classdef Controller < handle 
properties
    rates_integral = zeros(3,1);

    mQ = 0.85;
    g = 9.80665;
    J =  [0.0023, 1e-7, 1e-7;
        1e-7, 0.0023, 1e-7;
        1e-7, 1e-7, 0.0076];
    iJ double
    J_scale double

    e1 = [1; 0; 0];
    e2 = [0; 1; 0];
    e3 = [0; 0; 1];

    POS_FREQ = 100;
    ATT_FREQ = 500;
    dt = 1 / 500;
    fmin = 1.5740;
    fmax = 23.6746;
    Mmax = [1.3149; 1.3149; 0.1704];
    Mmin = [-1.3149; -1.3149; -0.1704];

    rates_lpf_ = LowPassFilterSO(500, 50);

    RATES_INT_LB = [-0.15,-0.15,-0.15];
    RATES_INT_UB = [0.15,0.15,0.15];
    
    trajectory function_handle
    flat2state function_handle
end
%%
methods
   function obj = Controller(varargin)
       obj.iJ = inv(obj.J);
       obj.J_scale = obj.J/min(eig(obj.J));
   end
   
   function updateInertia(obj, inertia)
      obj.J = inertia; 
       obj.iJ = inv(obj.J);
       obj.J_scale = obj.J/min(eig(obj.J));
   end
   
   function cmd = getCmdTraj(obj,t)
       cmd = obj.flat2state(obj.mQ, obj.J, obj.trajectory(t));
   end
end

    
end