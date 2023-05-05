classdef RateController < handle
    properties
        rates_integral = zeros(3,1);
        
        mQ =0.85;
        g = 9.81;
        
        ATT_FREQ = 500;
        dt = 1 / 500;
        fmin = 1.5740;
        fmax = 23.6746;
        Mmax = [1.3149; 1.3149; 0.1704];
        Mmin = [-1.3149; -1.3149; -0.1704];
        
        rates_lpf_ = LowPassFilterSO(500, 50);
        
        RATES_INT_LB = [-0.15,-0.15,-0.15];
        RATES_INT_UB = [0.15,0.15,0.15];
        
        kp_att = [8; 8; 3];
        kp_Om = [0.3;0.3;0.225];
        kd_Om = zeros(3,1);
        ki_Om = [0.15;0.15;0.05];
    end
    methods
        function obj = RateController(varargin)

        end
        function [u, M_] = apply(obj, F, R, Omega)
            % control input
            f = max(min(F(3), obj.fmax), obj.fmin);


            [r, p, y] = RotToRPY_ZXY(R);
            cmd_rpy = [((F(1) / obj.mQ) * sin(y) - (F(2) / obj.mQ) * cos(y)) / obj.g; ...
                ((F(1) / obj.mQ) * cos(y) + (F(2) / obj.mQ) * sin(y)) / obj.g; ...
                0.0];
            rpy = [r; p; y];

            % compute rate-sp
            rates_sp = -obj.kp_att .* (rpy - cmd_rpy);
            
            rates_err = Omega-rates_sp;
            rates_derv = obj.rates_lpf_.apply(Omega);
            obj.rates_integral = obj.rates_integral + rates_err*obj.dt;

            % unbounded moment
            M_ = -obj.kp_Om.*(rates_err)-obj.kd_Om.*(rates_derv)-obj.ki_Om.*(obj.rates_integral);
            
            M = max(min(M_, obj.Mmax), obj.Mmin);
            u = [f;M];
        end
    end
end