classdef TiltPrioritizedController < handle
    properties

        mQ = 0.85;
        g = 9.81;

        ATT_FREQ = 500;
        dt = 1 / 500;
        fmin = 1.5740;
        fmax = 23.6746;
        Mmax = [1.3149; 1.3149; 0.1704];
        Mmin = [-1.3149; -1.3149; -0.1704];

        rates_lpf_ = LowPassFilterSO(500, 50);

        RATES_INT_LB = [-0.15, -0.15, -0.15];
        RATES_INT_UB = [0.15, 0.15, 0.15];

        kp_att = [8; 8; 3];
        kp_Om = [0.3; 0.3; 0.225];
        kd_Om = zeros(3, 1);
        ki_Om = [0.15; 0.15; 0.05];

        yaw_w = 0;
        
        rate_limit = deg2rad([220;220;200]); % rad/s
        gain_p = [0.3; 0.3; 0.225];
        gain_d = zeros(3, 1);
        gain_i = [0.15; 0.15; 0.05];
        gain_ff = zeros(3,1);
        rate_int = zeros(3, 1);
        lim_int = [0.3, 0.3, 0.1]; 
    end
    %%
    methods
        function obj = TiltPrioritizedController(varargin)

        end
        function [u, moment] = apply(obj, F, R, Omega)
            % control input
            f = max(min(F(3), obj.fmax), obj.fmin);

            % calculating desired orientation
            b3c = F / norm(F);
%             b3 = R(:, 3);

            th = deg2rad(0);
            b1d = [cos(th); sin(th); 0];
            b1c = -vec_cross(b3c, vec_cross(b3c, b1d));
            b1c = b1c / norm(vec_cross(b3c, b1d));
            Rc = [b1c, vec_cross(b3c, b1c), b3c];
            Rd = Rc;
            
            % rotation error
            Re = Rd'*R;
%             Omegae = 
            e3 = [0;0;1];
            rho_r = acos(e3'*Re*e3);
            if abs(rho_r) > 1e-6
                n_r = hat(Re'*e3)*e3/sin(rho_r);
            else
                n_r = e3;
            end
            Rr = expm(hat(n_r*rho_r));
            
            Ry = Re*Rr';
            eta_y = vee(logm(Ry));
            rho_y = norm(eta_y);
            if abs(rho_y)>1e-6
                n_y = eta_y/rho_y;
            else
                n_y = e3;
            end

            kr = 8;
            ky = 1;
            
            rates_sp = -2*kr*n_r*sin(rho_r/2)-2*ky*n_y*sin(rho_y/2);
            % todo add desired trajectory
            % rates_sp +=  R'*Rd*Omegad;
            
            % limit-rates
            rates_sp = max(min(rates_sp, obj.rate_limit), -obj.rate_limit);
            
%             moment = obj.generic_rate_pid(Omega, rates_sp);
            moment = obj.rate_control(Omega, rates_sp);

            % bound the input
            M = max(min(moment, obj.Mmax), obj.Mmin);
            
            u = [f;M];
        end
        
        function moment = generic_rate_pid(obj, rates, rates_sp)
            rates_err = rates-rates_sp;
            rates_derv = obj.rates_lpf_.apply(rates);
            obj.rate_int = obj.rate_int + rates_err*obj.dt;

            % unbounded moment
            moment = -obj.kp_Om.*(rates_err)-obj.kd_Om.*(rates_derv)-obj.ki_Om.*(obj.rate_int); 
        end
        
        function torque = rate_control(obj, rates, rate_sp)
            % angular rates error
            rate_error = rate_sp - rates;
            
            % angular accel
            angular_accel = obj.rates_lpf_.apply(rates);

            % PID control with feed forward
            torque = obj.gain_p.*(rate_error) + obj.rate_int - obj.gain_d.*(angular_accel) + obj.gain_ff.*(rate_sp);

            % update integral only if we are not landed
            obj.updateIntegral(rate_error, obj.dt);            
        end
        
        function updateIntegral(obj, rate_error, dt)
            for i = 1:3
                % I term factor: reduce the I gain with increasing rate error.
                % This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
                % change (noticeable in a bounce-back effect after a flip).
                % The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
                % with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
                % and up to 200 deg error leads to <25% reduction of I.
                i_factor = rate_error(i) / deg2rad(400.);
                i_factor = max(0., 1. - i_factor * i_factor);

                % Perform the integration using a first order method
                rate_i = obj.rate_int(i) + i_factor * obj.gain_i(i) * rate_error(i) * dt;
                
                % do not propagate the result if out of range or invalid
                if ~isnan(rate_i)
                    obj.rate_int(i) = max(min(rate_i, obj.lim_int(i)), -obj.lim_int(i));
                end
            end
        end
    end
end