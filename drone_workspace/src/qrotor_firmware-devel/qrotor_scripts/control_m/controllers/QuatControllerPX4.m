classdef QuatControllerPX4 < handle
    properties
        rates_integral = zeros(3, 1);

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
        
        rate_limit = deg2rad([220,220,200]); % rad/s
    end
    methods
        function obj = QuatControllerPX4(varargin)

        end
        function [f, M, M_] = apply(obj, F, R, Omega)
            % control input
            f = max(min(F(3), obj.fmax), obj.fmin);

            % calculating desired orientation
            b3c = F / norm(F);
            b3 = R(:, 3);

            b1d = [1; 0; 0];
            b1c = -vec_cross(b3c, vec_cross(b3c, b1d));
            b1c = b1c / norm(vec_cross(b3c, b1d));
            Rc = [b1c, vec_cross(b3c, b1c), b3c];
            Rd = Rc;

            q = rotm2quat(R);
            qd = rotm2quat(Rd);

            rates_sp = obj.compute_rates_sp(q, R, qd, Rd);

            rates_err = Omega-rates_sp;
            rates_derv = obj.rates_lpf_.apply(Omega);
            obj.rates_integral = obj.rates_integral + rates_err*obj.dt;

            % unbounded moment
            M_ = -obj.kp_Om.*(rates_err)-obj.kd_Om.*(rates_derv)-obj.ki_Om.*(obj.rates_integral);

            M = max(min(M_, obj.Mmax), obj.Mmin);
        end

        %%
        function [rate_setpoint] = compute_rates_sp(obj, q, R, qd, Rd)
            e_z = R(:, 3);
            e_z_d = Rd(:, 3);

            qd_red = obj.vecs2quat(e_z, e_z_d);

            if (abs(qd_red(1)) > (1. - 1e-5) || abs(qd_red(2)) > (1. - 1e-5))
                %     // In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
                %     // full attitude control anyways generates no yaw input and directly takes the combination of
                %     // roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
                qd_red = qd;
            else
                %     // transform rotation from current to desired thrust vector into a world frame reduced desired attitude
                qd_red = quatmultiply(qd_red, q);
            end

            %   // mix full and reduced desired attitude
            q_mix = quatmultiply(quatinv(qd_red), qd);

            % q_mix.canonicalize();
            for i = 1:4
                if abs(q_mix(i)) > 1e-5
                    q_mix = q_mix * sign(q(i));
                    break
                end
            end

            % catch numerical problems with the domain of acosf and asinf
            q_mix(1) = min(max(q_mix(1), -1), 1);
            q_mix(4) = min(max(q_mix(4), -1), 1);
            qd = quatmultiply(qd_red, [cos(obj.yaw_w * acos(q_mix(1))), 0, 0, sin(obj.yaw_w * asin(q_mix(4)))]);

            %   quaternion attitude control law, qe is rotation from q to qd
            qe = quatmultiply(quatinv( q),qd);
            
            % using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
            % also taking care of the antipodal unit quaternion ambiguity
            if qe(1)<0
                qe = qe*-1;
            end
            eq = 2. * q(2:4)';
            
            % calculate angular rates setpoint
            rate_setpoint = eq.*obj.kp_att;
            
            % Feed forward the yaw setpoint rate.
            % yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
            % but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
            % Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
            % and multiply it by the yaw setpoint rate (yawspeed_setpoint).
            % This yields a vector representing the commanded rotation around the world z-axis expressed in the body frame
            % such that it can be added to the rates setpoint.
            % yawspeed_setpoint = 0; % FOR NOW
            % rate_setpoint = rate_setpoint + q.inversed().dcm_z() * yawspeed_setpoint;
            
            %
            %   // limit rates
            %   for (int i = 0; i < 3; i++) {
            %     rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
            %   }
            %
        end

        function [q] = vecs2quat(obj, src, dst)
            eps = 1e-5;
            cr = cross(src, dst);
            dth = dot(src, dst);
            if (norm(cr) < eps && dth < 0)
                % // handle corner cases with 180 degree rotations
                % // if the two vectors are parallel, cross product is zero
                % // if they point opposite, the dot product is negative
                cr = abs(src);
                if (cr(0) < cr(1))
                    if (cr(0) < cr(2))
                        cr = [1, 0, 0]';
                    else
                        cr = [0, 0, 1]';
                    end
                else
                    if (cr(1) < cr(2))
                        cr = [0, 1, 0]';
                    else
                        cr = [0, 0, 1]';
                    end
                    q(1) = 0;
                    cr = cross(src, cr);
                end
            else
                % normal case, do half-way quaternion solution
                q(1) = dth + sqrt(norm(src)*norm(src)*norm(dst)*norm(dst));
            end
            q = [q, cr'];
        end
    end
end