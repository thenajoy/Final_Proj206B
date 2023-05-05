classdef DebagQrotor
properties
    
end
%%
methods
   function obj = DebagQrotor()
   
   end   
end

methods (Static)
    %%
    function plotPowerReadings(t,V,C)
        P= V.*C;
        N= length(t);
        
        alpha = 0.005;
        vol = zeros(N,1); cur = zeros(N,1); power= zeros(N,1);
        vol(1) = V(1); cur(1) = C(1); power(1)=P(1);
        for j = 2:N
             cur(j)= cur(j-1)*(1-alpha)+alpha*C(j);
             vol(j) = vol(j-1)*(1-alpha)+alpha*V(j);
             power(j) = power(j-1)*(1-alpha) + alpha*P(j);
        end
        figure('name', 'power');
        subplot(3,1,1); hold on;
        plot(t, V, 'b', 'linewidth', 1);
        plot(t, vol, 'r','linewidth', 2);
        grid on, grid minor;
        latex_title('Battery-Voltage $$V$$');
        latex_ylabel('[volts]');

        subplot(3,1,2); hold on;
        plot(t, C, 'b','linewidth', 1);
        plot(t, cur, 'r','linewidth', 2);
        grid on, grid minor;
        latex_title('Current-Drawn $$i$$');
        latex_ylabel('[amps]');
        latex_xlabel('Time[s]');

        subplot(3,1,3); hold on;
        plot(t, P, 'b','linewidth', 1);
        plot(t, power, 'r','linewidth', 2);
        grid on, grid minor;
        latex_title('Power $$V*i$$');
        latex_ylabel('W?');
        latex_xlabel('Time[s]');   
    end
   %%
    function [odom] = readNavmsgsOdometry(bag, topic)
       bagselect = select(bag, 'Topic', topic);
        pose_struct = readMessages(bagselect,'DataFormat','struct');

        odom.quat = [cellfun(@(m) double(m.Pose.Pose.Orientation.W), pose_struct), ...
                cellfun(@(m) double(m.Pose.Pose.Orientation.X), pose_struct), ...
                cellfun(@(m) double(m.Pose.Pose.Orientation.Y), pose_struct),...
                cellfun(@(m) double(m.Pose.Pose.Orientation.Z), pose_struct)];

        time = cellfun(@(m) double(m.Header.Stamp.Sec), pose_struct) ...
            + cellfun(@(m) double(m.Header.Stamp.Nsec), pose_struct)*1e-9 ;
        odom.t0 = time(1);
        odom.time = time-odom.t0;

        odom.pos =  [cellfun(@(m) double(m.Pose.Pose.Position.X), pose_struct), ...
                cellfun(@(m) double(m.Pose.Pose.Position.Y), pose_struct),...
                cellfun(@(m) double(m.Pose.Pose.Position.Z), pose_struct)];

        odom.vel =  [cellfun(@(m) double(m.Twist.Twist.Linear.X), pose_struct), ...
                cellfun(@(m) double(m.Twist.Twist.Linear.Y), pose_struct),...
                cellfun(@(m) double(m.Twist.Twist.Linear.Z), pose_struct)];
        odom.ang_vel =  [cellfun(@(m) double(m.Twist.Twist.Angular.X), pose_struct), ...
                cellfun(@(m) double(m.Twist.Twist.Angular.Y), pose_struct),...
                cellfun(@(m) double(m.Twist.Twist.Angular.Z), pose_struct)];

        N = length(odom.time);
        Rot =  quat2rotm(odom.quat);
        odom.Eul = zeros(N,3);
        for i = 1:N
            odom.Eul(i,:) = rad2deg(Rot2RPY_ZXY(Rot(:,:,i))');
        end 
    end
    %%
    function [imu] = readSensormsgsImu(bag, topic)
        bagselect = select(bag, 'Topic',  topic);
        imu_struct = readMessages(bagselect,'DataFormat','struct');
        imu.Gyroscope = [cellfun(@(m) double(m.AngularVelocity.X), imu_struct), ...
                cellfun(@(m) double(m.AngularVelocity.Y), imu_struct),...
                cellfun(@(m) double(m.AngularVelocity.Z), imu_struct)];

        imu.time = cellfun(@(m) double(m.Header.Stamp.Sec), imu_struct) + ...
                cellfun(@(m) double(m.Header.Stamp.Nsec), imu_struct)*1e-9;
        imu.t0 = imu.time(1);
        imu.time = imu.time-imu.t0;

        imu.Accelerometer = [cellfun(@(m) double(m.LinearAcceleration.X), imu_struct), ...
                cellfun(@(m) double(m.LinearAcceleration.Y), imu_struct),...
                cellfun(@(m) double(m.LinearAcceleration.Z), imu_struct)]/9.81; 
    end
    %%
    function [log] = readQrotormsgsLog(bag, topic)
        bagselect = select(bag, 'Topic',  topic);
        log_struct = readMessages(bagselect,'DataFormat','struct');
        
        log.time = cellfun(@(m) double(m.Header.Stamp.Sec), log_struct) ...
            + cellfun(@(m) double(m.Header.Stamp.Nsec), log_struct)*1e-9;
        log.t0 = log.time(1);
        log.time = log.time-log.t0;
        log.Eul = rad2deg([cellfun(@(m) double(m.Euler.X), log_struct), ...
                cellfun(@(m) double(m.Euler.Y), log_struct),...
                cellfun(@(m) double(m.Euler.Z), log_struct)]);
            
        log.LoopRate = cellfun(@(m) double(m.LoopRate), log_struct);
        log.PWM = [cellfun(@(m) m.EscInUs(1), log_struct),...
            cellfun(@(m) m.EscInUs(2), log_struct),...
            cellfun(@(m) m.EscInUs(3), log_struct),...
            cellfun(@(m) m.EscInUs(4), log_struct)];

        log.body_rates = [cellfun(@(m) m.BodyRates.X, log_struct),...
            cellfun(@(m) m.BodyRates.Y, log_struct),...
            cellfun(@(m) m.BodyRates.Z, log_struct)];

        log.lin_accel = [cellfun(@(m) m.LinearAcceleration.X, log_struct),...
            cellfun(@(m) m.LinearAcceleration.Y, log_struct),...
            cellfun(@(m) m.LinearAcceleration.Z, log_struct)];

        log.voltage = cellfun(@(m) m.Voltage, log_struct);
        log.current = cellfun(@(m) m.Current, log_struct);
        log.power = log.voltage.*log.current;
        
        log.ctrl_mode = cellfun(@(m) m.AttitudeMode, log_struct);
        
        all_pos_mode_ind = find(log.ctrl_mode==3);
        break_points = find(diff(all_pos_mode_ind)>1);
        L = length(break_points);
        indices = zeros(L,2);
        for i = 1:L
            if i==1
                indices(i,:) = [all_pos_mode_ind(1), all_pos_mode_ind(break_points(i))];
            elseif i==L
                indices(i,:) =[all_pos_mode_ind( break_points(L)), all_pos_mode_ind(end)];
            else
               indices(i,:) =[all_pos_mode_ind(break_points(i-1)+1), all_pos_mode_ind(break_points(i))]; 
            end
        end
        log.pos_ctrl_ind = indices;
        log.pos_ctrl_times = zeros(L,2);
        for i = 1:L
            log.pos_ctrl_times(i,:) = [log.time(indices(i,1)),log.time(indices(i,2))];
        end
        
        log.moment = [cellfun(@(m) m.Moment.X, log_struct),...
                        cellfun(@(m) m.Moment.Y, log_struct),...
                        cellfun(@(m) m.Moment.Z, log_struct)];
        log.thrust = cellfun(@(m) m.Thrust, log_struct);
        log.Lyap = cellfun(@(m) m.Lyapunov, log_struct);

    end
end

end