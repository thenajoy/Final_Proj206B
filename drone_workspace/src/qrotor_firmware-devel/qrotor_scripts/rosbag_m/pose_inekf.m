clear;

load('data.mat');
load('offset.mat');
addpath('../inv_ekf_m');
addpath(genpath('../inv_ekf_m/codegen/mex/time_update/'));

%% pose estimation
accel_f = apply_lpf_so(imu.time, imu.accel, 500, 60);
gyro_f = apply_lpf_so(imu.time, imu.gyro, 500, 60);


%%

N = length(datum);
M = N;

state = struct();
state.pos = zeros(M,3);
state.vel = zeros(M,3);
state.rot = zeros(3,3,M);
state.Eul = zeros(M,3);

%% intialize

state.pos(1,:) = rb_mocap.pos(1,:);
state.vel(1,:) = zeros(1,3);
state.rot(:,:,1) = quat2rotm(rb_mocap.quat(1,:));
state.Eul(1,:) = Rot2RPY_ZXY(state.rot(:,:,1))';

imu_last_update_s = -1;
imu_curr = 0;
covar = 0.1*eye(9);

R = state.rot(:,:,1);
v = state.vel(1,:)';
p = state.pos(1,:)';

% t_hat = [];
% p_hat = [];
% v_hat = [];
% Eul_hat = [];
tic
for i = 1:M
    
    % time-update - imu-model propagation
    % -----------------------------------
    if datum(i,2)== 0
        if imu_last_update_s == -1
            imu_last_update_s = datum(i,1);
        else
            imu_now_s = datum(i,1);
            dt = imu_now_s-imu_last_update_s;
            imu_last_update_s = imu_now_s;
            
            id = datum(i,3);
%             accel = imu.accel(id,:)';
%             gyro = imu.gyro(id,:)';
            accel = accel_f(id, :)';
            gyro = gyro_f(id, :)';
            
            [R, v, p, covar] = time_update(dt, R, v, p, covar, accel, gyro);               
        end 
    
    % meas-update - tracking camera
    % -----------------------------
    elseif (datum(i,2) == 1)
        id = datum(i,3);
        Rm = quat2rotm(rb_tcam.quat(id,:))*Roffset;
        vm = rb_tcam.vel(id,:)';
        pm = rb_tcam.pos(id,:)'+pos_offset';
        [R, v, p, covar] = meas_update_tcam(R, v, p, Rm, vm, pm, covar);  
    
%         t_hat = [t_hat; datum(i,1)];
%         p_hat = [p_hat; p'];
%         v_hat = [v_hat; v'];
%         Eul_hat = [Eul_hat; Rot2RPY_ZXY(R)'];
    
    % meas-update - mocap
    % -----------------------------
    elseif (datum(i,2) == 2)
        id = datum(i,3);
        Rm = quat2rotm(rb_mocap.quat(id,:));
        vm = rb_mocap.vel(id,:)';
        pm = rb_mocap.pos(id,:)';
        [R, v, p, covar] = meas_update_mocap(R, v, p, Rm, vm, pm, covar);   
    
%         t_hat = [t_hat; datum(i,1)];
%         p_hat = [p_hat; p'];
%         v_hat = [v_hat; v'];
%         Eul_hat = [Eul_hat; Rot2RPY_ZXY(R)'];
         
    end
    
    
    % storage
    state.pos(i,:) = p';
    state.vel(i,:) = v';
    state.rot(:,:,i) = R;
    state.Eul(i,:) = Rot2RPY_ZXY(R)';   
    
end
toc



%%
close all
% 
% figure; hold on
% plot(imu.time,zeros(length(imu.time),1),'x')
% plot(rb_mocap.t,zeros(length(rb_mocap.t),1), 's')
% plot(rb_tcam.t,zeros(length(rb_tcam.t),1), 'd')

% latex_legend({'imu','mocap','tcam'});

% axis_chars = {'x', 'y', 'z'};
% figure;
% for i = 1:3
%     subplot(3,1,i); hold on;
%     plot(imu.time, imu.accel(:,i), 'linewidth', 1); 
%     plot(imu.time, accel_f(:,i), 'linewidth', 2); 
%     grid on; grid minor
%     latex_title(strcat('$$a_',axis_chars{i},'$$'));
% end
% 
% figure;
% for i = 1:3
%     subplot(3,1,i); hold on;
%     plot(imu.time, imu.gyro(:,i), 'linewidth', 1); 
%     plot(imu.time, gyro_f(:,i), 'linewidth', 2); 
%     grid on; grid minor
%     latex_title(strcat('$$g_',axis_chars{i},'$$'));
% end



legnds = {'t265', 'mocap', 'est'};

% 
figure('name', 'positions');
subplot(3,1,1);
hold on;
plot(tcam_t, rb_tcam.pos(:,1), 'b', 'linewidth', 1);
plot(tmocap_t, rb_mocap.pos(:,1), 'r', 'linewidth', 1);
plot(datum(1:M,1), state.pos(1:M,1), 'g', 'linewidth', 1);
% plot(t_hat, p_hat(:,1), 'k', 'linewidth', 1);
latex_title('$$p_x$$');
latex_legend(legnds);

subplot(3,1,2);
hold on;
plot(tcam_t, rb_tcam.pos(:,2), 'b', 'linewidth', 1);
plot(tmocap_t, rb_mocap.pos(:,2), 'r', 'linewidth', 1);
plot(datum(1:M,1), state.pos(1:M,2), 'g', 'linewidth', 1);
% plot(t_hat, p_hat(:,2), 'k', 'linewidth', 1);
latex_title('$$p_y$$');
latex_legend(legnds);

subplot(3,1,3);
hold on;
plot(tcam_t, rb_tcam.pos(:,3), 'b', 'linewidth', 1);
plot(tmocap_t, rb_mocap.pos(:,3), 'r', 'linewidth', 1);
plot(datum(1:M,1), state.pos(1:M,3), 'g', 'linewidth', 1);
% plot(t_hat, p_hat(:,3), 'k', 'linewidth', 1);
latex_title('$$p_z$$');
latex_legend(legnds);

% 
figure('name', 'velocities');
subplot(3,1,1);
hold on;
plot(tcam_t, rb_tcam.vel(:,1), 'b', 'linewidth', 1);
plot(tmocap_t, rb_mocap.vel(:,1), 'r', 'linewidth', 1);
plot(datum(1:M,1), state.vel(1:M,1), 'g', 'linewidth', 1);
latex_title('$$v_x$$');
latex_legend(legnds);

subplot(3,1,2);
hold on;
plot(tcam_t, rb_tcam.vel(:,2), 'b', 'linewidth', 1);
plot(tmocap_t, rb_mocap.vel(:,2), 'r', 'linewidth', 1);
plot(datum(1:M,1), state.vel(1:M,2), 'g', 'linewidth', 1);
latex_title('$$v_y$$');
latex_legend(legnds);

subplot(3,1,3);
hold on;
plot(tcam_t, rb_tcam.vel(:,3), 'b', 'linewidth', 1);
plot(tmocap_t, rb_mocap.vel(:,3), 'r', 'linewidth', 1);
plot(datum(1:M,1), state.vel(1:M,3), 'g', 'linewidth', 1);
latex_title('$$v_z$$');
latex_legend(legnds);

%
figure('name', 'attitude');
subplot(3,1,1); hold on;
plot(tcam_t, rb_tcam.Eul(:,1)*180/pi, 'b', 'linewidth', 1); 
plot(tmocap_t, rb_mocap.Eul(:,1)*180/pi, 'r', 'linewidth', 1);
plot(datum(1:M,1),state.Eul(1:M,1)*180/pi,'g', 'linewidth', 1);
latex_title('roll: $$\phi$$'); 
latex_legend(legnds);

subplot(3,1,2); hold on;
plot(tcam_t, rb_tcam.Eul(:,2)*180/pi, 'b', 'linewidth', 1); 
plot(tmocap_t, rb_mocap.Eul(:,2)*180/pi, 'r', 'linewidth', 1);
plot(datum(1:M,1),state.Eul(1:M,2)*180/pi,'g', 'linewidth', 1);
latex_title('pitch: $$\theta$$'); 
latex_legend(legnds);

subplot(3,1,3); hold on;
plot(tcam_t, rb_tcam.Eul(:,3)*180/pi, 'b', 'linewidth', 1); 
plot(tmocap_t, rb_mocap.Eul(:,3)*180/pi, 'r', 'linewidth', 1);
plot(datum(1:M,1),state.Eul(1:M,3)*180/pi,'g', 'linewidth', 1);
latex_title('yaw: $$\psi$$'); 
latex_legend({'onboard-used-in-ctrl', 't265'});
latex_xlabel('Time[s]');
latex_ylabel('[deg]');


