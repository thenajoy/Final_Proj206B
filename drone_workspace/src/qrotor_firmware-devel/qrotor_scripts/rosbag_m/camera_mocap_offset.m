% camera mocap offset
clear

% folder = '/home/kotaru/workspace/catkin_ws/qrotor_firmware_ws/bags/experiments/estimation/';
folder = '/home/kotaru/Workspace/catkin_ws/qrotor_ws/bags/';
file = 'camera_offset.bag';

filename = strcat(folder,file);
bag = rosbag(filename);


%%

bagselect = select(bag, 'Topic',  '/falcon/odometry/t265');
odom_struct = readMessages(bagselect,'DataFormat','struct');

bagselect = select(bag, 'Topic',  '/falcon/odometry/mocap');
mocap_struct = readMessages(bagselect,'DataFormat','struct');

p = [cellfun(@(m) double(m.Pose.Pose.Position.X), odom_struct), ...
        cellfun(@(m) double(m.Pose.Pose.Position.Y), odom_struct),...
        cellfun(@(m) double(m.Pose.Pose.Position.Z), odom_struct)];
    
v = [cellfun(@(m) double(m.Twist.Twist.Linear.X), odom_struct), ...
        cellfun(@(m) double(m.Twist.Twist.Linear.Y), odom_struct),...
        cellfun(@(m) double(m.Twist.Twist.Linear.Z), odom_struct)];
            
ang_vel_t265 = [cellfun(@(m) double(m.Twist.Twist.Angular.X), odom_struct), ...
        cellfun(@(m) double(m.Twist.Twist.Angular.Y), odom_struct),...
        cellfun(@(m) double(m.Twist.Twist.Angular.Z), odom_struct)];
    
q = [cellfun(@(m) double(m.Pose.Pose.Orientation.W), odom_struct), ...
    cellfun(@(m) double(m.Pose.Pose.Orientation.X), odom_struct), ...
        cellfun(@(m) double(m.Pose.Pose.Orientation.Y), odom_struct),...
        cellfun(@(m) double(m.Pose.Pose.Orientation.Z), odom_struct)];
R_t265 = quat2rotm(q);
N = length(q);
Eul_t265 = zeros(N,3);
for i = 1:N
    Eul_t265(i,:) = Rot2RPY_ZXY(R_t265(:,:,i))';
end

tOdom = cellfun(@(m) double(m.Header.Stamp.Sec), odom_struct) + ...
        cellfun(@(m) double(m.Header.Stamp.Nsec), odom_struct)*1e-9;
t0 =tOdom(1);
tOdom = tOdom-t0;

p_mocap = [cellfun(@(m) double(m.Pose.Pose.Position.X), mocap_struct), ...
        cellfun(@(m) double(m.Pose.Pose.Position.Y), mocap_struct),...
        cellfun(@(m) double(m.Pose.Pose.Position.Z), mocap_struct)];

v_mocap  = [cellfun(@(m) double(m.Twist.Twist.Linear.X), mocap_struct), ...
        cellfun(@(m) double(m.Twist.Twist.Linear.Y), mocap_struct),...
        cellfun(@(m) double(m.Twist.Twist.Linear.Z), mocap_struct)];

q_mocap  = [cellfun(@(m) double(m.Pose.Pose.Orientation.W), mocap_struct), ...
    cellfun(@(m) double(m.Pose.Pose.Orientation.X), mocap_struct), ...
        cellfun(@(m) double(m.Pose.Pose.Orientation.Y), mocap_struct),...
        cellfun(@(m) double(m.Pose.Pose.Orientation.Z), mocap_struct)];
R_mocap  = quat2rotm(q_mocap );
N = length(q_mocap );
Eul_mocap = zeros(N,3);
for i = 1:N
    Eul_mocap (i,:) = Rot2RPY_ZXY(R_mocap (:,:,i))';
end

t_mocap = cellfun(@(m) double(m.Header.Stamp.Sec), mocap_struct) + ...
        cellfun(@(m) double(m.Header.Stamp.Nsec), mocap_struct)*1e-9;
t_mocap = t_mocap-t0; 

%% computing position offset
pos_offset = zeros(1,3);
for j = 1:100
    pos_offset = pos_offset +  (p_mocap(j,:) - p(j,:));
end
pos_offset = pos_offset/100;

% pos_fixed = p+pos_offset;
N = length(p);
pos_fixed = zeros(N,3);
for i = 1:N
    indt = find(t_mocap>=tOdom(i),1);
    Rm = R_mocap(:,:,indt);
    pos_fixed(i,:) = p(i,:)+(Rm*pos_offset')';
end

%% computing rotation offset

angle_axis = zeros(3,1);
for j = 1:100
    % R0m = R0t*Roffset 
    Roffset = R_t265(:,:,j)'*R_mocap(:,:,j);
    angle_axis = angle_axis + vee(logm(Roffset));
end
angle_axis = angle_axis/100;
Roffset = expm(hat(angle_axis));

R_tcam_fixed = zeros(size(R_t265));
N = length(R_t265);
Eul_fixed = zeros(N,3);
for i = 1:N
   R_tcam_fixed(:,:,i) = R_t265(:,:,i)*Roffset;
   Eul_fixed(i,:) = Rot2RPY_ZXY(R_tcam_fixed (:,:,i))';
end
save('offset.mat','Roffset', 'pos_offset');

%%
close all
skip=1;
legnds = {'t265','mocap','t265-adjusted'};

figure('name', 'positions');
subplot(3,1,1);
hold on;
plot(tOdom(skip:end), p(skip:end,1), 'b', 'linewidth', 1);
plot(t_mocap(skip:end), p_mocap(skip:end,1), 'r', 'linewidth', 1);
plot(tOdom(skip:end), pos_fixed(skip:end,1), 'g', 'linewidth', 1);
latex_title('$$p_x$$');
latex_legend(legnds);

subplot(3,1,2);
hold on;
plot(tOdom(skip:end), p(skip:end,2), 'b', 'linewidth', 1);
plot(t_mocap(skip:end), p_mocap(skip:end,2), 'r', 'linewidth', 1);
plot(tOdom(skip:end), pos_fixed(skip:end,2), 'g', 'linewidth', 1);
latex_title('$$p_y$$');
latex_legend(legnds);

subplot(3,1,3);
hold on;
plot(tOdom(skip:end), p(skip:end,3), 'b', 'linewidth', 1);
plot(t_mocap(skip:end), p_mocap(skip:end,3), 'r', 'linewidth', 1);
plot(tOdom(skip:end), pos_fixed(skip:end,3), 'g', 'linewidth', 1);
latex_title('$$p_z$$');
latex_legend(legnds);




lngds_att = {'mocap', 't265', 't265-fixed'};
figure('name', 'attitude');
subplot(3,1,1); hold on;
l = plot(t_mocap, Eul_mocap(:,1)*180/pi, 'b', 'linewidth', 2);
plot(tOdom, Eul_t265(:,1)*180/pi, 'r');
plot(tOdom, Eul_fixed(:,1)*180/pi, 'g'); 
latex_title('roll: $$\phi$$'); 
latex_legend(lngds_att);

subplot(3,1,2); hold on;
l =plot(t_mocap, Eul_mocap(:,2)*180/pi, 'b', 'linewidth', 2);
plot(tOdom, Eul_t265(:,2)*180/pi, 'r'); 
plot(tOdom, Eul_fixed(:,2)*180/pi, 'g'); 
latex_title('pitch: $$\theta$$'); 
latex_legend(lngds_att);

subplot(3,1,3); hold on;
l =plot(t_mocap, Eul_mocap(:,3)*180/pi, 'b', 'linewidth', 2);
plot(tOdom, Eul_t265(:,3)*180/pi, 'r'); 
plot(tOdom, Eul_fixed(:,3)*180/pi, 'g'); 
latex_title('yaw: $$\psi$$'); 
latex_legend(lngds_att);
latex_xlabel('Time[s]');
latex_ylabel('[deg]');



























