clear
folder = '/home/kotaru/workspace/catkin_wss/qrotor_ws/bags/experiments/';
file = 'jan26/imu_gnd_noprop.mat';

filename = strcat(folder,file);
dat = load(filename);
accel = dat.accel;

G_SI = 9.80665;
gyro_bias = mean(dat.gyro)
gyro = dat.gyro-gyro_bias;

accel_scaled = dat.accel/G_SI;    
accel_bias = mean(accel_scaled-[0,0,1])
accel = dat.accel-accel_bias*G_SI;
    
%% 
close all
figure; hold on;
for i = 1:3
   subplot(3,2,2*i-1); hold on;
   plot(dat.tImu, gyro(:,i));
end

for i = 1:3
   subplot(3,2,2*i); hold on;
   plot(dat.tImu, accel(:,i));
end

    
    