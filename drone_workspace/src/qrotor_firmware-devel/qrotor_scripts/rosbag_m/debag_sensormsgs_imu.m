function [imu] = debag_sensormsgs_imu(bag, topic)


bagselect = select(bag, 'Topic',  topic);
imu_struct = readMessages(bagselect,'DataFormat','struct');

%%

imu.gyro = [cellfun(@(m) double(m.AngularVelocity.X), imu_struct), ...
        cellfun(@(m) double(m.AngularVelocity.Y), imu_struct),...
        cellfun(@(m) double(m.AngularVelocity.Z), imu_struct)];

imu.time = cellfun(@(m) double(m.Header.Stamp.Sec), imu_struct) + ...
        cellfun(@(m) double(m.Header.Stamp.Nsec), imu_struct)*1e-9;
imu.t0 = imu.time(1);
imu.time = imu.time-imu.t0;

imu.accel = [cellfun(@(m) double(m.LinearAcceleration.X), imu_struct), ...
        cellfun(@(m) double(m.LinearAcceleration.Y), imu_struct),...
        cellfun(@(m) double(m.LinearAcceleration.Z), imu_struct)];
    
end