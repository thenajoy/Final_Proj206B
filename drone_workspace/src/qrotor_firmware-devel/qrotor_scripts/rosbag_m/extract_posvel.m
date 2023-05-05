clear

filename = 'jan26/systems_check_filters_on_gyroaccel.bag';
bag = rosbag(filename);

bagselect = select(bag, 'Topic',  '/qrotor/t265/odometry');
odom_struct = readMessages(bagselect,'DataFormat','struct');

p = [cellfun(@(m) double(m.Pose.Pose.Position.X), odom_struct), ...
        cellfun(@(m) double(m.Pose.Pose.Position.Y), odom_struct),...
        cellfun(@(m) double(m.Pose.Pose.Position.Z), odom_struct)];
    
v = [cellfun(@(m) double(m.Twist.Twist.Linear.X), odom_struct), ...
        cellfun(@(m) double(m.Twist.Twist.Linear.Y), odom_struct),...
        cellfun(@(m) double(m.Twist.Twist.Linear.Z), odom_struct)];

tOdom = cellfun(@(m) double(m.Header.Stamp.Sec), odom_struct) + ...
        cellfun(@(m) double(m.Header.Stamp.Nsec), odom_struct)*1e-9;
t0 =tOdom(1);
tOdom = tOdom-t0;

    
save('pv_gnd_prop_hover.mat', 'tOdom', 'p', 'v')