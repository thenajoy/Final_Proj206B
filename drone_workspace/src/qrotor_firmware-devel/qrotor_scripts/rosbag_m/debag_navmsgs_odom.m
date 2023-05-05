function [odom] = debag_navmsgs_odom(bag, topic)

bagselect = select(bag, 'Topic', topic);
pose_struct = readMessages(bagselect,'DataFormat','struct');

odom.quat = [cellfun(@(m) double(m.Pose.Pose.Orientation.W), pose_struct), ...
        cellfun(@(m) double(m.Pose.Pose.Orientation.X), pose_struct), ...
        cellfun(@(m) double(m.Pose.Pose.Orientation.Y), pose_struct),...
        cellfun(@(m) double(m.Pose.Pose.Orientation.Z), pose_struct)];

t = cellfun(@(m) double(m.Header.Stamp.Sec), pose_struct) ...
    + cellfun(@(m) double(m.Header.Stamp.Nsec), pose_struct)*1e-9 ;
odom.t0 = t(1);
odom.t = t-odom.t0;

odom.pos =  [cellfun(@(m) double(m.Pose.Pose.Position.X), pose_struct), ...
        cellfun(@(m) double(m.Pose.Pose.Position.Y), pose_struct),...
        cellfun(@(m) double(m.Pose.Pose.Position.Z), pose_struct)];

odom.vel =  [cellfun(@(m) double(m.Twist.Twist.Linear.X), pose_struct), ...
        cellfun(@(m) double(m.Twist.Twist.Linear.Y), pose_struct),...
        cellfun(@(m) double(m.Twist.Twist.Linear.Z), pose_struct)];
odom.ang_vel =  [cellfun(@(m) double(m.Twist.Twist.Angular.X), pose_struct), ...
        cellfun(@(m) double(m.Twist.Twist.Angular.Y), pose_struct),...
        cellfun(@(m) double(m.Twist.Twist.Angular.Z), pose_struct)];

N = length(odom.t);
Rot =  quat2rotm(odom.quat);
odom.Eul = zeros(N,3);
for i = 1:N
    odom.Eul(i,:) = Rot2RPY_ZXY(Rot(:,:,i))';
end


end