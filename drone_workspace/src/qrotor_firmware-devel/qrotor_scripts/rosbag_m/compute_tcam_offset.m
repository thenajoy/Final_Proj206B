clear

folder = '/home/kotaru/Workspace/catkin_ws/qrotor_ws/bags/';
file = 'camera_offset.bag';

filename = strcat(folder,file);
bag = rosbag(filename);

%%

pose = DebagQrotor.readNavmsgsOdometry(bag, '/t265/odometry');
R = quat2rotm(pose.quat);
N = size(R,3)

angle_axis = zeros(3,N);
for i = 1:N
    angle_axis(:,i) = vee(logm(R(:,:,i)));
end

aa = mean(angle_axis,2)
Rmean = expm(hat(aa))

eul = rad2deg(Rot2RPY_ZXY(Rmean))
