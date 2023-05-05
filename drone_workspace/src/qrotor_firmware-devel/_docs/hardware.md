### Setting up the ground station (laptop)
1. Download and compile https://github.com/HybridRobotics/vrpn_client_ros in your catkin workspace
2. Ensure ip address for ROS are setup in `~/.bashrc`
   ```
   export ROS_MASTER_URI=http://<GROUND_STATION_IP_ADDRESS>:11311/
   export ROS_IP=<GROUND_STATION_IP_ADDRESS>
   ```
   Ip address can be found in the terminal using `hostname -I`
3. Source the bashrc and the catkin workspace
   ```
    source ~/.bashrc
    source devel/setup.bash
4. Launch the rigid-body tracker node from `vrpn_client_ros` to get the pose information from the mocap software; Ensure the IP address to the Mocap desktop is set in `track_rigidbodies.launch`
   ```
   roslaunch vrpn_client_ros track_rigidbodies.launch
   ```



### Connecting to the drone
1. Connect your laptop to lab router (`HRLab_5G`, password `differentialflatness#2018`) or the flight space router (`Mueller_Lab 5G`, password `crazyflie2016`)
2. SSH from your laptop to the raspberry pi on the drone using 
   ```
   ssh pi@whitefalcon.local
   ssh pi@redfalcon.local
   ssh pi@bluefalcon.local
   ```
   and username is `pi` and password is `raspberry` 
   
   
### Compiling the packages on the drone  (first time setup)
1. Download the `qrotor_firmware` package to the drone (if its not already downloaded)
2. Download `ddynamic_reconfigure` (https://github.com/pal-robotics/ddynamic_reconfigure) if its not already donwloaded to the workspace
3. Compile only `qrotor_firmware` package and from the qrotor_firmware folder else
   ```
   catkin_make -DCATKIN_BLACKLIST_PACKAGES='qrotor_gazebo;qrotor_ground;qrotor_description' -j2
   ```
4. Compile the packages

### Launching the firmware on the drone
1. Enter root access, `sudo su`
2. open `~/.bashrc` on the raspberry pi using `nano ~/.bashrc` and confirm the right IP addresses for the ROS setup
   ```
   export ROS_MASTER_URI=http://<<GROUND_STATION_IP_ADDRESS>>:11311/
   export ROS_IP=<PI_IP_ADDRESS>
   export ROS_HOSTNAME=<PI_IP_ADDRESS>
   ```
   Ip address can be found in the terminal using `hostname -I`
3. Source the bashrc to ensure the IP addresses are set using `source ~/.bashrc`
4. Source the ros packages in the catkin workspcae as `source devel/setup.bash`
5. Launch the drone in root using 
   ```
   roslaunch qrotor_firmware falcon.launch bird:=<vehicle type>
   ```
   vehicle type can be `blue`, `red`, `white`
   
#### Updating parameter using `dynamic_reconfigure`
1. Open `rqt_reconfigure`
   ```
   rosrun rqt_reconfigure rqt_reconfigure
   ```
2. In `EXTERNAL_POSE` value to `0 -tracking, 1-motion capture, 2- fusion of both`
3. If required switch to OFFBOARD mode as shown in [simulation instructions](https://github.com/HybridRobotics/qrotor_firmware/blob/devel/_docs/simulation.md)
