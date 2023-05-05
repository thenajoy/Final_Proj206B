
### Setting up different ROS IP

1. Use `ifconfig` in terminal to findout about the ip address of the system, its usually `192.168.**.**` 
2. Add the following to `~/.bashrc` and then `source ~/.bashrc`
   ```
   export ROS_MASTER_URI=http://<ROS MASTER IP ADDRESS HERE>:11311
   export ROS_IP=<LAPTOPS IP ADDRESS HERE>
   export ROS_HOSTNAME=<LAPTOPS ip ADDRESS HERE>
   ``` 
   For instance 
   ```
   export ROS_MASTER_URI=http://192.168.1.15:11311
   export ROS_IP=192.168.1.15
   export ROS_HOSTNAME=192.168.1.15
   ``` 
    


