# Installation

Install the following dependencies before proceeding to the rest of the package insatllation


<details>
  <summary>Dependencies (click to expand)</summary>

- Install Librealsense package from [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
  - `sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE`
  - `sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u`
  - `sudo apt-get install librealsense2-dkms`
  - `sudo apt-get install librealsense2-utils`
  - `sudo apt-get install librealsense2-dbg`
  - `sudo apt-get install librealsense2-dev`
- Install ros packages `dynamic_reconfigure` and `ddynamic_reconfigure`
  - `sudo apt-get install ros-<version>-dynamic-reconfigure`
  - `sudo apt-get install ros-<version>-ddynamic-reconfigure`
- You need to install `git`; If you are not familiar with git, either have [ssh enabled](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) or [generate a token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token)

</details>

#### Download 
Clone the repository and its submodules recursively

```
$ mkdir -p <path-to-ros-ws>/src
$ git clone https://github.com/HybridRobotics/qrotor_firmware.git
$ cd qrotor_firmware
$ git submodule update --init --recursive
```


#### Installation
```
$ cd <path-to-ros-ws>
$ catkin_make
```

:warning: ROS noetic can throw python error, try `catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3`

Next step: Refer [simulations](./simulation.md)

