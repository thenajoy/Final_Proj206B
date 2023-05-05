## qrotor_firmware

:warning: Use this repository only for quadrotor (and related) simulations, the associated **firmware** (qrotor_firmware/firmare) is deprecated and is no longer used in the hardware. However, this repo still provides close to real hardware-level simulation. 

> Refer [firmware](https://github.com/HybridRobotics/firmware), [qrotor_ros](https://github.com/HybridRobotics/qrotor_ros) and [qrotor_ros2](https://github.com/HybridRobotics/qrotor_ros2) packages for current hardware experiments and companion simulation-in-loop setup for the hardware. Contact Prasanth Kotaru (prasanth.kotaru@berkeley.edu) for access to these repositories

---

### Getting Started 

Refer [wiki](./_docs/README.md) 

---


<details>
  <summary>Firmware details (click to expand)</summary>

#### Control

| controller | status | remark | 
| :------ | :------ | :------------ | 
| `attitude_controller` | stable :heavy_check_mark: | Parent class for all attitude controllers |
| `euler_angle_ppid` | stable :heavy_check_mark: | Runs a PID on the rates, with rate setpoint from Euler angles |
| `euler_pid` | discontd. :no_entry: | PID on the Euler-angles directly |
| `attitude_control_px4` | discontd. :no_entry: | |
| `attitude_geometric_clf_qp` | wip :construction:  | |
| `attitude_vbl_lqr` | wip :construction: | |
| `attitude_geometric_control` | wip :construction:  | |
| | | |
| `position_controller` | stable :heavy_check_mark: | Parent class for all position controllers |
| `position_pid` | stable :heavy_check_mark: | PID position controller |
| `position_cbf_qp` | stable :heavy_check_mark: | Position Control Lyapunov Function based QP controller |
| `position_cbf_qp` | stable :heavy_check_mark: | Position Control Barrier Function based QP controller |
| `position_mpc` | stable :heavy_check_mark: | Linear Position MPC  |



</details>


---

## Dependencies

- [emlid/Navio2](https://github.com/emlid/Navio2)
- [PX4/Matrix](https://github.com/PX4/Matrix)
- [PX4/Firmware/mathlib](https://github.com/PX4/Firmware/tree/master/src/lib/mathlib)
- [IntelRealSense/librealsense](https://github.com/IntelRealSense/librealsense) (https://dev.intelrealsense.com/docs/using-depth-camera-with-raspberry-pi-3)
- ~~[lcm-proj/lcm](https://github.com/lcm-proj/lcm): Lightweight Communications and Marshalling~~


##### Disclaimer:
Code structure is inspired from multiple sources, primarily [rosflight-firmware](https://github.com/rosflight/firmware), [PX4-Frimware](https://github.com/PX4/Firmware)
