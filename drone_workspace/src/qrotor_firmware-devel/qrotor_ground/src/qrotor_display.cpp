// qrotor_hil_interface
#include "qrotor_ground/qrotor_display.h"

namespace qrotor_ground {

DisplayInterface::DisplayInterface(ros::NodeHandle &nh,
                                   std::string _vehicle_name)
    : nh_(nh) {

  cx = 0;
  cy = 0;
  position_ = Eigen::Vector3d::Zero();
  velocity_ = Eigen::Vector3d::Zero();
  euler_ = Eigen::Vector3d::Zero(); // 180.0, 45.5, -180.0;
  body_rates_ = Eigen::Vector3d::Zero();
  cmd_thrust_ = Eigen::Vector3d::Zero();
  load_position_ = Eigen::Vector3d::Zero();
  cmd_load_position_ = Eigen::Vector3d::Zero();
  cable_length = 1.0;

  thrust_ = 0.0;
  moment_ << 0.0, 0.0, 0.0;

  onboard_loop_freq_ = 0.0;
  voltage = 0.0;
  voltageString = "";
  mode = 0;
  // Colors
  // ------
  RED = "31";
  GREEN = "32";
  BLUE = "94";
  YELLOW = "93";

  //  _sub_qrotor_log =
  //      nh_.subscribe("/qrotor_onboard/log", 3,
  //                    &DisplayInterface::callback_sub_qrotor_log, this);
  //  _sub_cmd_traj = nh_.subscribe("/qrotor/cmd_traj", 3,
  //                                &DisplayInterface::callback_sub_cmd_traj,
  //                                this);
  //  _sub_qrotor_odom = nh_.subscribe("/qrotor_realsense/odom", 3,
  //                                   &DisplayInterface::callback_sub_odom,
  //                                   this);
  //  _sub_qrotor_cmd_input =
  //      nh_.subscribe("/qrotor_offboard/cmd_input", 3,
  //                    &DisplayInterface::callback_sub_cmd_input, this);

  // drone name
  quadNameString = "\033[1;";
  quadNameString.append(GREEN);
  quadNameString.append("m");
  quadNameString.append(_vehicle_name);
  quadNameString.append("\033[0m");
  _vehicle_name.append("/pose_vel");
  // std::cout <<
  //  _sub_qrotor_posevel = nh_.subscribe(
  //      _vehicle_name, 3, &DisplayInterface::callback_sub_qrotor_posevel,
  //      this);

  // ros params
  std::string vehicle_name, vehicle_type, load_name, vtype;
  nh.param<std::string>("/vehicle/type", vehicle_type, "Quadrotor");
  // vtype = "Quadrotorload";
  if (vehicle_type == "Quadrotorload") {
    nh.param<std::string>("/load/name", load_name, "load1");
    nh.param<double>("/load/cable", cable_length, 1.0);

    // load name
    loadNameString = "\033[1;";
    loadNameString.append(GREEN);
    loadNameString.append("m");
    loadNameString.append(load_name);
    loadNameString.append("\033[0m");
    load_name.append("/pose_vel");
//    _sub_load_posevel = nh_.subscribe(
//        load_name, 3, &DisplayInterface::callback_sub_load_posevel, this);

    is_load_suspended = true;
  }

  state_time_ = ros::Time::now().toSec();
}

DisplayInterface::~DisplayInterface() {}

/**************** callback functions ***************/
// void DisplayInterface::callback_sub_qrotor_log(
//     const qrotor_msgs::QuadrotorLog::ConstPtr &msg) {
//
//   // position_ 	<< msg->position.x, msg->position.y, msg->position.z;
//   // velocity_ 	<< msg->velocity.x, msg->velocity.y, msg->velocity.z;
//   thrust_ = msg->thrust;
//   moment_ << msg->moment.x, msg->moment.y, msg->moment.z;
//
//   euler_ << msg->euler.x * 180 / 3.1415, msg->euler.y * 180 / 3.1415,
//       msg->euler.z * 180 / 3.1415;
//   body_rates_ << msg->body_rates.x, msg->body_rates.y, msg->body_rates.z;
//
//   onboard_loop_freq_ = msg->loop_rate;
//   voltage = msg->voltage;
// }

// void DisplayInterface::callback_sub_qrotor_posevel(
//     const vrpn_client_ros::PoseVelStamped::ConstPtr &msg) {
//
//   // printf("POSE VEL RECEIVED\n");
//   position_ << msg->position.x, msg->position.y, msg->position.z;
//   velocity_ << msg->velocity.x, msg->velocity.y, msg->velocity.z;
// }
//
// void DisplayInterface::callback_sub_load_posevel(
//     const vrpn_client_ros::PoseVelStamped::ConstPtr &msg) {
//
//   // printf("POSE VEL RECEIVED\n");
//   load_position_ << msg->position.x, msg->position.y, msg->position.z;
//   load_velocity_ << msg->velocity.x, msg->velocity.y, msg->velocity.z;
// }

// void DisplayInterface::callback_sub_odom(
//     const nav_msgs::Odometry::ConstPtr &msg) {
//
//   // printf("POSE VEL RECEIVED\n");
//   position_ << msg->pose.pose.position.x, msg->pose.pose.position.y,
//       msg->pose.pose.position.z;
//   velocity_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
//       msg->twist.twist.linear.z;
// }

// void DisplayInterface::callback_sub_cmd_traj(
//     const qrotor_msgs::CommandTrajectory::ConstPtr &msg) {
//   mode = msg->mode;
//   cmd_position_ << msg->position.x, msg->position.y, msg->position.z;
//   traj_t_ = msg->traj_t;
//   Eigen::Vector3d q;
//   q << msg->q.x, msg->q.y, msg->q.z;
//   cmd_load_position_ = cmd_position_ + 0.58 * q;
//   // printf("%d\n",mode);
// }

// void DisplayInterface::callback_sub_cmd_input(
//     const qrotor_msgs::CommandThrustVectorMomentff::ConstPtr &msg) {
//   cmd_thrust_ << msg->thrust_vector.x, msg->thrust_vector.y,
//       msg->thrust_vector.z;
// }

/*********************************/
/*********************************/
/*********************************/
void DisplayInterface::addToLine(std::string c_) {
  str.append(c);
  cx = cx + c_.length();
}

void DisplayInterface::addNextLine() {
  str.append("\n");
  cx = 0;
  cy++;
}

void DisplayInterface::initialize() {

  for (int j = 0; j < height_; ++j) {
    for (int i = 0; i < width_; ++i) {
      if (j == 0 || j == height_ - 1) {
        c = "-";
        addToLine(c);
      } else if (i == 0 || i == width_ - 1) {
        c = "|";
        addToLine(c);
      } else if (j > height_ / 2) {
        c = ".";
        addToLine(c);
      } else {
        c = " ";
        addToLine(c);
      }
    }
    addNextLine();
  }
}

void DisplayInterface::displayScreen() { system("clear"); }

void DisplayInterface::updateTime() {
  current_time_ = ros::Time::now().toSec();
  dt_ = current_time_ - prev_time_;
  prev_time_ = current_time_;
  t_ = current_time_ - state_time_;
}

void DisplayInterface::update() {

  isArmed = "";
  isArmed.append("\033[1;32m");
  isArmed.append("bold red text");
  isArmed.append("\033[0m");

  // voltage reading
  voltageString = "\033[1;";
  if (voltage < 10) {
    voltageString.append(RED);
  } else {
    voltageString.append(GREEN);
  }
  voltageString.append("m");
  voltageString.append(std::to_string(voltage));
  voltageString.append("\033[0m");

  // mode reading
  modeString = "\033[1;";
  modeString.append(BLUE);
  modeString.append("m");
  switch (mode) {
  case 0:
    modeString.append("LAND");
    break;
  case 1:
    modeString.append("TAKE OFF");
    break;
  case 2:
    modeString.append("POSITION HOLD");
    break;
  case 3:
    modeString.append("TRAJECTORY TRACKING");
    break;
  case 4:
    modeString.append("SLOW STOP");
    break;
  case 5:
    modeString.append("PLANNER TRAJECTORY TRACKING");
    break;
  case 6:
    modeString.append("QROTORLOAD_SETPOINT");
    break;
  case 7:
    modeString.append("QROTORLOAD_TRAJ");
    break;
  case 1000:
    modeString.append("FORCE LAND");
    break;

  default:
    break;
  }
  modeString.append("\033[0m");

  DisplayInterface::updateTime();
  printf("|--------------------------------------------------------------------"
         "-------------------|\n");
  printf("| display-freq [Hz]: 0%3.2f \t | t(elapsed)[s]: 0%4.2f\n", (1 / dt_),
         t_);
  printf("| onboard-freq [Hz]: %.2f\t | battery-voltage [V]: %s\n",
         onboard_loop_freq_, voltageString.c_str());
  printf("|--------------------------------------------------------------------"
         "-------------------|\n");
  printf("| \033[1;93mQUADROTOR\033[0m: %s\n", quadNameString.c_str());
  printf("| cmd pos [m] \t\t> x: %.4f\t| y: %.4f\t| z: %.4f\t\t\t|\n",
         cmd_position_[0], cmd_position_[1], cmd_position_[2]);
  printf("| position [m] \t\t> x: \033[1;93m%.4f\033[0m\t| y: "
         "\033[1;93m%.4f\033[0m\t| z: \033[1;93m%.4f\033[0m\t\t\t|\n",
         position_[0], position_[1], position_[2]);
  printf("| velocity [m/s] \t> vx: %.4f\t| vy: %.4f\t| vz: %.4f\t\t\t|\n",
         velocity_[0], velocity_[1], velocity_[2]);
  printf("| euler [deg] \t\t> roll: %.2f\t| pitch: %.2f\t| yaw: %.2f\t\t\t|\n",
         euler_[0], euler_[1], euler_[2]);
  printf("| body rates [rad/s]  \t> gx: %.4f\t| gy: %.4f\t| gz: %.4f\t\t\t|\n",
         body_rates_[0], body_rates_[1], body_rates_[2]);
  printf("| \n");
  printf("| f [N]: %.4f\t | Mx [Nm]: %.4f\t | My [Nm]: %.4f\t | Mz [Nm]:%.4f\n",
         thrust_, moment_[0], moment_[1], moment_[2]);
  printf("| \n");
  printf("| speed: %.4f\n", velocity_.norm());
  std::cout << "|--------------------------------------------------------------"
               "-------------------------|"
            << std::endl;
  if (is_load_suspended) {
    printf("| \033[1;93mLoad\033[0m: %s\n", loadNameString.c_str());
    printf("| cmd pos [m] \t\t> x: %.4f\t| y: %.4f\t| z: %.4f\t\t\t|\n",
           cmd_load_position_[0], cmd_load_position_[1], cmd_load_position_[2]);
    printf("| position [m] \t\t> x: \033[1;93m%.4f\033[0m\t| y: "
           "\033[1;93m%.4f\033[0m\t| z: \033[1;93m%.4f\033[0m\t\t\t|\n",
           load_position_[0], load_position_[1], load_position_[2]);
    std::cout << "|------------------------------------------------------------"
                 "---------------------------|"
              << std::endl;
  }
  // printf("| status:\t %s \n", isArmed.c_str());
  // std::cout << "| mode  :\t" << "TRAJ" << "\t\t" << "remark:" << "\t" <<
  // "circular"  << std::endl;
  // std::cout <<
  // "|---------------------------------------------------------------------------------------|"
  // << std::endl;
  printf("| Status: %s\n", modeString.c_str());
  printf("| traj_t = %.2f\n", traj_t_);
  printf("| cmd thrust [m] \t> x: %.4f\t| y: %.4f\t| z: %.4f\t\t\t|\n",
         cmd_thrust_[0], cmd_thrust_[1], cmd_thrust_[2]);
  std::cout << "|--------------------------------------------------------------"
               "-------------------------|"
            << std::endl;

  // printf("|---------------------------------------------------------------------------------------|\n");
  // printf("| freqency [Hz]: %02.2f \t\t  | t (Elapsed)[s] %6.2f\n",  (1/dt_),
  // t_);
  // printf("|---------------------------------------------------------------------------------------|\n");
  // std::cout << "| position [m] \t\t> x: " << std::to_string(position_[0]) <<
  // "\t| y: " << std::to_string(float(position_[1]))  << "\t| z: "
  // <<std::to_string(position_[2])  << "\t\t\t|"  << std::endl; std::cout << "|
  // velocity [m/s] \t> vx : " << velocity_[0] << "\t| vy :" << velocity_[1] <<
  // "\t\t| vz :" << velocity_[2] << "\t\t\t\t|" <<  std::endl; printf("| euler
  // [deg] \t\t> roll: %.2f\t| pitch: %.2f\t| yaw: %.2f\t\t|\n", euler_[0],
  // euler_[1], euler_[2]); std::cout << "| body rates [rad/s] \t> gx : " <<
  // body_rates_[0] << "\t| gy :" << body_rates_[1] << "\t\t| gz :" <<
  // body_rates_[2] << "\t\t\t\t|" << std::endl;
  // std::cout << "| " << std::endl;
  // std::cout << "| f [N]: " << thrust_ << "\t | Mx [Nm]: " << moment_[0] <<
  // "\t | My [Nm]: " << moment_[1] << "\t | Mz [Nm]: " << moment_[2] <<
  // std::endl; std::cout <<
  // "|---------------------------------------------------------------------------------------|"
  // << std::endl; printf("| status:\t %s \n", isArmed.c_str()); std::cout << "|
  // mode  :\t" << "TRAJ" << "\t\t" << "remark:" << "\t" << "circular"  <<
  // std::endl;
  // std::cout <<
  // "|---------------------------------------------------------------------------------------|"
  // << std::endl;
  // // std::cout << "ROLL\t\t| PITCH \t| YAW\t\t| gx\t\t| gy\t\t| gz\t\t|" <<
  // std::endl;
  // // std::cout << "\r" << std::to_string(1/dt_) + "\t| " + std::to_string(0)
  // + "\t| " + std::to_string(0) + "\t| "+std::to_string(1)+"\t|
  // "+std::to_string(1)+"\t| "+std::to_string(0)+"\t|" << std::flush;
}

} // namespace qrotor_ground