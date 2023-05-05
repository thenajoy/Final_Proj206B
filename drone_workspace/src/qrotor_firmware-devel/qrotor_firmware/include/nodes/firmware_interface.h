#ifndef QROTOR_FIRMWARE_INTERFACE_H
#define QROTOR_FIRMWARE_INTERFACE_H

// #include <eigen_conversions/eigen_msg.h>
#include <boost/thread/thread.hpp>
#include <ctime>
#include <ros/ros.h>
#include <tf/tf.h>

#include <eigen3/Eigen/Dense>
#include <string>
#include <thread>
#include <vector>

#include "data_structures.h"
#include "navio_board.h"
#include "nodes/firmware_io.h"
#include "firmware_params_reconfig.h"
#include "peripherals/rs_t265.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {
class FirmwareInterface {
protected:
  /// \brief Ros Node Handle running the main qrotor firmware
  ros::NodeHandle nh_;
  /// \brief Drone Name
  std::string name = "falcon";
  /// \brief Navio Board handler
  NavioBoard board_;
  /// \brief Flight Controller
  FlightController firmware_;
  std::thread firmware_thread_, t265_thread_;

  /// \biref ros node pointer
  ros::NodeHandlePtr node_ptr_;
  /// \brief thread to run low-frequency/high-level stuff
  boost::thread hl_thread;
  /// \brief run high-level firmware
  void HighLevelThread();

  /// \brief time variables
  double start_time_s = 0.0;
  double last_update_s = 0.0; // ros::Time::now().toSec();
  double now_s = 0.0;         // ros::Time::now().toSec();
  float dt_s = 0.0;
  double dt_sum = 0.0;
  double ros_loop_rate_ = 500;

  [[noreturn]] void run_firmware();
  void run_t265();

  /// \brief params reconfiguration
  ParamsReconfig *params_reconfig_ = nullptr;
  /// \brief params input-output interface
  FirmwareIO *firmware_io_ = nullptr;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit FirmwareInterface(ros::NodeHandle &_nh);
  ~FirmwareInterface();

  /// \brief firmware initialization
  void init();
  /// \brief firmware run
  void run();
  /// \brief function to update the system clock based on ros::Time
  void clock();
  /// \brief function to return the firmware time
  double get_current_time() const;
};

} // namespace qrotor_firmware

#endif // QROTOR_FIRMWARE_INTERFACE_H
