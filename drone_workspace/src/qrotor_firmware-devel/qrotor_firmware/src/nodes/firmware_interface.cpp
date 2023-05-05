#include "nodes/firmware_interface.h"

namespace qrotor_firmware {
FirmwareInterface::FirmwareInterface(ros::NodeHandle &_nh)
    : nh_(_nh), firmware_(board_) {
  std::string ns = ros::this_node::getNamespace();
  ns.erase(0, 2);
  name = ns;
  std::cout << "entering firmware interface of " << name << std::endl;
  start_time_s = ros::Time::now().toSec();
  last_update_s = ros::Time::now().toSec();
  clock();
}

FirmwareInterface::~FirmwareInterface() = default;

// clock
void FirmwareInterface::clock() {
  now_s = get_current_time();
  dt_s = (float)(now_s - last_update_s);
  firmware_.update_time(float(now_s), dt_s);
  last_update_s = now_s;
  //  ROS_INFO("1/dt_s: %f", 1 / dt_s);
}
double FirmwareInterface::get_current_time() const {
  return ros::Time::now().toSec() - start_time_s;
}

void FirmwareInterface::init() {

  // initialize firmware
  firmware_.init();

  // binding a thread to high-level firmware
  //  this->hl_thread =
  //  boost::thread(boost::bind(&FirmwareInterface::HighLevelThread, this));

  // initial params re-configurator & read ros parameters
  params_reconfig_ = new ParamsReconfig(nh_, firmware_);
  params_reconfig_->init(); // TODO fix this inter-dependency!!!

  // ros service
  firmware_io_ = new FirmwareIO(nh_, firmware_);

  //  // creating and spinning a firmware thread
  //  firmware_thread_ =std::thread(&FirmwareInterface::run_firmware, this);
  // firmware_thread.join(); << do not add this :D >>
  // creating and spinning a t265 node
  t265_thread_ = std::thread(&FirmwareInterface::run_t265, this);

  // update time
  clock();
}

[[noreturn]] void FirmwareInterface::run_firmware() {
  while (true) {
    firmware_.run();
  }
}

void FirmwareInterface::run_t265() {
  if (firmware_.ext_pose_handler_->init())
    Logger::ERROR(utils::Cat("t265 initialization failed!"));
  else
    Logger::SUCCESS(utils::Cat("t265 initialized!"));

  try {
    // Main loop
    while (true) {
      firmware_.ext_pose_handler_->run();
    }
  } catch (const rs2::error &e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "("
              << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    printf("\033[31;1mt265 loop failed! \033[0m\n");
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    printf("\033[31;1mt265 loop failed! \033[0m\n");
  }
}

// outer loop
void FirmwareInterface::HighLevelThread() {
  //  ROS_WARN("Initializing new thread for high-level control!");
  //
  //  ros::NodeHandlePtr node =
  //  boost::make_shared<ros::NodeHandle>("falcon_hl"); ros::Publisher pub_b =
  //  node->advertise<std_msgs::Empty>("topic_b", 10); this->_sub_mocap =
  //  node->subscribe("odometry/mocap", 3,
  //  &FirmwareInterface::callback_sub_mocap, this);
  //
  //  ros::Rate loop_rate(100);
  //  double prev_time, curr_time, dt_hl;
  //  prev_time = ros::Time::now().toSec();
  //  while (ros::ok()) {
  //    curr_time = ros::Time::now().toSec();
  //    dt_hl = curr_time - prev_time;
  //    prev_time = curr_time;
  //    ROS_WARN("outer loop: %f", 1 / dt_hl);
  //
  ////    this->firmware_.outer_loop(dt_s);
  //
  //    std_msgs::Empty msg;
  //    pub_b.publish(msg);
  //    loop_rate.sleep();
  //  }
}

// public functions
void FirmwareInterface::run() {
  clock();
  ros::Rate loop_handle(ros_loop_rate_);
  //    run_firmware();
  while (ros::ok()) {
    ros::spinOnce();

    // update time
    clock();

    // run the main firmware for the drone
    firmware_.run();
    firmware_io_->run();

    // handle loop rate
    loop_handle.sleep();
  }
  ros::shutdown();
}

} // namespace qrotor_firmware
