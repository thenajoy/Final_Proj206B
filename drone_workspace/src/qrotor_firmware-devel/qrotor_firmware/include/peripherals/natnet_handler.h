#ifndef __QROTOR_FIRMWARE_NATNET_HANDLER_H__
#define __QROTOR_FIRMWARE_NATNET_HANDLER_H__
//
// Created by kotaru on 3/9/21.
//
#include "peripherals/external_pose_handler.hpp"

namespace qrotor_firmware {

class NatnetHandler : public ExternalPoseHandler {

private:
public:
  NatnetHandler();
  ~NatnetHandler();

  int init() override;
  int run() override;
};

} // namespace qrotor_firmware

#endif // __QROTOR_FIRMWARE_NATNET_HANDLER_H__
