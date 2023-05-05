#include "estimation/pose_estimator.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

PoseEstimator::PoseEstimator(FlightController& _flightcontroller): firmware_(_flightcontroller) {

}

PoseEstimator::~PoseEstimator() = default;

void PoseEstimator::init() {
    pose_.reset();
    Logger::STATUS(std::string("PoseEstimator initialized!"));
    std::cout << "Position: " << std::setprecision(3) << std::fixed << pose_.position(0) << " " <<
              pose_.position(1) << " " << pose_.position(2) << " (meters)" << std::endl;
    std::cout << "Velocity: " << std::setprecision(3) << std::fixed << pose_.velocity(0) << " " <<
              pose_.velocity(1) << " " << pose_.velocity(2) << " (meters)" << std::endl;
}

void PoseEstimator::run(float dt) {

}

}
