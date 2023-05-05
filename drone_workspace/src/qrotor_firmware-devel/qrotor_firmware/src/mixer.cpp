#include "mixer.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

Mixer::Mixer(FlightController &_flightController) : firmware_(_flightController) {

  allocation_matrix_.zero();
  allocation_offset_.zero();
  forces_to_rotor_speed_.zero();
}

void Mixer::init() {
  allocation_offset_.zero();
  for (int i = 0; i < firmware_.vehicle_params_.num_rotors_; ++i) {

    float l_ = firmware_.vehicle_params_.rotors_.at(i).arm_length;
    float th_ = firmware_.vehicle_params_.rotors_.at(i).angle;
    auto d_ = (float) firmware_.vehicle_params_.rotors_.at(i).direction;
    float kf_ = firmware_.vehicle_params_.rotors_.at(i).rotor_force_constant;
    float cf_ = firmware_.vehicle_params_.rotors_.at(i).rotor_force_offset;
    float km_ = firmware_.vehicle_params_.rotors_.at(i).rotor_moment_constant;
    float cm_ = firmware_.vehicle_params_.rotors_.at(i).rotor_moment_offset;

    allocation_matrix_(0, i) = kf_;
    allocation_matrix_(1, i) = l_ * sin(th_) * kf_;
    allocation_matrix_(2, i) = -l_ * cos(th_) * kf_;
    allocation_matrix_(3, i) = d_ * km_;

    //        std::cout << "kf: " << kf_ << " l " << l_ << " th_ " << th_
    //        << " d_ " << d_ << " "
    //        << l_ * sin(th_) * kf_ << " " << -l_ * cos(th_) * kf_ << " " <<
    //        d_*km_ << std::endl;

    matrix::Matrix<float, 4, 1> data_tmp;
    data_tmp(0, 0) = cf_;
    data_tmp(1, 0) = l_ * float(sin(th_)) * cf_;
    data_tmp(2, 0) = -l_ * float(cos(th_)) * cf_;
    data_tmp(3, 0) = d_ * cm_;
    allocation_offset_ = allocation_offset_ + data_tmp;
  }

  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  // using 10e6 for numerical stability
  forces_to_rotor_speed_ = geninv(allocation_matrix_ * 10e6) * 10e6;

  //  printf("allocation matrix\n");
  //  allocation_matrix_.print();
  //  printf("allocation offset\n");
  //  allocation_offset_.print();
  //  printf("forces to rotor speed\n");
  //  forces_to_rotor_speed_.print();
  Logger::SUCCESS(std::string("Mixer initialized!"));
}

void Mixer::run() {
  // set pwm to motors
  if (!firmware_.state_machine_.state().failsafe) {
    // no failsafe
    if (firmware_.state_machine_.state().armed) {
      // vehicle armed
      if (firmware_.state_machine_.state().mode == StateMachine::POSITION_HOLD) {
        if (firmware_.mission_planner_->is_in_flight()) {
          // if in flight mode start using control
          this->write_to_board();
        } else {
          // if not in flight mode, keep it armed
          firmware_.board_.arm_motors();
        }
      } else {
        // attitude/stabilize control
        this->write_to_board();
      }
    } else {
      // vehicle not armed
      firmware_.board_.disarm_motors();
    }
  } else {
    // failsafe, disarm motors
    firmware_.board_.disarm_motors();
  }
}

void Mixer::write_to_board() {
  this->allocate_pwms();
  firmware_.board_.write_pwms(pwm_us_array_);

}

void Mixer::allocate_pwms() {
  // map force & moment to pwms
  float data_f[4] = {
      firmware_.att_controller_->input().thrust,
      firmware_.att_controller_->input().moment(0),
      firmware_.att_controller_->input().moment(1),
      firmware_.att_controller_->input().moment(2)}; // SI units (N, Nm, Nm, Nm)
  matrix::Matrix<float, 4, 1> rotors_speed_squared_,
      thrust_moment_vector(data_f);
  rotors_speed_squared_ =
      forces_to_rotor_speed_ * (thrust_moment_vector - allocation_offset_);

  // thrust to rotor-speed mapping
  for (int i = 0; i < PWM_NUM_OUTPUTS; ++i) {
    rotors_speed_squared_(i, 0) =
        rotors_speed_squared_(i, 0) > 0 ? rotors_speed_squared_(i, 0) : 0;
    // pwm[i] = (sqrt(rotors_speed_squared_[i]) -
    // vehicle_params_.pwm2rotorspeed_offset_)/vehicle_params_.pwm2rotorspeed_scale_;
    float w = sqrt(rotors_speed_squared_(i, 0));
    float pwm_ = firmware_.vehicle_params_.rotorspeed2pwm_scale_ * w +
        firmware_.vehicle_params_.rotorspeed2pwm_offset_;
    pwm_us_array_[i] = pwm_;
    //    pwm_us_array_[i] = pwm_ * (alpha) + (1 - alpha) * pwm_us_array_old[i];
    //    pwm_us_array_old[i] = pwm_us_array_[i];
    //    printf("i: %d, w: %f, pwm: %f\n", i, w, pwm_us_array_[i]);
  }
//  printf("pwm[0]: %f, pwm[1]: %f, pwm[2]: %f, pwm[3]: %f\n",
//         pwm_us_array_[0],
//         pwm_us_array_[1], pwm_us_array_[2], pwm_us_array_[3]);
  //  printf("a: %f, , b:%f", firmware_.vehicle_params_.rotorspeed2pwm_scale_,
  //         firmware_.vehicle_params_.rotorspeed2pwm_offset_);
}

} // namespace qrotor_firmware
