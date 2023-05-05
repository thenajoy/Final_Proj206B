#ifndef _QROTOR_FIRMWARE_BOARD_H_
#define _QROTOR_FIRMWARE_BOARD_H_

namespace qrotor_firmware {

class Board {
private:
  /* data */
public:
  // Board(/* args */) {}
  // ~Board() {}

  // setup
  virtual void init() {}

  // sensors
  virtual void sensors_init() {}

  // IMU
  virtual bool read_accel_gyro(float &ax, float &ay, float &az, float &gx,
                               float &gy, float &gz) {
    return true;
  }

  virtual bool read_accel_gyro_mag(float &ax, float &ay, float &az, float &gx,
                                   float &gy, float &gz, float &mx, float &my,
                                   float &mz) {
    return true;
  }

  // Radio
  virtual int read_channel(int _ch) { return 1; }

  //
  // Motors
  //
  virtual void write_pwms(const float *pwm) {}

  virtual void arm_motors() {}

  virtual void disarm_motors() {}

  //
  // ADC
  //
  virtual void adc_init() {}

  virtual void read_adc(int &voltage, int &current) {
    voltage = -1;
    current = -1;
  }

  //
  // LEDs
  //
  virtual void led0_on() {}

  virtual void led0_off() {}

  virtual void led0_toggle() {}

  virtual void led1_on() {}

  virtual void led1_off() {}

  virtual void led1_toggle() {}

  virtual void led_check() {}
};

} // namespace qrotor_firmware

#endif // _QROTOR_FIRMWARE_BOARD_H_