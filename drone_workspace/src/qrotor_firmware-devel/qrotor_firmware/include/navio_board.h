#ifndef _QROTOR_FIRMWARE_NAVIO_BOARD_H__
#define _QROTOR_FIRMWARE_NAVIO_BOARD_H__

#include <chrono>
#include <ctime>
#include <map>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

// Navio common
#include "Common/ADC.h"
#include "Common/I2Cdev.h"
#include "Common/InertialSensor.h"
#include "Common/Led.h"
#include "Common/MPU9250.h"
#include "Common/MS5611.h"
#include "Common/RCInput.h"
#include "Common/RCOutput.h"
#include "Common/SPIdev.h"
#include "Common/Ublox.h"
#include "Common/Util.h"
#include "Common/gpio.h"
// Navio 2
#include "Navio2/ADC_Navio2.h"
#include "Navio2/LSM9DS1.h"
#include "Navio2/Led_Navio2.h"
#include "Navio2/PWM.h"
#include "Navio2/RCInput_Navio2.h"
#include "Navio2/RCOutput_Navio2.h"
#include "Navio2/RGBled.h"

#include "navio_utils.h"
#include "parameters.h"
#include "board.h"

namespace qrotor_firmware {

class NavioBoard : public Board {
  private:
    // clock time
    // Using time point and system_clock
    std::chrono::time_point<std::chrono::high_resolution_clock> clock_start_,
        clock_now_;
    std::chrono::duration<double, std::milli> millis_;
    std::chrono::duration<double, std::micro> micros_;

    // imu
    MPU9250 imu_;
    //   LSM9DS1   imu2_;

    // radio receiver
    RCInput_Navio2 receiver_;

    // output
    // leds
    Led_Navio2 led_;
    bool led0_ = false;
    bool led1_ = false;
    // motors
    int num_of_motors;
    std::vector<BrushlessMotor*> motors_;
    BrushlessMotor* r_pointer_ = NULL;

    //   float _accel_scale = 1.0;
    //   float _gyro_scale = 1.0;

    //   bool new_imu_data_;
    //   uint64_t imu_time_us_;

    // ADC to read battery voltage and current
    ADC_Navio2 adc_{};

  public:
    NavioBoard();
    ~NavioBoard();

    // setup
    virtual void init();

    // sensors
    virtual void sensors_init();

    // IMU
    virtual bool read_accel_gyro(float& ax, float& ay, float& az, float& gx, float& gy,
                         float& gz);
    virtual bool read_accel_gyro_mag(float& ax, float& ay, float& az, float& gx,
                             float& gy, float& gz, float& mx, float& my,
                             float& mz);

    //
    // Radio
    //
    virtual int read_channel(int _ch);

    //
    // Motors
    //
    virtual void write_pwms(const float* pwm);
    virtual void arm_motors();
    virtual void disarm_motors();

    //
    // ADC
    //
    virtual void adc_init();
    virtual void read_adc(int &voltage, int &current);

    //
    // LEDs
    //
    virtual void led0_on();
    virtual void led0_off();
    virtual void led0_toggle();

    virtual void led1_on();
    virtual void led1_off();
    virtual void led1_toggle();

    virtual void led_check();
};

} // namespace qrotor_firmware

#endif /* _QROTOR_FIRMWARE_NAVIO_BOARD_H__ */
