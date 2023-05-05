#ifndef __QROTOR_FIRMWARE_NAVIO_UTILS_H__
#define __QROTOR_FIRMWARE_NAVIO_UTILS_H__

#include <cmath>
#include <stdio.h>
#include <memory>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>

#include "Navio2/RCInput_Navio2.h"
#include "Navio2/RCOutput_Navio2.h"

namespace qrotor_firmware {

class BrushlessMotor {
  private:
    unsigned int CHANNEL;
    unsigned int PWM_FREQ = 500;
    RCOutput_Navio2 pwm;

    float PWM_CUTOFF_MAX = 1900; // us (micro-second)
    float PWM_CUTOFF_MIN = 1200; // us (micro-second)
    const float PWM_HIGH = 2000; // us (micro-second)
    const float PWM_LOW = 1000;  // us (micro-second)

  public:
    BrushlessMotor(int _ch) {
        printf("Brushless motor on channel %d\n", _ch);
        CHANNEL = (unsigned int)_ch;
        initialize();
    }
    ~BrushlessMotor() {}

    void initialize() {
        pwm.initialize(CHANNEL);
        pwm.set_frequency(CHANNEL, PWM_FREQ);
        if (!BrushlessMotor::enable()) {
            printf("\033[31;1m  PWM channel not enabled!!! \033[0m\n");
        }
    }

    bool enable() {
        return pwm.enable(CHANNEL);
    }

    void update_frequency(int _freq) {
        PWM_FREQ = _freq;
        pwm.set_frequency(CHANNEL, PWM_FREQ);
    }

    void set_pwm_cutoff_max(float _pwm) {
        PWM_CUTOFF_MAX = _pwm;
    }

    void set_pwm_cutoff_min(float _pwm) {
        PWM_CUTOFF_MIN = _pwm;
    }

    void arm() {
        // printf("Channel %d, PWM_CUTOFF_MIN %f\n", CHANNEL, PWM_CUTOFF_MIN);
        pwm.set_duty_cycle(CHANNEL, PWM_CUTOFF_MIN);
    }

    void disarm() {
        // printf("Channel %d, PWM_LOW %f\n", CHANNEL, PWM_LOW);
        pwm.set_duty_cycle(CHANNEL, PWM_LOW);
    }
    void set_duty_cycle(float pwm_in_us) {
        // pwm in microseconds
        // printf("Channel %d, pwm_in_us %f\n", CHANNEL, pwm_in_us);
        pwm_in_us = pwm_in_us > PWM_CUTOFF_MAX ? PWM_CUTOFF_MAX : pwm_in_us < PWM_CUTOFF_MIN ? PWM_CUTOFF_MIN : pwm_in_us;
        pwm.set_duty_cycle(CHANNEL, pwm_in_us);
    }

    void calibrate() {
        sleep(1);
        pwm.set_duty_cycle(CHANNEL, PWM_LOW);
        sleep(1);
        // pwm.set_duty_cycle(CHANNEL, PWM_LOW);
        // sleep(3);
    }
};

}

#endif /* __QROTOR_FIRMWARE_NAVIO_UTILS_H__ */
