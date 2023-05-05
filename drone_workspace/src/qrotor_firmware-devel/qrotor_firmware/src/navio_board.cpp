#include "navio_board.h"

namespace qrotor_firmware {

NavioBoard::NavioBoard() {
  // Motors
  for (int i = 0; i < PWM_NUM_OUTPUTS; i++) {
    r_pointer_ = new BrushlessMotor(PWM_CHANNELS[i]);
    motors_.push_back(r_pointer_);
  }

  num_of_motors = PWM_NUM_OUTPUTS;
  // system check
  check_apm(); // if apm is running, hardware cannot be accessed
  if (getuid()) {
    printf("Not root. Please launch with sudo \n");
  }

  // led
  led_.initialize();
  NavioBoard::led_check();
}

NavioBoard::~NavioBoard() {
  for (std::vector<BrushlessMotor *>::iterator pObj = motors_.begin();
       pObj != motors_.end(); ++pObj) {
    delete *pObj; // Note that this is deleting what pObj points to,
    // which is a pointer
  }
}

void NavioBoard::init() {
  // IMU
  imu_.initialize();
  if (!imu_.probe()) {
    printf("Sensor not enable\n");
  }

  float a1, a2, a3, g1, g2, g3;
  imu_.update();
  imu_.read_accelerometer(&a1, &a2, &a3);
  imu_.read_gyroscope(&g1, &g2, &g3);
  printf("a:%f, %f, %f, g: %f, %f, %f\n", a1, a2, a3, g1, g2, g3);
}

// sensors
void NavioBoard::sensors_init() {
  // Barometer

  // Radio receiver
  receiver_.initialize();

  // adc
  adc_init();
}

bool NavioBoard::read_accel_gyro(float &ax, float &ay, float &az, float &gx,
                                 float &gy, float &gz) {
  imu_.update();
  imu_.read_accelerometer(&ax, &ay, &az);
  imu_.read_gyroscope(&gx, &gy, &gz);
  return true;
}

bool NavioBoard::read_accel_gyro_mag(float &ax, float &ay, float &az, float &gx,
                                     float &gy, float &gz, float &mx, float &my,
                                     float &mz) {
  imu_.update();
  imu_.read_accelerometer(&ax, &ay, &az);
  imu_.read_gyroscope(&gx, &gy, &gz);
  imu_.read_magnetometer(&mx, &my, &mz);
  return true;
}

// Radio
int NavioBoard::read_channel(int _ch) {
  return receiver_.read(_ch);
}

void NavioBoard::write_pwms(const float *pwm) {
  for (int i = 0; i < num_of_motors; ++i) {
    motors_.at(i)->set_duty_cycle(pwm[i]);
  }
}

void NavioBoard::arm_motors() {
  for (int i = 0; i < num_of_motors; ++i) {
    motors_.at(i)->arm();
  }
}

void NavioBoard::disarm_motors() {
  //    printf("motors disarmed\n");
  for (int i = 0; i < num_of_motors; ++i) {
    motors_.at(i)->disarm();
  }
}

void NavioBoard::adc_init() { adc_.initialize(); }

void NavioBoard::read_adc(int &voltage, int &current) {
  voltage = adc_.read(2); // https://docs.emlid.com/navio2/dev/adc/
  current = adc_.read(3);
//  float results[6] = {0.0f};
//  for (int i = 0; i < adc_.get_channel_count(); i++)
//  {
//    results[i] = adc_.read(i);
//    if (results[i] == -1)
//      printf("adc read error");
//    printf("A%d: %.4fV ", i, results[i] / 1000);
//  }
//  printf("\n");
}

// LED
/*
  color combination since there
  is only one color led on Navio2
  led0    led1    color
  ---------------------
   yes     no      Blue
   no      yes     Green
   no      no      Red
   yes     yes     Cyan
*/
void NavioBoard::led0_on() {
  if (led1_)
    led_.setColor(Colors::Cyan);
  else
    led_.setColor(Colors::Blue);
  led0_ = true;
}

void NavioBoard::led0_off() {
  if (led1_)
    led_.setColor(Colors::Green);
  else
    led_.setColor(Colors::Red);
  led0_ = false;
}

void NavioBoard::led0_toggle() {
  if (led0_)
    NavioBoard::led0_off();
  else
    NavioBoard::led0_on();
}

void NavioBoard::led1_on() {
  if (led0_)
    led_.setColor(Colors::Cyan);
  else
    led_.setColor(Colors::Green);
  led1_ = true;
}

void NavioBoard::led1_off() {
  if (led0_)
    led_.setColor(Colors::Blue);
  else
    led_.setColor(Colors::Red);
  led1_ = false;
}

void NavioBoard::led1_toggle() {
  if (led1_)
    NavioBoard::led1_off();
  else
    NavioBoard::led1_on();
}

void NavioBoard::led_check() {
  led_.setColor(Colors::Green);
  printf("LED is green\n");
  usleep(200000);
  led_.setColor(Colors::Cyan);
  printf("LED is cyan\n");
  usleep(200000);
  led_.setColor(Colors::Blue);
  printf("LED is blue\n");
  usleep(200000);
  led_.setColor(Colors::Magenta);
  printf("LED is magenta\n");
  usleep(200000);
  led_.setColor(Colors::Red);
  printf("LED is red\n");
  usleep(200000);
  led_.setColor(Colors::Yellow);
  printf("LED is yellow\n");
  usleep(200000);
  printf("LED check complete\n");
}

} // namespace qrotor_firmware
