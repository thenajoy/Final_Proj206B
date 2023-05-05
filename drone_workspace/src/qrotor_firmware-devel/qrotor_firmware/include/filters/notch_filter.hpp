///
/// Created by kotaru on 1/26/21.
/// Based on answer given here % source:https://dsp.stackexchange.com/a/1090
///

#include "Matrix/matrix/math.hpp"

#ifndef __QROTOR_FIRMWARE_NOTCH_FILTER_HPP_
#define __QROTOR_FIRMWARE_NOTCH_FILTER_HPP_

namespace qrotor_firmware {

class NotchFilter {
 protected:
  float _a0{0.0f}, _a1{0.0f}, _a2{0.0f};
  float _b0{0.0f}, _b1{0.0f}, _b2{0.0f};

  float _notch_width{0.1f};
  float _sample_freq{500.0f};
  float _notch_freq{80.f};

  matrix::Vector3f _delay_sample_1{0.0f, 0.0f, 0.0f};
  matrix::Vector3f _delay_sample_2{0.0f, 0.0f, 0.0f};
  matrix::Vector3f _delay_filter_1{0.0f, 0.0f, 0.0f};
  matrix::Vector3f _delay_filter_2{0.0f, 0.0f, 0.0f};

 public:
  NotchFilter() : NotchFilter(500.0f, 80.0f, 0.55f) {
  }

  NotchFilter(float sample_freq, float notch_freq, float notch_width) {
    set_notch_frequency(sample_freq, notch_freq, notch_width);
  }

  void set_notch_frequency(float sample_freq, float notch_freq, float notch_width) {
    _sample_freq = sample_freq;
    _notch_freq = notch_freq;
    _notch_width = notch_width;

    _delay_filter_1.zero();
    _delay_filter_2.zero();
    _delay_sample_1.zero();
    _delay_sample_2.zero();

    if (_notch_freq <= 0) {
      _b0 = 1.0f;
      _b1 = 0.0f;
      _b2 = 0.0f;
      _a0 = 1.0f;
      _a1 = 0.0f;
      _a2 = 0.0f;
      return;
    }

    const float f0 = _notch_freq;     // notch frequency
    const float fn = _sample_freq / 2;  // Nyquist frequency
    const float fr = f0 / fn;    // ratio of notch freq. to Nyquist freq
    const float th = M_PI * fr;
    const float nwc = 1 - _notch_width;

    _b0 = 1;
    _b1 = -2 * cosf(th);
    _b2 = 1;

    _a0 = 1;
    _a1 = -2 * cosf(th) * nwc;
    _a2 = nwc * nwc;
  }

  inline matrix::Vector3f apply(const matrix::Vector3f &sample) {
    matrix::Vector3f output;
    output = sample * _b0 + _delay_sample_1 * _b1 + _delay_sample_2 * _b2;
    output += -_delay_filter_1 * _a1 - _delay_filter_2 * _a2;

    _delay_sample_2 = _delay_sample_1;
    _delay_sample_1 = sample;

    _delay_filter_2 = _delay_filter_1;
    _delay_filter_1 = output;

    return output;
  }

  matrix::Vector3f reset(const matrix::Vector3f &sample) {
    _delay_sample_1 = sample;
    _delay_sample_2 = sample;
    _delay_filter_1.zero();
    _delay_filter_2.zero();
    return apply(sample);
  }

};

}

#endif // __QROTOR_FIRMWARE_NOTCH_FILTER_HPP_
