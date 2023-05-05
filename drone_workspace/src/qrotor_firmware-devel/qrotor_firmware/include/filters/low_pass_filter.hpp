///
/// @file	LowPassFilter.h
/// @brief	A class to implement a second order low pass filter
/// Based on PX4/LowPassFilter2p.hpp by Leonard Hall <LeonardTHall@gmail.com>
//
#include "Matrix/matrix/math.hpp"

#ifndef __QROTOR_FIRMWARE_LOW_PASS_FILTER_HPP__
#define __QROTOR_FIRMWARE_LOW_PASS_FILTER_HPP__

namespace qrotor_firmware {

class LowPassFilter {
 protected:
  float _cutoff_freq{60.f};

  float _a1{0.0f}, _a2{0.0f};
  float _b1{0.0f}, _b2{0.0f}, _b0{0.0f};

  matrix::Vector3f _delay_element_1{0.0f, 0.0f, 0.0f};
  matrix::Vector3f _delay_element_2{0.0f, 0.0f, 0.0f};

 public:
  LowPassFilter() : LowPassFilter(500.f, 60.f) {}

  LowPassFilter(float sample_freq, float cutoff_freq) {
    set_cutoff_freq(sample_freq, cutoff_freq);
  }

  /// set cutoff frequency
  void set_cutoff_freq(float sample_freq, float cutoff_freq) {
    _cutoff_freq = cutoff_freq;

    // reset delay elements on filter change
    _delay_element_1.zero();
    _delay_element_2.zero();

    if (_cutoff_freq <= 0.0f) {
      // no filtering_al
      _b0 = 1.0f;
      _b1 = 0.0f;
      _b2 = 0.0f;

      _a1 = 0.0f;
      _a2 = 0.0f;

      return;
    }

    const float fr = sample_freq / _cutoff_freq;
    const float ohm = tanf(M_PI / fr);
    const float c = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;

    _b0 = ohm * ohm / c;
    _b1 = 2.0f * _b0;
    _b2 = _b0;

    _a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    _a2 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
  }
  matrix::Vector3f reset(const matrix::Vector3f &sample) {
    const matrix::Vector3f dval = sample / (_b0 + _b1 + _b2);

    _delay_element_1 = sample;
    _delay_element_2 = sample;
    return apply(sample);
  }

  inline matrix::Vector3f apply(const matrix::Vector3f &sample) {
    // do the filtering
    const matrix::Vector3f delay_element_0{sample - _delay_element_1 * _a1 - _delay_element_2 * _a2};
    const matrix::Vector3f output{delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2};

    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;

    return output;
  }

};

}

#endif //__QROTOR_FIRMWARE_LOW_PASS_FILTER_HPP__
