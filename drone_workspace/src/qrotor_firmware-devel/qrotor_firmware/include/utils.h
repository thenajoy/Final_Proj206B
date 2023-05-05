#ifndef QROTOR_FIRMWARE_UTILS_H
#define QROTOR_FIRMWARE_UTILS_H
#include <iostream>
#include <sstream>
#include <string>
#include <sys/time.h>

namespace utils {

static struct timeval tv;
inline unsigned long get_current_time() {
  gettimeofday(&tv, nullptr);
  return 1000000 * tv.tv_sec + tv.tv_usec;
}

/* http://www.cplusplus.com/forum/beginner/235373/ */
template <typename T> std::string Cat(T arg) {
  std::stringstream ss;
  ss << arg;
  return ss.str();
}

template <typename T, typename... Args>
std::string Cat(T current, Args... args) {
  std::string result;
  result += Cat(current);
  result += Cat((args)...);
  return result;
}

inline void ERROR() {}

inline void WARN() {}

} // namespace utils

#endif // QROTOR_FIRMWARE_UTILS_H
