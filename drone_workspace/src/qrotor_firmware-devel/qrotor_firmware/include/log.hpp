#ifndef QROTOR_FIRMWARE_LOGGING_H
#define QROTOR_FIRMWARE_LOGGING_H
#include <bits/stdc++.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <string>
// #include <lcm/lcm-cpp.hpp>

#include <mathlib/mathlib.h>
#include <Matrix/matrix/math.hpp>

#include "board.h"
#include "data_structures.h"
#include "parameters.h"
#include "utils.h"

namespace qrotor_firmware {

static const tColors _tColors;

class Logger {
    std::string log_rel_dir = "/qrotor_firmware/logs";
    std::string home_dir, log_dir;

   public:
    Logger() {}
    ~Logger() {}

    static void WARN(const std::string& msg) {
        std::string s = "";
        s += _tColors.YELLOW + std::string("[WARN] [") + std::to_string(utils::get_current_time()) + std::string("]: ") + msg + _tColors.NC;
        std::cout << s << std::endl;
    }

    static void ERROR(const std::string& msg) {
        std::string s = "";
        s += _tColors.RED + std::string("[ERROR] [") + std::to_string(utils::get_current_time()) + std::string("]: ") + msg + _tColors.NC;
        std::cout << s << std::endl;
    }

    static void SUCCESS(const std::string& msg) {
        std::string s = "";
        s += _tColors.GREEN + std::string("[SUCCESS] [") + std::to_string(utils::get_current_time()) + std::string("]: ") + msg + _tColors.NC;
        std::cout << s << std::endl;
    }

    static void STATUS(const std::string& msg) {
        std::string s = "";
        s += _tColors.CYAN + std::string("[STATUS] [") + std::to_string(utils::get_current_time()) + std::string("]: ") + msg + _tColors.NC;
        std::cout << s << std::endl;
    }

    static void INFO(const std::string& msg) {
        std::string s = "";
        s += _tColors.WHITE + std::string("[INFO] [") + std::to_string(utils::get_current_time()) + std::string("]: ") + msg + _tColors.NC;
        std::cout << s << std::endl;
    }

    void init() {
        home_dir = std::string("/home/pi");  // getenv("HOME"); TODO: fix this
        std::cout << home_dir << std::endl;

        log_dir = home_dir.append(log_rel_dir);
        // Creating a directory
        if (mkdir(log_dir.c_str(), 0777) == -1)
            std::cout << strerror(errno) << std::endl;
        else
            std::cout << "Directory created" << std::endl;
        STATUS(std::string("Logger initialized!"));
    }
};

}  // namespace qrotor_firmware

#endif  // QROTOR_FIRMWARE_LOGGING_H
