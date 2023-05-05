#include <iostream>
#include <pthread.h>
#include "navio_board.h"
#include "qrotor_flight.h"
#include "peripherals/rs_t265.h"

void* runRSt265(void* fcu_) {
    // TODO: replace pthread with std::thread
    auto* fcu = (qrotor_firmware::FlightController*) fcu_;
    try {
        // Main loop
        while (true) {
            fcu->ext_pose_handler_->run();
        }
    } catch (const rs2::error& e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        printf("\033[31;1mt265 loop failed! \033[0m\n");
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        printf("\033[31;1mt265 loop failed! \033[0m\n");
    }
    pthread_exit(nullptr);
}


int main(int argc, char* argv[]) {

    qrotor_firmware::NavioBoard board;
    qrotor_firmware::FlightController firmware(board);

    // initialize firmware
    firmware.init();

    // initialize tracking camera
    if (firmware.ext_pose_handler_->init()) {
        qrotor_firmware::Logger::ERROR(utils::Cat("t265 initialization failed!"));
    }

    // creating a thread to run realsense tracking camera
    pthread_t t265_thread;
    if(pthread_create(&t265_thread, nullptr, runRSt265, (void*)&firmware)) {
        qrotor_firmware::Logger::ERROR(utils::Cat("Failed to create firmware thread!"));
        return 0;
    }

    firmware.main();
    pthread_exit(nullptr);

    return 1;
}


