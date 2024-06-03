#include <stdio.h>
#include <unistd.h>
#include <csignal> // For signal handling
#include <iostream>

#include "moteus.h"

// Global flag for indicating if Ctrl+C was pressed
volatile sig_atomic_t ctrl_c_pressed = 0;

std::vector<long long> loop_durations_;

// Signal handler function
void signalHandler(int signal)
{
    ctrl_c_pressed = 1; // Set flag to indicate Ctrl+C was pressed
}

int main(int argc, char **argv)
{
    // Set up signal handler for Ctrl+C (SIGINT)
    std::signal(SIGINT, signalHandler);

    using namespace mjbots;
    // Set up controllers and transport
    moteus::Controller::DefaultArgProcess(argc, argv);
    auto transport = moteus::Controller::MakeSingletonTransport({});

    // Options for setting up controllers
    mjbots::moteus::Controller::Options options_common;

    // Set position format
    auto &pf = options_common.position_format;
    auto &qf = options_common.query_format;
    pf.position = mjbots::moteus::kInt16;
    pf.velocity = mjbots::moteus::kIgnore;
    pf.feedforward_torque = mjbots::moteus::kFloat;
    pf.kp_scale = mjbots::moteus::kInt8;
    pf.kd_scale = mjbots::moteus::kInt8;
    pf.accel_limit = mjbots::moteus::kInt16;
    qf.trajectory_complete = mjbots::moteus::kIgnore;

    // Create two controllers
    std::vector<std::shared_ptr<moteus::Controller>> controllers = {
        std::make_shared<moteus::Controller>([&]()
                                             {
            auto options = options_common;
            options.id = 1;
            return options; }()),
        std::make_shared<moteus::Controller>([&]()
                                             {
            auto options = options_common;
            options.id = 2;
            return options; }()),
    };

    moteus::PositionMode::Command cmd;
    cmd.kp_scale = 0.0;
    cmd.kd_scale = 0.0;
    cmd.feedforward_torque = 0.0;

    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;

    double torque_command[2] = {0.1, -0.1};

    // Main loop
    while (!ctrl_c_pressed)
    {

        send_frames.clear();
        receive_frames.clear();

        auto maybe_servo1 = controllers[0]->SetQuery();
        auto maybe_servo2 = controllers[1]->SetQuery();

        const auto &v1 = maybe_servo1->values;
        const auto &v2 = maybe_servo2->values;

        printf("MODE: %2d/%2d  POSITION: %6.3f/%6.3f  TORQUE: %6.3f/%6.3f  TEMP: %4.1f/%4.1f  TRAJCOMPLETE: %s/%s FAULTS: %2d/%2d\r",
               static_cast<int>(v1.mode), static_cast<int>(v2.mode),
               v1.position, v2.position,
               v1.torque, v2.torque,
               v1.temperature, v2.temperature, v1.trajectory_complete, v2.trajectory_complete, static_cast<int>(v1.fault), static_cast<int>(v2.fault));
        fflush(stdout);
        ::usleep(10000);
    }

    std::cout << "Entering fault mode!" << std::endl;

    ::usleep(50000);

    return 0;
}
