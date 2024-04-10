// Test Plan 1 & 2

#include <stdio.h>
#include <unistd.h>

#include <iostream>

#include "moteus.h"

int main(int argc, char **argv)
{
    using namespace mjbots;
    // Set up controllers and transport
    moteus::Controller::DefaultArgProcess(argc, argv);
    auto transport = moteus::Controller::MakeSingletonTransport({});

    // Options for setting up controllers
    moteus::Controller::Options options_common;

    // Set position format
    auto &pf = options_common.position_format;
    pf.position = moteus::kIgnore;
    pf.velocity = moteus::kIgnore;
    pf.feedforward_torque = moteus::kFloat;
    pf.kp_scale = moteus::kInt8;
    pf.kd_scale = moteus::kInt8;

    // Create two controllers
    std::shared_ptr<moteus::Controller> controller1 = std::make_shared<moteus::Controller>([&]()
                                                                                           {
        auto options = options_common;
        options.id = 1;
        return options; }());

    std::shared_ptr<moteus::Controller> controller2 = std::make_shared<moteus::Controller>([&]()
                                                                                           {
        auto options = options_common;
        options.id = 2;
        return options; }());

    // Stop the controllers initially
    controller1->SetStop();
    controller2->SetStop();

    // Command for position mode
    moteus::PositionMode::Command cmd;
    cmd.kp_scale = 0.0;
    cmd.kd_scale = 0.0;
    cmd.feedforward_torque = 0.0;

    double torque_command[2] = {0.0, 0.0}; // Torque commands for each motor

    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;

    // Main loop
    while (true)
    {
        ::usleep(10);

        send_frames.clear();
        receive_frames.clear();

        // Manually input torque commands
        std::cout << "Enter torque command for motor 1: ";
        std::cin >> torque_command[0];
        std::cout << "Enter torque command for motor 2: ";
        std::cin >> torque_command[1];

        // Create send frames
        std::vector<moteus::CanFdFrame> send_frames = {controller1->MakePosition(cmd),
                                                       controller2->MakePosition(cmd)};

        // Send frames
        transport->BlockingCycle(send_frames.data(), send_frames.size(), nullptr);

        // Display torque commands
        std::cout << "Torque command for motor 1: " << torque_command[0] << std::endl;
        std::cout << "Torque command for motor 2: " << torque_command[1] << std::endl;
    }

    printf("Entering fault mode!\n");

    while (true)
    {
        ::usleep(50000);

        controller1->SetStop();
        controller2->SetStop();
    }

    return 0;
}
