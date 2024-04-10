// Test Plan 1 & 2

#include <stdio.h>
#include <unistd.h>

#include <iostream>

#include "moteus.h"

double inputAndLimitTorque(const std::string &motor_name, double max_torque)
{
    double torque_command;
    std::cout << "Enter torque command for " << motor_name << ": ";
    std::cin >> torque_command;

    // Limit the torque command to the nearest value within the range [-max_torque, max_torque]
    if (torque_command < -max_torque)
        torque_command = -max_torque;
    else if (torque_command > max_torque)
        torque_command = max_torque;

    return torque_command;
}

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

    for (auto &c : controllers)
    {
        c->SetStop();
    }

    moteus::PositionMode::Command cmd;
    cmd.kp_scale = 0.0;
    cmd.kd_scale = 0.0;
    cmd.feedforward_torque = 0.0;

    const double MAX_TORQUE = 0.2;
    double torque_command[2] = {};

    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;

    // Manually input torque commands
    torque_command[0] = inputAndLimitTorque("motor 1", MAX_TORQUE);
    torque_command[1] = inputAndLimitTorque("motor 2", MAX_TORQUE);

    std::cout << "Torque command for motor 1: " << torque_command[0] << std::endl;
    std::cout << "Torque command for motor 2: " << torque_command[1] << std::endl;

    // Main loop
    while (true)
    {
        ::usleep(10);

        send_frames.clear();
        receive_frames.clear();

        for (size_t i = 0; i < controllers.size(); i++)
        {
            cmd.feedforward_torque = torque_command[i];
            send_frames.push_back(controllers[i]->MakePosition(cmd));
        }

        // Send frames
        transport->BlockingCycle(&send_frames[0], send_frames.size(), &receive_frames);
    }

    printf("Entering fault mode!\n");

    while (true)
    {
        ::usleep(50000);

        for (auto &c : controllers)
        {
            c->SetStop();
        }
    }

    return 0;
}
