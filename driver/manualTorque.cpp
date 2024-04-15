// Test Plan 1 & 2

#include <stdio.h>
#include <unistd.h>
#include <csignal> // For signal handling
#include <iostream>

#include "moteus.h"

// Global flag for indicating if Ctrl+C was pressed
volatile sig_atomic_t ctrl_c_pressed = 0;

// Signal handler function
void signalHandler(int signal)
{
    ctrl_c_pressed = 1; // Set flag to indicate Ctrl+C was pressed
}

double inputAndLimitTorque(const std::string &motor_name, double max_torque)
{
    double torque_command;
    std::cout << "Enter torque command for " << motor_name << ": ";
    std::cin >> torque_command;

    // Limit the torque command to the nearest value within the range [-max_torque, max_torque]
    if (torque_command < -max_torque)
    {
        std::cout << "TORQUE " << torque_command << "Nm TOO HIGH. SETTING TO MAX TORQUE: " << -max_torque << "Nm" << std::endl;
        torque_command = -max_torque;
    }
    else if (torque_command > max_torque)
    {
        std::cout << "TORQUE " << torque_command << "Nm TOO HIGH. SETTING TO MAX TORQUE: " << max_torque << "Nm" << std::endl;
        torque_command = max_torque;
    }
    return torque_command;
}

double getPracTorque(double scaleWeight)
{
    double armLength = 0.195;

    return ((scaleWeight / 9.81) * armLength) / 10.0;
}

double getPracTorqueFromInertia(double scaleWeight)
{
    double armLength = 0.195;
    double motorRadius = 0.035;
    double armMass = 0.12;
    double motorMass = 0.24;

    // Moment of Inertia for rod with disk attached at the end
    double MOI = ((1 / 3) * armMass * pow(armLength, 2)) +
                 ((1 / 2) * motorMass * pow(motorRadius, 2)) +
                 (motorMass * pow(armLength + motorRadius, 2));

    std::cout << "MOI: " << MOI << std::endl;

    // Linear acceleration
    double a = (scaleWeight / 1000 * 9.81) / (0.26); // 0.26kg is setup based. This WILL change with different Setups
    std::cout << "a: " << a << std::endl;

    // Angular acceleration
    double aa = (a / armLength);
    std::cout << "aa: " << aa << std::endl;

    return aa * MOI;
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

    std::cout << "Torque command for motor 1: " << torque_command[0] << "Nm" << std::endl;
    std::cout << "Torque command for motor 2: " << torque_command[1] << "Nm" << std::endl;
    std::cout << "Press Ctrl+C to Stop Test" << std::endl;

    // Main loop
    while (!ctrl_c_pressed)
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

    std::cout << "Entering fault mode!" << std::endl;

    ::usleep(50000);

    for (auto &c : controllers)
    {
        c->SetStop();
    }

    double scaleWeight;
    std::cout << "Enter scale reading (grams): ";
    std::cin >> scaleWeight;

    double pracTorque = getPracTorque(scaleWeight);
    std::cout << "Practical Torque Output: " << pracTorque << "Nm" << std::endl;

    double pracTorque2 = getPracTorqueFromInertia(scaleWeight);
    std::cout << "Practical Torque Output Inertia Method: " << pracTorque2 << "Nm" << std::endl;

    return 0;
}
