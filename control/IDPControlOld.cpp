// PLACEHOLDER

#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <optional>
#include <csignal> // For signal handling
#include "moteus.h"

#include <fstream>
#include <sstream>

// USING THIS FOR TESTING RN, DIDNT WANNA BREAK MAIN

volatile sig_atomic_t ctrl_c_pressed = 0;

// Signal handler function
void signalHandler(int signal)
{
    ctrl_c_pressed = 1; // Set flag to indicate Ctrl+C was pressed
}

double PositionCalculation(double desired_position_degrees, double feedback_position_degrees,
                           double feedback_velocity, double dt, double kp, double ki, double kd, double ilimit)
{
    // Convert desired position to radians
    double desired_position = desired_position_degrees * M_PI / 180.0;

    // Trajectory follower (simplified)
    double acceleration = 0.0;
    double control_velocity = 0.0;                                      // OR control_velocity + acceleration * dt OR 0.0
    double control_position = feedback_position_degrees * M_PI / 180.0; // OR control_position + control_velocity * dt

    // Position and velocity errors
    double position_error = control_position - desired_position;
    double velocity_error = control_velocity - feedback_velocity;

    // Position integrator
    double position_integrator = limit(position_integrator + ki * position_error * dt, ilimit);

    // Calculate torque command
    double torque = position_integrator +
                    kp * position_error +
                    kd * velocity_error;

    return torque;
}

int main(int argc, char **argv)
{
    // Desired position in degrees (0 degrees is downwards)
    double desired_position_degrees;
    std::cout << "ENTER DESIRED POSITION IN DEGREES (0 IS DOWNWARDS, MAKE SURE ITS CALIBRATED FIRST): ";
    std::cin >> desired_position_degrees;

    double kp;
    std::cout << "ENTER DESIRED KP SCALE:";
    std::cin >> kp;

    double kd;
    std::cout << "ENTER DESIRED KP SCALE:";
    std::cin >> kp;

    double ki;
    std::cout << "ENTER DESIRED KI SCALE:";
    std::cin >> ki;

    double ilimit;
    std::cout << "ENTER DESIRED ILIMIT SCALE:";
    std::cin >> ilimit;

    using namespace mjbots;
    moteus::Controller::DefaultArgProcess(argc, argv);
    auto transport = moteus::Controller::MakeSingletonTransport({});

    moteus::Controller::Options options_common;

    auto &pf = options_common.position_format;
    pf.position = moteus::kIgnore;
    pf.velocity = moteus::kIgnore;
    pf.feedforward_torque = moteus::kFloat;
    pf.kp_scale = moteus::kp;
    pf.kd_scale = moteus::kd;
    pf.ki_scale = moteus::ki;
    pf.ilimit = moteus::ilimit;

    moteus::PositionMode::Command cmd;
    cmd.kp_scale = 0.0;
    cmd.kd_scale = 0.0;
    cmd.feedforward_torque = 0.0;

    double torque_command[2] = {};
    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;

    // Feedback from motor (position in degrees and velocity)
    double feedback_position_degrees = 0.0;
    double feedback_velocity = 0.0;

    // Time step (e.g., 0.01 seconds)
    double dt = 0.0;

    // Compute torque command
    double torque = PositionCalculation(desired_position_degrees, feedback_position_degrees, 
                                        feedback_velocity, dt, kd, ki, kp, ilimit);

    torque_command[0] = torque;
    torque_command[1] = torque;

    std::cout << "TORQUE FOR MOTOR 1: " << torque_command[0] << "Nm" << std::endl;
    std::cout << "TORQUE FOR MOTOR 2: " << torque_command[1] << "Nm" << std::endl;
    std::cout << "PRESS CTRL+C TO STOP TEST" << std::endl;

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

    return 0;
}