// PLACEHOLDER

#include <unistd.h>
#include <stdio.h>

#include <iostream>
#include <optional>

#include "moteus.h"

#include <fstream>
#include <sstream>

// USING THIS FOR TESTING RN, DIDNT WANNA BREAK MAIN

double PositionCalculation(double desired_position_degrees, double feedback_position_degrees, 
                         double feedback_velocity, double dt) {
        // Convert desired position to radians
        double desired_position = desired_position_degrees * M_PI / 180.0;

        // Trajectory follower (simplified)
        double acceleration = 0.0;
        double control_velocity = 0.0; // OR control_velocity + acceleration * dt OR 0.0
        double control_position = feedback_position_degrees * M_PI / 180.0; // OR control_position + control_velocity * dt

        // Position and velocity errors
        double position_error = control_position - desired_position;
        double velocity_error = control_velocity - feedback_velocity;

        // Position integrator
        position_integrator = limit(position_integrator + ki * position_error * dt, ilimit);

        // Calculate torque command
        double torque = position_integrator +
                        kp * kp_scale * position_error +
                        kd * kd_scale * velocity_error;

        return torque;
    }

int main() {

    using namespace mjbots;
    moteus::Controller::DefaultArgProcess(argc, argv);
    auto transport = moteus::Controller::MakeSingletonTransport({});

    moteus::Controller::Options options_common;

    auto &pf = options_common.position_format;
    pf.position = moteus::kIgnore;
    pf.velocity = moteus::kIgnore;
    pf.feedforward_torque = moteus::kFloat;
    pf.kp_scale = moteus::kInt8;
    pf.kd_scale = moteus::kInt8;

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

    double torque_command[2] = {};
    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;

    // Feedback from motor (position in degrees and velocity)
    double feedback_position_degrees = 0.0;
    double feedback_velocity = 0.0;

    // Desired position in degrees (0 degrees is downwards)
    double desired_position_degrees;
    std::cout << "ENTER DESIRED POSITION IN DEGREES (0 IS DOWNWARDS, MAKE SURE IT CALIBRATED FIRST): ";
    std::cin >> desired_position_degrees;

    // Time step (e.g., 0.01 seconds)
    double dt = 0.0;

    // Compute torque command
    double torque_command = PositionCalculation(desired_position_degrees, feedback_position_degrees, feedback_velocity, dt);

    // Output torque command
    std::cout << "Torque Command: " << torque_command << " Nm" << std::endl;

    return 0;
}