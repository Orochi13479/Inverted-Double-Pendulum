#include <stdio.h>
#include <unistd.h>

#include <boost/optional.hpp>
#include <csignal>  // For signal handling
#include <iostream>

#include "moteus.h"

// Global flag for indicating if Ctrl+C was pressed
volatile sig_atomic_t ctrl_c_pressed = 0;

// Signal handler function
void signalHandler(int signal) {
    ctrl_c_pressed = 1;  // Set flag to indicate Ctrl+C was pressed
}

double degreesToRevolutions(double degrees) {
    const double degreesPerRevolution = 360.0;
    return degrees / degreesPerRevolution;
}

double revolutionsToDegrees(double revolutions) {
    const double degreesPerRevolution = 360.0;
    return revolutions * degreesPerRevolution;
}

boost::optional<mjbots::moteus::Query::Result> FindServo(
    const std::vector<mjbots::moteus::CanFdFrame> &frames,
    int id) {
    for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
        if (it->source == id) {
            return mjbots::moteus::Query::Parse(it->data, it->size);
        }
    }
    return {};
}

double convertRevolutionsToTorque(double revolutions) {
    // Example parameters for torque calculation
    const double arm_length = 0.195;  // Length of the arm in meters
    const double mass = 0.36;         // Mass in kilograms
    const double gravity = 9.81;      // Acceleration due to gravity in m/s^2

    // Moment of inertia calculation (assuming a simple pendulum)
    const double moment_of_inertia = (mass * arm_length * arm_length) / 3.0;

    // Torque calculation based on the equation: Torque = Moment of Inertia * Angular Acceleration
    // Angular acceleration can be calculated as the second derivative of position with respect to time
    // However, since we are only given revolutions (position), we'll assume a constant angular velocity for simplicity
    // You may need to adjust this calculation based on the specific dynamics of your system

    // For simplicity, let's assume a constant angular velocity (1 revolution per second)
    const double angular_velocity = 2 * M_PI;  // 1 revolution per second in radians per second

    // Torque calculation
    const double torque = moment_of_inertia * angular_velocity;

    return torque;
}

int main(int argc, char **argv) {
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
        std::make_shared<moteus::Controller>([&]() {
            auto options = options_common;
            options.id = 1;
            return options; }()),
        std::make_shared<moteus::Controller>([&]() {
            auto options = options_common;
            options.id = 2;
            return options; }()),
    };

    for (auto &c : controllers) {
        c->SetStop();
    }

    moteus::PositionMode::Command cmd;
    cmd.kp_scale = 10.0;
    cmd.kd_scale = 7.5;
    cmd.velocity = 0.5;
    cmd.feedforward_torque = 0.0;

    int missed_replies = 0;
    int status_count = 0;
    constexpr int kStatusPeriod = 100;

    double desired_position_deg;
    std::cout << "ENTER DESIRED END-EFFECTOR POSITION IN DEGREES (MAKE SURE IT IS CALIBRATED): ";
    std::cin >> desired_position_deg;

    // Convert desired position from degrees to revolutions
    double desired_position_rev = degreesToRevolutions(desired_position_deg);

    // Calculate required torque based on the desired position
    double required_torque = convertRevolutionsToTorque(desired_position_rev);

    std::cout << "Required Torque for desired position: " << required_torque << "Nm" << std::endl;
    double torque_command[2] = {};

    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;

    std::cout << "Torque command for motor 1: " << torque_command[0] << "Nm" << std::endl;
    std::cout << "Torque command for motor 2: " << torque_command[1] << "Nm" << std::endl;
    std::cout << "Press Ctrl+C to Stop Test" << std::endl;

    // Main loop
    while (!ctrl_c_pressed) {
        ::usleep(10);

        send_frames.clear();
        receive_frames.clear();

        for (size_t i = 0; i < controllers.size(); i++) {
            cmd.feedforward_torque = torque_command[i];
            send_frames.push_back(controllers[i]->MakePosition(cmd));
        }

        // Send frames
        transport->BlockingCycle(&send_frames[0], send_frames.size(), &receive_frames);

        auto maybe_servo1 = FindServo(receive_frames, 1);
        auto maybe_servo2 = FindServo(receive_frames, 2);

        if (!maybe_servo1 || !maybe_servo2) {
            missed_replies++;
            if (missed_replies > 3) {
                printf("\n\nServo not responding 1=%d 2=%d\n",
                       maybe_servo1 ? 1 : 0,
                       maybe_servo2 ? 1 : 0);
                break;
            }
            continue;
        } else {
            missed_replies = 0;
        }

        const auto &v1 = *maybe_servo1;
        const auto &v2 = *maybe_servo2;

        torque_command[0] = required_torque;
        torque_command[1] = required_torque;

        status_count++;
        if (status_count > kStatusPeriod) {
            printf("MODE: %2d/%2d  POSITION IN DEGREES: %6.3f  TORQUE: %6.3f/%6.3f  TEMP: %4.1f/%4.1f  VELOCITY: %6.3f/%6.3f\r",
                   static_cast<int>(v1.mode), static_cast<int>(v2.mode),
                   revolutionsToDegrees(v1.position + v2.position),
                   v1.torque, v2.torque,
                   v1.temperature, v2.temperature, v1.velocity, v2.velocity);
            fflush(stdout);

            status_count = 0;
        }
    }

    std::cout << "Entering fault mode!" << std::endl;

    ::usleep(50000);

    for (auto &c : controllers) {
        c->SetStop();
    }

    return 0;
}
