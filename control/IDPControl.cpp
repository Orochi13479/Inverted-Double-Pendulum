// Test Plan 1 & 2

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

double getPracTorque(double scaleWeight) {
    double armLength = 0.195;

    return ((scaleWeight / 9.81) * armLength) / 10.0;
}

double getPracTorqueFromInertia(double scaleWeight) {
    double armLength = 0.195;
    double motorRadius = 0.035;
    double armMass = 0.12;
    double motorMass = 0.24;

    // Moment of Inertia for rod with disk attached at the end
    double MOI = ((1 / 3) * armMass * pow(armLength, 2)) +
                 ((1 / 2) * motorMass * pow(motorRadius, 2)) +
                 (motorMass * pow(armLength + motorRadius, 2));

    // Linear acceleration
    double a = (scaleWeight / 1000 * 9.81) / (0.26);  // 0.26kg is setup based. This WILL change with different Setups

    // Angular acceleration
    double aa = (a / armLength);

    return aa * MOI;
}

double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

// Calculate torque based on desired position using proportional control
std::pair<double, double> calculateTorqueFromSystemPosition(
    double desired_system_position_deg,
    double current_position_motor1_deg,
    double current_position_motor2_deg,
    double kp) {
    // Calculate current system position
    double current_system_position_deg = current_position_motor1_deg + current_position_motor2_deg;

    // Calculate system position error in degrees
    double system_position_error_deg = desired_system_position_deg - current_system_position_deg;

    // Convert position error to radians
    double system_position_error_rad = degreesToRadians(system_position_error_deg);

    // Calculate weighted torques for each motor based on their positions
    double torque_motor1 = kp * system_position_error_rad * (current_position_motor2_deg / current_system_position_deg);
    double torque_motor2 = kp * system_position_error_rad * (current_position_motor1_deg / current_system_position_deg);

    return {torque_motor1, torque_motor2};
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

double revolutionsToDegrees(double revolutions) {
    const double degreesPerRevolution = 360.0;
    return revolutions * degreesPerRevolution;
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

    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;

    moteus::PositionMode::Command cmd;
    cmd.kp_scale = 10.0;
    cmd.kd_scale = 7.5;
    cmd.feedforward_torque = 0.0;

    double torque_command[2] = {};

    double desired_position_deg;
    std::cout << "ENTER DESIRED END-EFFECTOR POSITION IN DEGREES (MAKE SURE IT IS CALIBRATED): ";
    std::cin >> desired_position_deg;

    double current_position_motor1_deg = 0.0;  // Get current position for motor 1
    double current_position_motor2_deg = 0.0;  // Get current position for motor 2

    // Calculate torque based on desired system position
    auto torques = calculateTorqueFromSystemPosition(
        desired_position_deg,
        current_position_motor1_deg,
        current_position_motor2_deg,
        0.1);  // kp

    std::cout << "Torque command for motor 1: " << torques.first << "Nm" << std::endl;
    std::cout << "Torque command for motor 2: " << torques.second << "Nm" << std::endl;
    std::cout << "Press Ctrl+C to Stop Test" << std::endl;

    int missed_replies = 0;
    int status_count = 0;
    constexpr int kStatusPeriod = 100;

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

        torque_command[0] = torques.first;
        torque_command[1] = torques.second;

        status_count++;
        if (status_count > kStatusPeriod) {
            printf("MODE: %2d/%2d  POSITION IN DEGREES: %6.3f/%6.3f  TORQUE: %6.3f/%6.3f  TEMP: %4.1f/%4.1f  VELOCITY: %6.3f/%6.3f\r",
                   static_cast<int>(v1.mode), static_cast<int>(v2.mode),
                   revolutionsToDegrees(v1.position),
                   revolutionsToDegrees(v2.position),
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
