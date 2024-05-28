#include <stdio.h>
#include <unistd.h>

#include <boost/optional.hpp>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "moteus.h"

// mjbots::moteus::MultiplexParser parser;

// Global flag for indicating if Ctrl+C was pressed
volatile sig_atomic_t ctrl_c_pressed = 0;

// Signal handler function
void signalHandler(int signal) {
    ctrl_c_pressed = 1;  // Set flag to indicate Ctrl+C was pressed
}

// Arrays to store data for each column
std::vector<float> timestamp, q1, q1_dot, q1_dot_dot, tau1, q2, q2_dot, q2_dot_dot, tau2;

void readCSV(const std::string& filename) {
    std::string filepath = "../trajGen/" + filename;

    // Open the file
    std::ifstream file(filepath);

    if (!file.is_open()) {
        throw std::runtime_error("Error: Unable to open file " + filepath);
    }

    // Skip the first line (column headings)
    std::string line;
    std::getline(file, line);

    // Read and process the CSV data
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        float t, q1_val, q1_dot_val, q1_dot_dot_val, tau1_val, q2_val, q2_dot_val, q2_dot_dot_val, tau2_val;
        char comma;
        if (iss >> t >> comma >> q1_val >> comma >> q1_dot_val >> comma >> q2_val >> comma >> q2_dot_val) {
            // Add data to arrays
            timestamp.push_back(t);
            q1.push_back(q1_val);
            q1_dot.push_back(q1_dot_val);
            q1_dot_dot.push_back(q1_dot_dot_val);
            tau1.push_back(tau1_val);
            q2.push_back(q2_val);
            q2_dot.push_back(q2_dot_val);
            q2_dot_dot.push_back(q2_dot_dot_val);
            tau2.push_back(tau2_val);
        }
    }
}

boost::optional<mjbots::moteus::Query::Result> FindServo(
    const std::vector<mjbots::moteus::CanFdFrame>& frames,
    int id) {
    for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
        if (it->source == id) {
            return mjbots::moteus::Query::Parse(it->data, it->size);
        }
    }
    return {};
}

// boost::optional<double> TorqueError(
//     const std::vector<mjbots::moteus::CanFdFrame>& frames,
//     int id) {
//     for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
//         if (it->source == id) {
            // auto result = mjbots::moteus::Query::ParseGeneric(&parser, mjbots::moteus::Register::kControlTorqueError, mjbots::moteus::Resolution::kFloat);
//             return result;
//         }
//     }
// }

double revolutionsToDegrees(double revolutions) {
    const double degreesPerRevolution = 360.0;
    return revolutions * degreesPerRevolution;
}

int main(int argc, char** argv) {
    // Set up signal handler for Ctrl+C (SIGINT)
    std::signal(SIGINT, signalHandler);

    using namespace mjbots;
    moteus::Controller::DefaultArgProcess(argc, argv);
    auto transport = moteus::Controller::MakeSingletonTransport({});

    // Specify the full path to the CSV file
    std::string filename = "trajectory_data.csv";

    readCSV(filename);

    moteus::Controller::Options options_common;

    auto& pf = options_common.position_format;
    pf.position = moteus::kIgnore;
    pf.velocity = moteus::kIgnore;
    pf.feedforward_torque = moteus::kFloat;
    pf.kp_scale = moteus::kInt8;
    pf.kd_scale = moteus::kInt8;

    std::vector<std::shared_ptr<moteus::Controller>> controllers = {
        std::make_shared<moteus::Controller>([&]() {
            auto options = options_common;
            options.id = 1;
            return options;
        }()),
        std::make_shared<moteus::Controller>([&]() {
            auto options = options_common;
            options.id = 2;
            return options;
        }()),
    };

    for (auto& c : controllers) {
        c->SetStop();
    }

    moteus::PositionMode::Command cmd;
    cmd.kp_scale = 5.0;
    cmd.kd_scale = 1.5;
    // cmd.velocity_limit = 0.1;
    cmd.feedforward_torque = 0.0;
    // cmd.velocity = 1.0;
    // cmd.accel_limit = 0;
    // cmd.maximum_torque = 2.0;

    double torque_command[2] = {};
    double kp_scale[2] = {};
    double kd_scale[2] = {};
    double velocity[2] = {};
    std::vector<double> torque_error1;
    std::vector<double> torque_error2;

    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;

    int missed_replies = 0;
    int status_count = 0;
    constexpr int kStatusPeriod = 100;

    while (!ctrl_c_pressed) {
        for (std::size_t i = 0; i < timestamp.size(); ++i) {
            ::usleep(10);
            torque_command[0] = tau1[i];
            torque_command[1] = tau2[i];
            velocity[0] = q1_dot[i];
            velocity[1] = q2_dot[i];

            send_frames.clear();
            receive_frames.clear();

            for (size_t i = 0; i < controllers.size(); i++) {
                cmd.feedforward_torque = torque_command[i];
                cmd.kp_scale = kp_scale[i];
                cmd.kd_scale = kd_scale[i];
                cmd.velocity = velocity[i];
                send_frames.push_back(controllers[i]->MakePosition(cmd));
            }

            transport->BlockingCycle(
                &send_frames[0], send_frames.size(),
                &receive_frames);

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

            const auto& v1 = *maybe_servo1;
            const auto& v2 = *maybe_servo2;

            // auto motor1_torque_error = TorqueError(receive_frames, 1);
            // auto motor2_torque_error = TorqueError(receive_frames, 2);

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

            std::cout << "Entering fault mode!" << std::endl;

            ::usleep(50000);

            for (auto& c : controllers) {
                c->SetStop();
            }
        }

        return 0;
    }
}
