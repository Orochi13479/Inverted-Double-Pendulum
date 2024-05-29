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
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/sample-models.hpp"

// Global flag for indicating if Ctrl+C was pressed
volatile sig_atomic_t ctrl_c_pressed = 0;

// Signal handler function
void signalHandler(int signal) {
    ctrl_c_pressed = 1;  // Set flag to indicate Ctrl+C was pressed
}

// Define a struct to hold data for each timestamp
struct DataPoint {
    std::chrono::system_clock::time_point timestamp;
    double position;
    double torque1;
    double torque2;
    double velocity1;
    double velocity2;
};

namespace {
template <typename Scalar, int Options,
          template <typename, int> class JointCollectionTpl>
void BuildModel(pinocchio::ModelTpl<Scalar, Options, JointCollectionTpl>* model) {
    using namespace pinocchio;

    using M = Model;
    using JC = JointCollectionTpl<Scalar, Options>;
    using CV = typename JC::JointModelRX::ConfigVector_t;
    using TV = typename JC::JointModelRX::TangentVector_t;

    M::JointIndex idx = 0;

    double armLength = 0.195;
    double motorRadius = 0.035;
    double armMass = 0.12;
    double motorMass = 0.24;
    double secondArmMass = 0.21;

    double MOI = ((1 / 3) * armMass * pow(armLength, 2)) +
                 ((1 / 2) * motorMass * pow(motorRadius, 2)) +
                 (motorMass * pow(armLength + motorRadius, 2));

    double I_arm1 = (1.0 / 3.0) * armMass * pow(armLength, 2);
    double I_motor1 = (1.0 / 2.0) * motorMass * pow(motorRadius, 2);
    double I_arm2 = (1.0 / 3.0) * armMass * pow(armLength, 2);
    double I_motor2 = (1.0 / 2.0) * motorMass * pow(motorRadius, 2);
    double I_weight = motorMass * pow(armLength + motorRadius, 2);

    SE3 Tlink(SE3::Matrix3::Identity(), SE3::Vector3(0, 0, armLength));
    Inertia Ilink1((armMass + motorMass), Tlink.translation(),
                   Inertia::Matrix3::Identity() * MOI);
    Inertia Ilink2(secondArmMass, Tlink.translation(),
                   Inertia::Matrix3::Identity() * (I_arm2 + I_motor2 + I_weight));

    // Setting limits
    CV qmin = CV::Constant(-360 * M_PI / 180);  // position min radians
    CV qmax = CV::Constant(360 * M_PI / 180);   // position max radians
    TV vmax = CV::Constant(10);                 // velocity max radians/sec
    TV taumax = CV::Constant(1.5);              // torque max nm

    idx = model->addJoint(idx, typename JC::JointModelRY(), Tlink,
                          "link1_joint", taumax, vmax, qmin, qmax);
    model->appendBodyToJoint(idx, Ilink1);
    model->addJointFrame(idx);
    model->addBodyFrame("link1_body", idx);

    idx = model->addJoint(idx, typename JC::JointModelRY(), Tlink,
                          "link2_joint", taumax, vmax, qmin, qmax);
    model->appendBodyToJoint(idx, Ilink2);
    model->addJointFrame(idx);
    model->addBodyFrame("link2_body", idx);
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

double revolutionsToDegrees(double revolutions) {
    const double degreesPerRevolution = 360.0;
    return revolutions * degreesPerRevolution;
}

}  // namespace

int main(int argc, char** argv) {
    // Set up signal handler for Ctrl+C (SIGINT)
    std::signal(SIGINT, signalHandler);

    std::vector<DataPoint> dataPoints;  // Vector to store data points

    std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();

    using namespace mjbots;
    moteus::Controller::DefaultArgProcess(argc, argv);
    auto transport = moteus::Controller::MakeSingletonTransport({});

    pinocchio::Model model;
    BuildModel(&model);
    pinocchio::Data data(model);

    double desired_position_deg;
    std::cout << "ENTER DESIRED END-EFFECTOR POSITION IN DEGREES (MAKE SURE IT IS CALIBRATED): ";
    std::cin >> desired_position_deg;

    // Convert desired position from degrees to radians
    double desired_position_rad = -desired_position_deg * M_PI / 180.0;

    Eigen::VectorXd q_desired(2);  // Desired joint positions
    q_desired << desired_position_rad, desired_position_rad;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nv);  // in rad
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);  // in rad/s
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);  // in rad/s²

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
    cmd.kp_scale = 0.0;
    cmd.kd_scale = 0.0;
    // cmd.velocity = 1.0;
    // cmd.velocity_limit = 0.1;
    // cmd.accel_limit = 0;
    cmd.feedforward_torque = 0.0;
    // cmd.maximum_torque = 2.0;

    double torque_command[2] = {};
    double velocity_count[2] = {};
    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;

    int missed_replies = 0;
    int status_count = 0;
    constexpr int kStatusPeriod = 100;

    // Set current joint positions to desired positions
    auto q_current = q_desired;

    // // Assume zero current joint velocities and accelerations for simplicity
    v.setZero();
    a.setZero();

    // // Calculate inverse dynamics torques
    const Eigen::VectorXd& tau = pinocchio::rnea(model, data, q_current, v, a);

    // Display calculated torques
    std::cout << "CALCULATED TORQUES (Nm): "
              << "MOTOR 1: " << tau(0) << ", MOTOR 2: " << tau(1) << std::endl;

    std::cout << "Press Ctrl+C to Stop Test" << std::endl;

    double kp = 5.0;
    double kd = 1.5;
    float prev_error1 = 0;
    float prev_error2 = 0;
    float control_signal1;
    float control_signal2;
    float derivative1;
    float derivative2;

    while (!ctrl_c_pressed) {
        // Get current time
        std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();
        // Calculate elapsed time
        std::chrono::duration<double> elapsedTime = currentTime - startTime;
        double deltaTime = elapsedTime.count();

        DataPoint currentDataPoint;
        currentDataPoint.timestamp = currentTime;

        ::usleep(10);

        send_frames.clear();
        receive_frames.clear();

        for (size_t i = 0; i < controllers.size(); i++) {
            cmd.feedforward_torque = torque_command[i];
            cmd.velocity = velocity_count[i];
            // cmd.kp_scale = 5.0;
            // cmd.kd_scale = 1.5;
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

        // torque_command[0] = tau(0);
        // torque_command[1] = tau(1);

        double torque_check1 = tau(0) - v1.torque;
        double torque_check2 = tau(1) - v2.torque;

        if (torque_check1 < 0 || torque_check1 > 0) {
            derivative1 = (deltaTime > 0) ? (torque_check1 - prev_error1) / deltaTime : 0.0;
            control_signal1 = kp * torque_check1 + kd * derivative1;
            prev_error1 = torque_check1;
            torque_command[0] = tau(0) + control_signal1;
        }

        if (torque_check2 < 0 || torque_check2 > 0) {
            derivative2 = (deltaTime > 0) ? (torque_check2 - prev_error2) / deltaTime : 0.0;
            control_signal2 = kp * torque_check2 + kd * derivative2;
            prev_error2 = torque_check2;
            torque_command[1] = tau(1) + control_signal2;
        }

        currentDataPoint.position = revolutionsToDegrees(v1.position + v2.position);
        currentDataPoint.torque1 = v1.torque;
        currentDataPoint.torque2 = v2.torque;
        currentDataPoint.velocity1 = v1.velocity;
        currentDataPoint.velocity2 = v2.velocity;

        dataPoints.push_back(currentDataPoint);

        status_count++;
        if (status_count > kStatusPeriod) {
            printf("MODE: %2d/%2d POSITION IN DEGREES: %6.3f TORQUE: %6.3f/%6.3f VELOCITY: %6.3f/%6.3f CONTROL_SIGNAL: %6.3f/%6.3f\r",
                   static_cast<int>(v1.mode), static_cast<int>(v2.mode),
                   revolutionsToDegrees(v1.position + v2.position),
                   v1.torque, v2.torque,
                   v1.velocity, v2.velocity, control_signal1, control_signal2);
            fflush(stdout);

            status_count = 0;
        }
    }
    std::string outputDirectory = "../control/";
    // Save data to CSV file
    std::string outputFilePath = outputDirectory + "test_position1.csv";  // Construct full path

    // Save data to CSV file
    std::ofstream outputFile(outputFilePath);
    if (outputFile.is_open()) {
        // Write CSV header
        outputFile << "Timestamp,pos,tau1,tau2,vel1,vel2\n";
        // Write data to CSV
        for (const auto& dataPoint : dataPoints) {
            std::time_t timestamp = std::chrono::system_clock::to_time_t(dataPoint.timestamp);
            // Format timestamp as string
            std::ostringstream timestampStr;
            timestampStr << std::put_time(std::localtime(&timestamp), "%Y-%m-%d %H:%M:%S");
            // Write data to CSV
            outputFile << timestampStr.str() << ","
                       << dataPoint.position << ","
                       << dataPoint.torque1 << ","
                       << dataPoint.torque2 << ","
                       << dataPoint.velocity1 << ","
                       << dataPoint.velocity2 << "\n";
        }
        outputFile.close();
        std::cout << "Data saved to data.csv" << std::endl;
    } else {
        std::cerr << "Error: Unable to open file for writing" << std::endl;
    }

    std::cout << "Entering fault mode!" << std::endl;

    ::usleep(50000);

    for (auto& c : controllers) {
        c->SetStop();
    }

    return 0;
}