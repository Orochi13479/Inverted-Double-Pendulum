#include <stdio.h>
#include <unistd.h>

#include <boost/optional.hpp>
#include <chrono>
#include <csignal>  // For signal handling
#include <iostream>
#include <optional>

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

    constexpr double kFudge = 0.95;

    SE3 Tlink(SE3::Matrix3::Identity(), SE3::Vector3(0, 0, 0.195));  // 0.195 is the arm length in metres
    Inertia Ilink1(kFudge * 0.36, Tlink.translation(),               // 0.36 is the 1st arm weight in kgs
                   Inertia::Matrix3::Identity() * 0.001);
    Inertia Ilink2(kFudge * 0.21, Tlink.translation(),  // 0.21 is the 2nd arm weight in kgs
                   Inertia::Matrix3::Identity() * 0.001);

    // Setting limits
    CV qmin = CV::Constant(0);                 // position min radians
    CV qmax = CV::Constant(360 * M_PI / 180);  // position max radians
    TV vmax = CV::Constant(0.01);              // velocity max radians/sec
    TV taumax = CV::Constant(2.0);             // torque max nm

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

double WrapAround0(double v) {
    const auto v1 = std::fmod(v, 1.0);
    const auto v2 = (v1 < 0.0) ? (v1 + 1.0) : v1;
    return v2 > 0.5 ? (v2 - 1.0) : v2;
}

Eigen::VectorXd CalculateTorques(const pinocchio::Model& model,
                                 const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& v,
                                 const Eigen::VectorXd& a) {
    pinocchio::Data data(model);
    return pinocchio::rnea(model, data, q, v, a);
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

// double degreesToRevolutions(double degrees) {
//     const double degreesPerRevolution = 360.0;
//     return degrees / degreesPerRevolution;
// }

double revolutionsToDegrees(double revolutions) {
    const double degreesPerRevolution = 360.0;
    return revolutions * degreesPerRevolution;
}

double LinearRamp(double start_torque, double end_torque, double current_time, double ramp_duration) {
    if (current_time < ramp_duration) {
        return start_torque + (end_torque - start_torque) * (current_time / ramp_duration);
    } else {
        return end_torque;
    }
}

}  // namespace

int main(int argc, char** argv) {
    // Set up signal handler for Ctrl+C (SIGINT)
    std::signal(SIGINT, signalHandler);

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
    // double desired_position_rev = degreesToRevolutions(desired_position_deg);

    Eigen::VectorXd q_desired(2);  // Desired joint positions
    q_desired << desired_position_rad, desired_position_rad;
    // q_desired << desired_position_rev, desired_position_rev;

    // Eigen::VectorXd q(2);  // Current joint positions
    // Eigen::VectorXd v(2);  // Current joint velocities
    // Eigen::VectorXd a(2);  // Current joint accelerations

    Eigen::VectorXd q = randomConfiguration(model);  // in rad
    Eigen::VectorXd v = Eigen::VectorXd::Zero(2);    // in rad/s
    Eigen::VectorXd a = Eigen::VectorXd::Zero(2);    // in rad/s²

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
    cmd.kp_scale = 750.0;
    cmd.kd_scale = 750;
    cmd.velocity_limit = 0.005;
    // cmd.accel_limit = 0;
    cmd.feedforward_torque = 0.0;
    cmd.maximum_torque = 2.0;

    double torque_command[2] = {};
    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;

    int missed_replies = 0;
    int status_count = 0;
    constexpr int kStatusPeriod = 100;

    // Set current joint positions to desired positions
    auto q_current = q_desired;

    // Assume zero joint velocities and accelerations for simplicity
    v.setZero();
    a.setZero();

    // Calculate inverse dynamics torques
    const Eigen::VectorXd& tau = CalculateTorques(model, q_current, v, a);

    constexpr double torque_ramp_duration = 5.0;  // Duration of the torque ramp in seconds
    double ramp_start_time = 0.0;                 // Start time of the torque ramp

    std::chrono::high_resolution_clock::time_point start_time;
    double current_time = 0.0;

    // Display calculated torques
    std::cout << "CALCULATED TORQUES (Nm): "
              << "MOTOR 1: " << tau(0) << ", MOTOR 2: " << tau(1) << std::endl;

    std::cout << "Press Ctrl+C to Stop Test" << std::endl;

    while (!ctrl_c_pressed) {
        ::usleep(10);
        start_time = std::chrono::high_resolution_clock::now();

        send_frames.clear();
        receive_frames.clear();

        for (size_t i = 0; i < controllers.size(); i++) {
            cmd.feedforward_torque = torque_command[i];
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

        q(0) = WrapAround0(v1.position + 0.5) * 2 * M_PI;
        q(1) = WrapAround0(v2.position) * 2 * M_PI;

        torque_command[0] = LinearRamp(0.0, tau(0), current_time - ramp_start_time, torque_ramp_duration);
        torque_command[1] = LinearRamp(0.0, tau(1), current_time - ramp_start_time, torque_ramp_duration);

        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = now - start_time;
        current_time = elapsed_seconds.count();

        if (current_time - ramp_start_time >= torque_ramp_duration) {
            ramp_start_time = current_time;  // Reset the start time for the next ramp
        }

        status_count++;
        if (status_count > kStatusPeriod) {
            printf("MODE: %2d/%2d  POSITION IN DEGREES: %6.3f  TORQUE: %6.3f/%6.3f  TEMP: %4.1f/%4.1f  \r",
                   static_cast<int>(v1.mode), static_cast<int>(v2.mode),
                   revolutionsToDegrees(v1.position + v2.position),
                   v1.torque, v2.torque,
                   v1.temperature, v2.temperature);
            fflush(stdout);

            status_count = 0;
        }
    }

    std::cout << "Entering fault mode!" << std::endl;

    ::usleep(50000);

    for (auto& c : controllers) {
        c->SetStop();
    }

    return 0;
}