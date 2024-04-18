#include <stdio.h>
#include <unistd.h>

#include <iostream>
#include <optional>
#include <boost/optional.hpp>
#include <csignal> // For signal handling
#include "moteus.h"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/sample-models.hpp"

// Global flag for indicating if Ctrl+C was pressed
volatile sig_atomic_t ctrl_c_pressed = 0;

// Signal handler function
void signalHandler(int signal)
{
    ctrl_c_pressed = 1; // Set flag to indicate Ctrl+C was pressed
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

    SE3 Tlink(SE3::Matrix3::Identity(), SE3::Vector3(0, 0, 0.195));  // 0.15 is the egs arm length in metres
    Inertia Ilink1(kFudge * 0.36, Tlink.translation(),              // 0.29 is the egs 1st arm weight in kgs
                   Inertia::Matrix3::Identity() * 0.001);
    Inertia Ilink2(kFudge * 0.21, Tlink.translation(),  // 0.28 is the egs 2nd arm weight in kgs
                   Inertia::Matrix3::Identity() * 0.001);

    // Setting limits
    CV qmin = CV::Constant(0);    // position min radians
    CV qmax = CV::Constant(360 * M_PI /180);  // position max radians
    TV vmax = CV::Constant(0.2);    // velocity max radians/sec
    TV taumax = CV::Constant(10);  // torque max nm

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

}  // namespace

int main(int argc, char** argv) {
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
    double desired_position_rad = desired_position_deg * M_PI / 180.0;

    Eigen::VectorXd q_desired(2);  // Desired joint positions
    q_desired << desired_position_rad, desired_position_rad;

    Eigen::VectorXd q(2);  // Current joint positions
    Eigen::VectorXd v(2);  // Current joint velocities
    Eigen::VectorXd a(2);  // Current joint accelerations

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
    cmd.feedforward_torque = 0.0;

    double torque_command[2] = {};
    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;

    int missed_replies = 0;
    int status_count = 0;
    constexpr int kStatusPeriod = 100;

    while (!ctrl_c_pressed) {
        ::usleep(10);

        // Set current joint positions to desired positions
        auto q = q_desired;

        // Assume zero joint velocities and accelerations for simplicity
        v.setZero();
        a.setZero();

        // Calculate inverse dynamics torques
        const Eigen::VectorXd& tau = CalculateTorques(model, q, v, a);

        // Display calculated torques
        std::cout << "Calculated Torques (Nm): "
                  << "Tau1: " << tau(0) << ", Tau2: " << tau(1) << std::endl;

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

        torque_command[0] = tau(0);
        torque_command[1] = tau(1);

        status_count++;
        if (status_count > kStatusPeriod) {
            printf("Mode: %2d/%2d  position: %6.3f/%6.3f  torque: %6.3f/%6.3f  temp: %4.1f/%4.1f  \r",
                   static_cast<int>(v1.mode), static_cast<int>(v2.mode),
                   v1.position, v2.position,
                   torque_command[0], torque_command[1],
                   v1.temperature, v2.temperature);
            fflush(stdout);

            status_count = 0;
        }
    }

    printf("Entering fault mode!\n");

    while (true) {
        ::usleep(50000);

        for (auto& c : controllers) {
            c->SetBrake();
        }
    }

    return 0;
}