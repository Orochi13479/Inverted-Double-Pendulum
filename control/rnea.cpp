#include <stdio.h>
#include <unistd.h>

#include <boost/optional.hpp>
#include <chrono>
#include <csignal>  // For signal handling
#include <iostream>
#include <optional>
#include <fstream>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/sample-models.hpp"

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

}

int main(int argc, char** argv) {
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

    // Set current joint positions to desired positions
    auto q_current = q_desired;

    // // Assume zero current joint velocities and accelerations for simplicity
    v.setZero(); // Derivative of position
    a.setZero(); //Derivative of velocity

    // // Calculate inverse dynamics torques
    const Eigen::VectorXd& tau = pinocchio::rnea(model, data, q_current, v, a);

    // Display calculated torques
    std::cout << "CALCULATED TORQUES (Nm): "
              << "MOTOR 1: " << tau(0) << ", MOTOR 2: " << tau(1) << std::endl;

}