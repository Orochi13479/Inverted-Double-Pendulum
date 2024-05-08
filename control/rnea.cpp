#include "pinocchio/algorithm/rnea.hpp"

#include <stdio.h>
#include <unistd.h>

#include <Eigen/Core>
#include <boost/optional.hpp>
#include <chrono>
#include <csignal>  // For signal handling
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "pinocchio/algorithm/joint-configuration.hpp"
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

// Arrays to store data for each column
std::vector<float> timestamp;
std::vector<float> q1;
std::vector<float> q1_dot;
std::vector<float> q1_dot_dot;
std::vector<float> q2;
std::vector<float> q2_dot;
std::vector<float> q2_dot_dot;

void readCSV(const std::string& filename) {
    std::string filepath = "/home/student/git/Inverted-Double-Pendulum/trajGen/" + filename;

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
        float t, q1_val, q1_dot_val, q2_val, q2_dot_val;
        char comma;
        if (iss >> t >> comma >> q1_val >> comma >> q2_val >> comma >> q1_dot_val >> comma >> q2_dot_val) {
            // Add data to arrays
            timestamp.push_back(t);
            q1.push_back(q1_val);
            q2.push_back(q2_val);
            q1_dot.push_back(q1_dot_val);
            q2_dot.push_back(q2_dot_val);
        }
    }
}

float calculateAcceleration(const std::vector<float>& velocities, const std::vector<float>& timestamps) {
    float acceleration;
    for (size_t i = 1; i < timestamps.size(); ++i) {
        // Calculate delta time
        float dt = timestamps[i] - timestamps[i - 1];
        if (dt == 0) {
            // Avoid division by zero
            acceleration = 0.0;
        } else {
            // Calculate acceleration using velocity difference
            float dv = velocities[i] - velocities[i - 1];
            acceleration = dv / dt;
        }
    }

    return acceleration;
}

}  // namespace

int main(int argc, char** argv) {
    pinocchio::Model model;
    BuildModel(&model);
    pinocchio::Data data1(model);
    pinocchio::Data data2(model);

    std::string filename = "Hand_Desgined_traj_gen_draft1.csv";

    readCSV(filename);

    for (std::size_t i = 0; i < timestamp.size(); ++i) {
        float acceleration1;
        float acceleration2;
        float velocity1;
        float velocity2;
        float delta_time = timestamp[i] - timestamp[i - 1];
        if (delta_time == 0) {
            // Avoid division by zero
            acceleration1 = 0.0;
            acceleration2 = 0.0;
            q1_dot_dot.push_back(acceleration1);
            q2_dot_dot.push_back(acceleration2);
        } else {
            float delta_pos1 = q1[i] - q1[i - 1];
            float delta_pos2 = q2[i] - q2[i - 1];
            velocity1 = delta_pos1 / delta_time;
            velocity2 = delta_pos2 / delta_time;
            q1_dot.push_back(velocity1);
            q2_dot.push_back(velocity2);
            float dv1 = q1_dot[i] - q1_dot[i - 1];
            float dv2 = q2_dot[i] - q2_dot[i - 1];
            acceleration1 = dv1 / delta_time;
            acceleration2 = dv2 / delta_time;
            q1_dot_dot.push_back(acceleration1);
            q2_dot_dot.push_back(acceleration2);
        }
    }

    // Convert float values to Eigen::VectorXd
    std::vector<Eigen::VectorXd> pos1, pos2, v1, v2, a1, a2, torque1, torque2;

    for (std::size_t i = 0; i < timestamp.size(); ++i) {
        pos1.push_back(Eigen::VectorXd::Constant(1, static_cast<double>(q1[i])));
        pos2.push_back(Eigen::VectorXd::Constant(1, static_cast<double>(q2[i])));
        v1.push_back(Eigen::VectorXd::Constant(1, static_cast<double>(q1_dot[i])));
        v2.push_back(Eigen::VectorXd::Constant(1, static_cast<double>(q2_dot[i])));
        a1.push_back(Eigen::VectorXd::Constant(1, static_cast<double>(q1_dot_dot[i])));
        a2.push_back(Eigen::VectorXd::Constant(1, static_cast<double>(q2_dot_dot[i])));
    }

    // for (std::size_t i = 0; i < timestamp.size(); ++i) {
    //     torque1.push_back(pinocchio::rnea(model, data1, pos1[i], v1[i], a1[i]));
    //     torque2.push_back(pinocchio::rnea(model, data2, pos2[i], v2[i], a2[i]));
    // }

    // for (std::size_t i = 0; i < timestamp.size(); ++i) {
    //     std::cout << "timestamp: " << timestamp[i] << " ";
    //     std::cout << "pos1: " << pos1[i] << " ";
    //     std::cout << "v1: " << v1[i] << " ";
    //     std::cout << "a1: " << a1[i] << " ";
    //     std::cout << "pos2: " << pos2[i] << " ";
    //     std::cout << "v2: " << v2[i] << " ";
    //     std::cout << "a2: " << a2[i] << " ";
    //     std::cout << "torque1: " << torque1[i] << " ";
    //     std::cout << "torque2: " << torque2[i] << std::endl;
    // }

    return 0;
}