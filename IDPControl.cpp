// PLACEHOLDER

#include <unistd.h>
#include <stdio.h>

#include <iostream>
#include <optional>

#include "moteus.h"

#include <fstream>
#include <sstream>

// USING THIS FOR TESTING RN, DIDNT WANNA BREAK MAIN

int main(int argc, char **argv)
{
    using namespace mjbots;
    moteus::Controller::DefaultArgProcess(argc, argv);
    auto transport = moteus::Controller::MakeSingletonTransport({});

    moteus::Controller::Options options_common;

    auto &pf = options_common.position_format;
    pf.position = moteus::kIgnore;
    pf.velocity = moteus::kIgnore;
    pf.feedforward_torque = moteus::kFloat;
    // pf.kp_scale = moteus::kInt8;
    // pf.kd_scale = moteus::kInt8;

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
    // cmd.kp_scale = 0.0;
    // cmd.kd_scale = 0.0;
    // cmd.feedforward_torque = 0.0;

    double torque_command[2] = {};
    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;

    return 0;
}