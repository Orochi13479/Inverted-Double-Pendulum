#include <iostream>
#include <unistd.h>
#include <csignal>
#include <chrono>
#include <vector>
#include <memory>
#include <cmath>

#include "moteus.h"

volatile sig_atomic_t ctrl_c_pressed = 0;

void signalHandler(int signal)
{
    ctrl_c_pressed = 1;
}

double inputAndLimitTorque(const std::string &motor_name, double max_torque)
{
    double torque_command;
    std::cout << "Enter torque command for " << motor_name << ": ";
    std::cin >> torque_command;
    if (torque_command < -max_torque)
    {
        std::cout << "TORQUE TOO HIGH. SETTING TO MAX TORQUE: " << -max_torque << "Nm" << std::endl;
        return -max_torque;
    }
    else if (torque_command > max_torque)
    {
        std::cout << "TORQUE TOO HIGH. SETTING TO MAX TORQUE: " << max_torque << "Nm" << std::endl;
        return max_torque;
    }
    return torque_command;
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signalHandler);
    using namespace mjbots;
    moteus::Controller::DefaultArgProcess(argc, argv);
    auto transport = moteus::Controller::MakeSingletonTransport({});
    moteus::Controller::Options options_common;
    auto &pf = options_common.position_format;
    pf.feedforward_torque = moteus::kFloat;
    pf.kp_scale = moteus::kInt8;
    pf.kd_scale = moteus::kInt8;

    // Create two controllers
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

    const double MAX_TORQUE = 0.2;
    double torque_command[] = {inputAndLimitTorque("motor 1", MAX_TORQUE), inputAndLimitTorque("motor 2", MAX_TORQUE)};
    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;

    using namespace std::chrono;
    auto start = steady_clock::now();
    int desired_frequency = 100;                  // Hz, the desired control loop frequency
    int sleep_time = 1000000 / desired_frequency; // microseconds
    int loop_count = 0;
    int missed_ticks = 0;

    while (!ctrl_c_pressed)
    {
        auto loop_start = steady_clock::now();

        send_frames.clear();
        receive_frames.clear();
        for (size_t i = 0; i < controllers.size(); i++)
        {
            moteus::PositionMode::Command cmd;
            cmd.feedforward_torque = torque_command[i];
            send_frames.push_back(controllers[i]->MakePosition(cmd));
        }
        transport->BlockingCycle(send_frames.data(), send_frames.size(), &receive_frames);

        auto loop_end = steady_clock::now();
        auto duration = duration_cast<microseconds>(loop_end - loop_start).count();

        int delay = sleep_time - duration;
        if (delay > 0)
        {
            usleep(delay);
        }
        else
        {
            missed_ticks++;
        }
        loop_count++;
    }

    auto end = steady_clock::now();
    auto total_time = duration_cast<seconds>(end - start).count();

    ::usleep(50000);

    for (auto &c : controllers)
    {
        c->SetStop();
    }

    std::cout << "Total runtime: " << total_time << " seconds\n";
    std::cout << "Total loops: " << loop_count << "\n";
    std::cout << "Missed ticks: " << missed_ticks << "\n";
    std::cout << "Exiting program." << std::endl;

    return 0;
}
