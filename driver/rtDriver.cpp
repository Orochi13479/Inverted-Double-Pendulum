#include <cactus_rt/rt.h>
#include "moteus.h"

#include <iostream>
#include <unistd.h>
#include <csignal>
#include <chrono>
#include <vector>
#include <memory>
#include <map>
#include <cmath>
#include <future>

using cactus_rt::App;
using cactus_rt::CyclicThread;

// Global flag for indicating if Ctrl+C was pressed
volatile sig_atomic_t ctrl_c_pressed = 0;

// Signal handler function
void signalHandler(int signal)
{
    ctrl_c_pressed = 1; // Set flag to indicate Ctrl+C was pressed
}

double inputAndLimitTorque(const std::string &motor_name, double max_torque)
{
    double torque_command;
    std::cout << "Enter torque command for " << motor_name << ": ";
    std::cin >> torque_command;

    // Limit the torque command to the nearest value within the range [-max_torque, max_torque]
    if (torque_command < -max_torque)
    {
        std::cout << "TORQUE " << torque_command << "Nm TOO HIGH. SETTING TO MAX TORQUE: " << -max_torque << "Nm" << std::endl;
        torque_command = -max_torque;
    }
    else if (torque_command > max_torque)
    {
        std::cout << "TORQUE " << torque_command << "Nm TOO HIGH. SETTING TO MAX TORQUE: " << max_torque << "Nm" << std::endl;
        torque_command = max_torque;
    }
    return torque_command;
}

int main(int argc, char **argv)
{
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
    cmd.kp_scale = 0.0;
    cmd.kd_scale = 0.0;
    cmd.feedforward_torque = 0.0;

    const double MAX_TORQUE = 0.2;
    double torque_command[2] = {};

    // Manually input torque commands
    torque_command[0] = inputAndLimitTorque("motor 1", MAX_TORQUE);
    torque_command[1] = inputAndLimitTorque("motor 2", MAX_TORQUE);

    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;

    using namespace std::chrono;
    auto start = steady_clock::now();
    int desired_frequency = 50;
    const int max_frequency = 350;
    int sleep_time = 1000000 / desired_frequency;
    int loop_count = 0, missed_ticks = 0;
    std::map<int, int> missed_ticks_per_frequency;
    steady_clock::time_point last_frequency_increase = start;

    while (!ctrl_c_pressed)
    {
        auto loop_start = steady_clock::now();

        send_frames.clear();
        receive_frames.clear();

        for (size_t i = 0; i < controllers.size(); i++)
        {
            cmd.feedforward_torque = torque_command[i];
            send_frames.push_back(controllers[i]->MakePosition(cmd));
        }

        // Asynchronously send frames
        std::promise<void> done_promise;
        auto done_future = done_promise.get_future();
        bool promise_set = false; // Flag to track whether the promise has been set

        transport->Cycle(&send_frames[0], send_frames.size(), &receive_frames,
                         [&](int status) { // This matches the expected signature of CompletionCallback
                             if (!promise_set)
                             {
                                 done_promise.set_value();
                                 promise_set = true;
                             }
                         });

        // Wait for the asynchronous operation to complete or timeout
        if (done_future.wait_for(std::chrono::microseconds(sleep_time)) == std::future_status::timeout)
        {
            missed_ticks++;
            // std::cerr << "Error: Cycle did not complete in time!" << std::endl;
        }

        // Check if it's time to increase the frequency
        if (duration_cast<seconds>(loop_start - last_frequency_increase).count() >= 5)
        {
            if (desired_frequency < max_frequency)
            {
                missed_ticks_per_frequency[desired_frequency] = missed_ticks;
                missed_ticks = 0;

                desired_frequency += 10;
                if (desired_frequency >= max_frequency)
                {
                    break;
                }
                sleep_time = 1000000 / desired_frequency;
                last_frequency_increase = loop_start;
                std::cout << "Increased frequency to " << desired_frequency << " Hz\n";
            }
        }

        loop_count++;
    }

    missed_ticks_per_frequency[desired_frequency] = missed_ticks;

    auto end = steady_clock::now();
    auto total_time = duration_cast<seconds>(end - start).count();

    for (auto &c : controllers)
    {
        c->SetStop();
    }

    // Print missed tick percentages
    std::cout << "Missed tick percentages:\n";
    for (const auto &freq : missed_ticks_per_frequency)
    {
        double percentage = (static_cast<double>(freq.second) / loop_count) * 100;
        std::cout << "At " << freq.first << " Hz: " << percentage << "%\n";
    }

    std::cout << "Total runtime: " << total_time << " seconds\n";
    std::cout << "Total loops: " << loop_count << "\n";

    return 0;
}
