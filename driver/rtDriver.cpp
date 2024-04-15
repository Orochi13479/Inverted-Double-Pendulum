#include <iostream>
#include <unistd.h>
#include <csignal>
#include <chrono>
#include <vector>
#include <memory>
#include <map>
#include <cmath>

#include "moteus.h"

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
    int desired_frequency = 0.1;                  // Hz, initial desired control loop frequency
    const int max_frequency = 450;                // Hz, maximum control loop frequency
    int frequency_increase_interval = 5;          // seconds
    int sleep_time = 1000000 / desired_frequency; // microseconds
    int loop_count = 0;
    int missed_ticks = 0;

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

        // Send frames
        transport->BlockingCycle(&send_frames[0], send_frames.size(), &receive_frames);

        // Check if it's time to increase the frequency
        if (duration_cast<seconds>(loop_start - last_frequency_increase).count() >= frequency_increase_interval)
        {
            if (desired_frequency < max_frequency)
            {
                missed_ticks_per_frequency[desired_frequency] = missed_ticks;
                missed_ticks = 0; // Reset missed ticks for new frequency

                desired_frequency += 10; // Increase frequency by 50 Hz
                if (desired_frequency > max_frequency)
                {
                    desired_frequency = max_frequency;
                }
                sleep_time = 1000000 / desired_frequency; // Recalculate sleep time
                last_frequency_increase = loop_start;
                std::cout << "Increased frequency to " << desired_frequency << " Hz\n";
            }
        }

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

    missed_ticks_per_frequency[desired_frequency] = missed_ticks; // Record for the last frequency

    auto end = steady_clock::now();
    auto total_time = duration_cast<seconds>(end - start).count();

    ::usleep(50000);

    for (auto &c : controllers)
    {
        c->SetStop();
    }

    std::cout << "Total runtime: " << total_time << " seconds\n";
    std::cout << "Total loops: " << loop_count << "\n";
    for (const auto &freq : missed_ticks_per_frequency)
    {
        std::cout << "Missed ticks at " << freq.first << " Hz: " << freq.second << "\n";
    }

    return 0;
}
