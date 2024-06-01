#include <cactus_rt/rt.h>
#include <signal.h>

#include <algorithm>
#include <boost/optional.hpp>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "moteus.h"

// Global flag for indicating if Ctrl+C was pressed
volatile sig_atomic_t ctrl_c_pressed = 0;

// Signal handler function
void signalHandler(int signal)
{
    ctrl_c_pressed = 1; // Set flag to indicate Ctrl+C was pressed
}

// Arrays to store data for each column
std::vector<float> timestamp, q1, q1_dot, q1_dot_dot, tau1, q2, q2_dot, q2_dot_dot, tau2;

// Function to read CSV data
std::vector<std::vector<float>> readCSV(const std::string &filename)
{
    std::vector<std::vector<float>> data;
    std::string filepath = "../trajGen/" + filename;
    std::ifstream file(filepath);

    if (!file.is_open())
    {
        throw std::runtime_error("Error: Unable to open file " + filepath);
    }

    std::string line;
    std::getline(file, line); // Skip the first line (column headings)

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::vector<float> row(9);
        char comma;
        if (iss >> row[0] >> comma >> row[1] >> comma >> row[2] >> comma >> row[3] >> comma >> row[4] >> comma >> row[5] >> comma >> row[6] >> comma >> row[7] >> comma >> row[8])
        {
            data.push_back(row);
        }
    }

    return data;
}

double revolutionsToDegrees(double revolutions)
{
    const double degreesPerRevolution = 360.0;
    return revolutions * degreesPerRevolution;
}

double TorqueError(double desired_torque, double actual_torque)
{
    double torque_error = actual_torque - desired_torque;

    return torque_error;
}

// A simple way to get the current time accurately as a double.
static double GetNow()
{
    struct timespec ts = {};
    ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<double>(ts.tv_sec) +
           static_cast<double>(ts.tv_nsec) / 1e9;
}

// Class for controlling motors in a cyclic thread
class MotorControlThread : public cactus_rt::CyclicThread
{
private:
    std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers_;
    std::shared_ptr<mjbots::moteus::Transport> transport_;
    mjbots::moteus::PositionMode::Command cmd_;
    std::vector<std::vector<double>> torque_commands_;
    std::vector<int> time_intervals_;
    size_t index_;
    int interval_index_;

    // RT Additions
    std::map<int, bool> responses_;
    int total_count_;
    double total_hz_;

public:
    // Constructor
    MotorControlThread(const char *name, cactus_rt::CyclicThreadConfig config,
                       std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers,
                       std::vector<std::vector<double>> torque_commands,
                       std::vector<int> time_intervals,
                       std::shared_ptr<mjbots::moteus::Transport> transport)
        : CyclicThread(name, config), controllers_(controllers), torque_commands_(torque_commands), time_intervals_(time_intervals), transport_(transport), index_(0), interval_index_(0), total_count_(0), total_hz_(0)
    {
        // cmd_.maximum_torque = 1.0;
        cmd_.accel_limit = 200;
        cmd_.velocity_limit = 200;

        // Measuring Frequency
        int id = 0;
        for (const auto &controller : controllers_)
            responses_[id++] = false;
    }

protected:
    // Main loop function that executes cyclically
    bool Loop(int64_t /*now*/) noexcept final
    {
        std::vector<mjbots::moteus::CanFdFrame> send_frames;
        std::vector<mjbots::moteus::CanFdFrame> receive_frames;

        std::vector<double> cmd_kp = {10.0, 1};
        std::vector<double> cmd_kd = {5.0, 0.5};
        std::vector<double> cmd_pos = {0.0, 0.1};

        auto maybe_servo1 = controllers_[0]->SetQuery();
        auto maybe_servo2 = controllers_[1]->SetQuery();

        const auto &v1 = maybe_servo1->values;
        const auto &v2 = maybe_servo2->values;

        const std::vector<double> &last_torque_command = torque_commands_.back();
        std::vector<double> torque_diff = {TorqueError(last_torque_command[0], v1.torque), TorqueError(last_torque_command[1], v2.torque)};

        printf("MODE: %2d/%2d  POSITION: %6.3f/%6.3f  TORQUE: %6.3f/%6.3f  TORQUE ERROR: %6.3f/%6.3f  TEMP: %4.1f/%4.1f  VELOCITY: %6.3f/%6.3f FAULTS: %6.3f/%6.3f\r",
               static_cast<int>(v1.mode), static_cast<int>(v2.mode),
               v1.position, v2.position,
               v1.torque, v2.torque, torque_diff[0], torque_diff[1],
               v1.temperature, v2.temperature, v1.velocity, v2.velocity, v1.fault, v2.fault);
        fflush(stdout);

        for (size_t i = 0; i < controllers_.size(); i++)
        {
            if (index_ >= torque_commands_.size())
            {
                // cmd_.feedforward_torque = mjbots::moteus::kIgnore;
                // cmd_.velocity = 0.0;

                std::vector<double> torqueWithError = {v1.torque + torque_diff[0], v2.torque + torque_diff[1]};
                cmd_.feedforward_torque = torqueWithError[i];

                // return true;
            }
            else
            {
                std::cout << "TRAJ MODE" << std::endl;

                cmd_.feedforward_torque = torque_commands_[index_][i];
                // cmd_.velocity = 0.3;

                // cmd_.position = 0.1;
            }
            cmd_.kp_scale = cmd_kp[i];
            cmd_.kd_scale = cmd_kd[i];
            send_frames.push_back(controllers_[i]->MakePosition(cmd_));
            // controllers_[i]->SetPositionWaitComplete(cmd_, 1);
        }

        for (auto &pair : responses_)
            pair.second = false;

        transport_->BlockingCycle(&send_frames[0], send_frames.size(), &receive_frames);

        for (const auto &frame : receive_frames)
            responses_[frame.source] = true;

        const int count = std::count_if(responses_.begin(), responses_.end(),
                                        [](const auto &pair)
                                        { return pair.second; });

        constexpr double kStatusPeriodS = 0.1;
        static double status_time = GetNow() + kStatusPeriodS;
        static int hz_count = 0;

        hz_count++;

        const auto now = GetNow();
        if (now > status_time)
        {
            printf("             %6.1fHz  rx_count=%2d   \r",
                   hz_count / kStatusPeriodS, count);
            fflush(stdout);

            total_count_++;
            total_hz_ += (hz_count / kStatusPeriodS);

            hz_count = 0;
            status_time += kStatusPeriodS;
        }

        interval_index_++;
        if (interval_index_ >= time_intervals_[index_])
        {
            interval_index_ = 0;
            index_++;
        }

        ::usleep(1000);

        return false;
    }

public:
    // Function to calculate average frequency
    double GetAverageHz() const { return total_hz_ / total_count_; }
};

// Main function
int main(int argc, char **argv)
{
    // Signal handling setup
    std::signal(SIGINT, signalHandler);
    // Specify the full path to the CSV file
    std::string filename = "../trajGen/RTTestTraj.csv";

    std::vector<std::vector<float>> data = readCSV(filename);

    // Real-time thread configuration
    cactus_rt::CyclicThreadConfig config;
    config.period_ns = 2000'000; // Target Time in ns
    config.SetFifoScheduler(98); // Priority 0-100

    // Set up controllers and transport
    mjbots::moteus::Controller::DefaultArgProcess(argc, argv);
    auto transport = mjbots::moteus::Controller::MakeSingletonTransport({});

    // Options for setting up controllers
    mjbots::moteus::Controller::Options options_common;

    // Set position format
    auto &pf = options_common.position_format;
    pf.position = mjbots::moteus::kIgnore;
    pf.velocity = mjbots::moteus::kIgnore;
    pf.feedforward_torque = mjbots::moteus::kFloat;
    pf.kp_scale = mjbots::moteus::kInt8;
    pf.kd_scale = mjbots::moteus::kInt8;

    // Create two controllers
    std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers = {
        std::make_shared<mjbots::moteus::Controller>([&]()
                                                     {
                                                 auto options = options_common;
                                                 options.id = 1;
                                                 return options; }()),
        std::make_shared<mjbots::moteus::Controller>([&]()
                                                     {
                                                 auto options = options_common;
                                                 options.id = 2;
                                                 return options; }()),
    };

    for (auto &c : controllers)
    {
        c->SetStop();
    }

    // Extract torque commands, disregarding the first command
    std::vector<std::vector<double>> torque_commands;
    for (size_t i = 1; i < data.size(); ++i)
    {
        torque_commands.push_back({data[i][4], data[i][8]});
    }

    // Calculate time intervals as differences between timestamps
    std::vector<int> time_intervals;
    for (size_t i = 1; i < data.size(); ++i) // Start from the second element
    {
        time_intervals.push_back((data[i][0] * 1000) - (data[i - 1][0] * 1000));
    }
    std::cout << "time_intervalssize " << time_intervals.size() << std::endl;
    std::cout << "Torquesize " << torque_commands.size() << std::endl;

    // Printing torque commands
    std::cout << "Torque Commands:\n";
    for (size_t i = 0; i < torque_commands.size(); ++i)
    {
        std::cout << "Action " << i + 1 << ": Motor 1: " << torque_commands[i][0] << ", Motor 2: " << torque_commands[i][1] << std::endl;
    }

    // Print time intervals
    std::cout << "\nTime Intervals (in milliseconds):\n";
    for (size_t i = 0; i < time_intervals.size(); ++i)
    {
        std::cout << "Interval " << i + 1 << ": " << time_intervals[i] << " ms" << std::endl;
    }

    // Prompt the user to press enter
    std::cout << "CAREFULLY CHECK TORQUE VALUES";
    std::cout << "Press Enter to start the real-time loop...";
    std::cin.get();

    // Create the motor control thread
    auto motor_thread = std::make_shared<MotorControlThread>(
        "MotorControlThread", config, controllers, torque_commands, time_intervals, transport);

    // Create an application to manage the real-time thread
    cactus_rt::App app;
    app.RegisterThread(motor_thread);
    cactus_rt::SetUpTerminationSignalHandler();

    // Inform the user that the real-time loop is starting and will run until interrupted
    std::cout << "Testing RT loop until CTRL+C\n";

    // Start the application (and thus the motor control thread)
    app.Start();

    // Wait for a termination signal (e.g., Ctrl+C) and handle it appropriately
    cactus_rt::WaitForAndHandleTerminationSignal();

    // Request the application to stop and wait for it to join (clean exit)
    app.RequestStop();
    app.Join();

    // Ensure all controllers are set to stop, deactivating the mjbots
    for (auto &c : controllers)
    {
        c->SetStop();
    }

    // Calculate the average loop duration from the motor control thread
    std::cout << "Target Duration: " << config.period_ns << "ns" << std::endl;
    std::cout << "Target Frequency: " << 1 / (config.period_ns / 1e9) << "Hz" << std::endl;

    // Output the average speed of the motor control thread
    std::cout << "\nAverage speed: " << motor_thread->GetAverageHz() << " Hz\n";

    return 0;
}