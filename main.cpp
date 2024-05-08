#include <cactus_rt/rt.h>
#include "moteus.h"
#include <vector>
#include <memory>
#include <iostream>
#include <chrono>
#include <signal.h>

// Global flag for indicating if Ctrl+C was pressed
volatile sig_atomic_t ctrl_c_pressed = 0;

// Signal handler function
void signalHandler(int signal)
{
    ctrl_c_pressed = 1; // Set flag to indicate Ctrl+C was pressed
}

// Arrays to store data for each column
std::vector<int> timestamp;
std::vector<float> q1;
std::vector<float> q1_dot;
std::vector<float> q2;
std::vector<float> q2_dot;

void readCSV(const std::string &filename)
{
    std::string filepath = "../trajGen/" + filename;

    // Open the file
    std::ifstream file(filepath);

    if (!file.is_open())
    {
        throw std::runtime_error("Error: Unable to open file " + filepath);
    }

    // Skip the first line (column headings)
    std::string line;
    std::getline(file, line);

    // Read and process the CSV data
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        float t, q1_val, q1_dot_val, q2_val, q2_dot_val;
        char comma;
        if (iss >> t >> comma >> q1_val >> comma >> q1_dot_val >> comma >> q2_val >> comma >> q2_dot_val)
        {
            // Add data to arrays
            timestamp.push_back(t * 1000);
            q1.push_back(q1_val);
            q1_dot.push_back(q1_dot_val);
            q2.push_back(q2_val);
            q2_dot.push_back(q2_dot_val);
        }
    }
}

class MotorControlThread : public cactus_rt::CyclicThread
{
private:
    std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers_;
    std::shared_ptr<mjbots::moteus::Transport> transport_;
    mjbots::moteus::PositionMode::Command cmd_;
    std::vector<std::vector<double>> torque_commands_;
    std::vector<long long> loop_durations_;
    std::vector<int> time_intervals_;
    size_t index_;
    int interval_index_;

public:
    MotorControlThread(const char *name, cactus_rt::CyclicThreadConfig config,
                       std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers,
                       std::vector<std::vector<double>> torque_commands,
                       std::vector<int> time_intervals,
                       std::shared_ptr<mjbots::moteus::Transport> transport)
        : CyclicThread(name, config), controllers_(controllers), torque_commands_(torque_commands), time_intervals_(time_intervals), transport_(transport), index_(0), interval_index_(0)
    {
        cmd_.kp_scale = 0.0;
        cmd_.kd_scale = 0.0;
        cmd_.feedforward_torque = 0.0;
        cmd_.velocity_limit = 0.1; // Hertz revolutions / s
    }
    long long GetAverageLoopDuration() const
    {
        long long total_duration = 0;
        for (auto duration : loop_durations_)
        {
            total_duration += duration;
        }
        return loop_durations_.empty() ? 0 : total_duration / loop_durations_.size();
    }

protected:
    bool Loop(int64_t /*now*/) noexcept final
    {
        auto start_time = std::chrono::steady_clock::now(); // Start timing

        std::vector<mjbots::moteus::CanFdFrame> send_frames;
        std::vector<mjbots::moteus::CanFdFrame> receive_frames;

        if (index_ >= torque_commands_.size())
        {
            // Stop the thread if all torque commands have been sent
            return true;
        }

        for (size_t i = 0; i < controllers_.size(); i++)
        {
            cmd_.feedforward_torque = torque_commands_[index_][i];
            send_frames.push_back(controllers_[i]->MakePosition(cmd_));
        }

        // Perform the cycle with error handling
        transport_->BlockingCycle(&send_frames[0], send_frames.size(), &receive_frames);

        auto end_time = std::chrono::steady_clock::now(); // End timing
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
        loop_durations_.push_back(duration.count());

        interval_index_++;
        if (interval_index_ >= time_intervals_[index_])
        {
            interval_index_ = 0;
            index_++;
        }

        return false;
    }
};

int main(int argc, char **argv)
{
    // Signal handling setup
    std::signal(SIGINT, signalHandler);
    // Specify the full path to the CSV file
    std::string filename = "Hand_Desgined_traj_gen_draft1.csv";

    readCSV(filename);

    // Real-time thread configuration
    cactus_rt::CyclicThreadConfig config;
    config.period_ns = 2'500'000; // Target Time in ns
    config.SetFifoScheduler(98);  // Priority 0-100

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

    std::vector<std::vector<double>> torque_commands = {
        {0.05, -0.05},
        {-0.05, 0.05},
        {0.05, -0.05},
        {-0.05, 0.05},
        {0.05, -0.05},
        {-0.05, 0.05},
        {0.05, -0.05},
        {-0.05, 0.05},
        {0.05, -0.05},
    };

    // Calculate time intervals as differences between timestamps
    std::vector<int> time_intervals;
    for (size_t i = 1; i < timestamp.size(); ++i)
    {
        time_intervals.push_back(timestamp[i] - timestamp[i - 1]);
    }

    // Printing torque commands
    std::cout << "Torque Commands:\n";
    for (size_t i = 0; i < torque_commands.size(); ++i)
    {
        std::cout << "Motor 1: " << torque_commands[i][0] << ", Motor 2: " << torque_commands[i][1] << std::endl;
    }

    // Print time_intervals
    std::cout << "\nTime Intervals (in milliseconds):\n";
    for (size_t i = 0; i < time_intervals.size(); ++i)
    {
        std::cout << "Interval " << i + 1 << ": " << time_intervals[i] << " ms" << std::endl;
    }

    // Prompt the user to press enter
    std::cout << "Press Enter to start the real-time loop...";
    std::cin.get();

    // Create the motor control thread
    auto motor_thread = std::make_shared<MotorControlThread>(
        "MotorControlThread", config, controllers, torque_commands, time_intervals, transport);

    cactus_rt::App app;
    app.RegisterThread(motor_thread);
    cactus_rt::SetUpTerminationSignalHandler();

    std::cout << "Testing RT loop until CTRL+C\n";

    app.Start();

    cactus_rt::WaitForAndHandleTerminationSignal();

    app.RequestStop();
    app.Join();

    for (auto &c : controllers)
    {
        c->SetStop();
    }

    auto loopDuration = motor_thread->GetAverageLoopDuration();

    std::cout << "Target Duration: " << config.period_ns << "ns" << std::endl;
    std::cout << "Target Frequency: " << 1 / (config.period_ns / 1e9) << "Hz" << std::endl;

    std::cout << "Average Loop Duration: " << loopDuration << "ns" << std::endl;
    std::cout << "Average Loop Frequency: " << 1 / (loopDuration / 1e9) << "Hz" << std::endl;

    return 0;
}
