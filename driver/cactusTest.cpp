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

class MotorControlThread : public cactus_rt::CyclicThread
{
private:
    std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers_;
    std::shared_ptr<mjbots::moteus::Transport> transport_;
    mjbots::moteus::PositionMode::Command cmd_;
    std::vector<double> torque_commands_;
    std::vector<long long> loop_durations_;

public:
    MotorControlThread(const char *name, cactus_rt::CyclicThreadConfig config,
                       std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers,
                       std::vector<double> torque_commands,
                       std::shared_ptr<mjbots::moteus::Transport> transport)
        : CyclicThread(name, config), controllers_(controllers), torque_commands_(torque_commands), transport_(transport)
    {
        cmd_.kp_scale = 0.0;
        cmd_.kd_scale = 0.0;
        cmd_.feedforward_torque = 0.0;
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

        for (size_t i = 0; i < controllers_.size(); i++)
        {
            cmd_.feedforward_torque = torque_commands_[i];
            send_frames.push_back(controllers_[i]->MakePosition(cmd_));
        }

        // Perform the cycle with error handling
        transport_->BlockingCycle(&send_frames[0], send_frames.size(), &receive_frames);

        auto end_time = std::chrono::steady_clock::now(); // End timing
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
        loop_durations_.push_back(duration.count());

        return false;
    }
};

int main(int argc, char **argv)
{
    // Signal handling setup
    std::signal(SIGINT, signalHandler);

    // Real-time thread configuration
    cactus_rt::CyclicThreadConfig config;
    config.period_ns = 2'000'000; // Target Time in ns
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

    const double MAX_TORQUE = 0.2;
    std::vector<double> torque_commands = {
        inputAndLimitTorque("motor 1", MAX_TORQUE),
        inputAndLimitTorque("motor 2", MAX_TORQUE)};

    // Create the motor control thread
    auto motor_thread = std::make_shared<MotorControlThread>(
        "MotorControlThread", config, controllers, torque_commands, transport);

    cactus_rt::App app;
    app.RegisterThread(motor_thread);

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
