#include <cactus_rt/rt.h>
#include "moteus.h"
#include <vector>
#include <memory>
#include <iostream>
#include <chrono>
#include <signal.h>

using namespace cactus_rt;
using namespace mjbots;

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

class MotorControlThread : public CyclicThread
{
private:
    std::vector<std::shared_ptr<moteus::Controller>> controllers_;
    std::shared_ptr<moteus::Transport> transport_;
    moteus::PositionMode::Command cmd_;
    std::vector<double> torque_commands_;

public:
    MotorControlThread(const char *name, CyclicThreadConfig config,
                       std::vector<std::shared_ptr<moteus::Controller>> controllers,
                       std::vector<double> torque_commands,
                       std::shared_ptr<moteus::Transport> transport)
        : CyclicThread(name, config), controllers_(controllers), torque_commands_(torque_commands), transport_(transport)
    {
        cmd_.kp_scale = 0.0;
        cmd_.kd_scale = 0.0;
        cmd_.feedforward_torque = 0.0;
        printf("constructor");
    }

protected:
    bool Loop(int64_t /*now*/) noexcept final
    {

        std::vector<moteus::CanFdFrame> send_frames;
        std::vector<moteus::CanFdFrame> receive_frames;

        for (size_t i = 0; i < controllers_.size(); i++)
        {
            cmd_.feedforward_torque = torque_commands_[i];
            send_frames.push_back(controllers_[i]->MakePosition(cmd_));
        }
        printf("blocking call");
        // Perform the cycle with error handling
        transport_->BlockingCycle(&send_frames[0], send_frames.size(), &receive_frames);

        return true;
    }
};

int main(int argc, char **argv)
{
    // Signal handling setup
    std::signal(SIGINT, signalHandler);

    // Real-time thread configuration
    CyclicThreadConfig config;
    config.period_ns = 3'000'000; // 1 ms period (1000 Hz)
    // config.cpu_affinity = std::vector<size_t>{4}; //Threads Changing causes issues
    config.SetFifoScheduler(90); // Priority

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

    const double MAX_TORQUE = 0.2;
    std::vector<double> torque_commands = {
        inputAndLimitTorque("motor 1", MAX_TORQUE),
        inputAndLimitTorque("motor 2", MAX_TORQUE)};

    // Create the motor control thread
    auto motor_thread = std::make_shared<MotorControlThread>(
        "MotorControlThread", config, controllers, torque_commands, transport);

    App app;
    app.RegisterThread(motor_thread);
    SetUpTerminationSignalHandler();

    std::cout << "Testing RT loop until CTRL+C\n";

    app.Start();
    while (!cactus_rt::HasTerminationSignalBeenReceived())
    {
        printf("whileloop");
        std::this_thread::sleep_for(std::chrono::nanoseconds(1));
    }
    printf("stopping thread");
    app.RequestStop();
    app.Join();

    for (auto &c : controllers)
    {
        c->SetStop();
    }

    return 0;
}
