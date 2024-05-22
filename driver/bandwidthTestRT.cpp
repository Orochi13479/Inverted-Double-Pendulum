#include <cactus_rt/rt.h>
#include "moteus.h"
#include <vector>
#include <memory>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <chrono>

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "moteus.h"

// A simple way to get the current time accurately as a double.
static double GetNow()
{
    struct timespec ts = {};
    ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<double>(ts.tv_sec) +
           static_cast<double>(ts.tv_nsec) / 1e9;
}

class RealTimeThread : public cactus_rt::CyclicThread
{
private:
    std::shared_ptr<mjbots::moteus::Transport> transport_;
    std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers_;
    std::map<int, bool> responses_;
    int total_count_;
    double total_hz_;

public:
    RealTimeThread(const char *name, cactus_rt::CyclicThreadConfig config,
                   std::shared_ptr<mjbots::moteus::Transport> transport,
                   std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers)
        : CyclicThread(name, config), transport_(transport), controllers_(controllers), total_count_(0), total_hz_(0)
    {
        int id = 0;
        for (const auto &controller : controllers_)
            responses_[id++] = false;
    }

protected:
    bool Loop(int64_t /*now*/) noexcept final
    {
        std::vector<mjbots::moteus::CanFdFrame> send_frames;
        std::vector<mjbots::moteus::CanFdFrame> receive_frames;
        mjbots::moteus::PositionMode::Command cmd;
        cmd.position = NaN;
        cmd.velocity = 0.0;

        // for (auto &controller : controllers_)
        //     send_frames.push_back(controller->MakePosition(cmd));
        for (size_t i = 0; i < controllers_.size(); i++)
        {
            send_frames.push_back(controllers_[i]->MakePosition(cmd));
        }

        for (auto &pair : responses_)
            pair.second = false;

        auto section_start_time = std::chrono::high_resolution_clock::now();

        transport_->BlockingCycle(&send_frames[0], send_frames.size(), &receive_frames);

        auto section_end_time = std::chrono::high_resolution_clock::now();
        auto section_duration = std::chrono::duration_cast<std::chrono::microseconds>(section_end_time - section_start_time).count();
        std::cout << "Section 2 (Transport Cycle) duration: " << section_duration << " microseconds\n";

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
            printf("                 %6.1fHz  rx_count=%2d   \r",
                   hz_count / kStatusPeriodS, count);
            fflush(stdout);

            total_count_++;
            total_hz_ += (hz_count / kStatusPeriodS);

            hz_count = 0;
            status_time += kStatusPeriodS;
        }

        ::usleep(10);

        return false;
    }

public:
    double GetAverageHz() const { return total_hz_ / total_count_; }
};

int main(int argc, char **argv)
{
    // Real-time thread configuration
    cactus_rt::CyclicThreadConfig config;
    config.period_ns = 100'000;  // Target Time in ns
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

    const std::vector<std::string> args_in(argv, argv + argc);
    auto args = mjbots::moteus::Controller::ProcessTransportArgs(args_in);

    // Just for some kind of "--help".
    mjbots::moteus::Controller::DefaultArgProcess(argc, argv);

    args.erase(args.begin()); // our name

    // Should use use int16 position/velocity command and query and
    // disable torque query?
    const bool minimal_format = [&]()
    {
        auto it = std::find(args.begin(), args.end(), "--minimal-format");
        if (it != args.end())
        {
            args.erase(it);
            return true;
        }
        return false;
    }();

    // Populate our list of controllers with IDs from the command line.
    std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers;
    for (const auto &arg : args)
    {
        auto id = std::stoul(arg);
        controllers.push_back(std::make_shared<mjbots::moteus::Controller>(
            [&]()
            {
                mjbots::moteus::Controller::Options options;
                options.id = id;
                if (minimal_format)
                {
                    options.position_format.position = mjbots::moteus::kInt16;
                    options.position_format.velocity = mjbots::moteus::kInt16;
                    options.query_format.position = mjbots::moteus::kInt16;
                    options.query_format.velocity = mjbots::moteus::kInt16;
                    options.query_format.torque = mjbots::moteus::kIgnore;
                }
                return options;
            }()));
    }

    // Create the real-time thread
    auto rt_thread = std::make_shared<RealTimeThread>(
        "RealTimeThread", config, transport, controllers);

    // Register and start the real-time thread
    cactus_rt::App app;
    app.RegisterThread(rt_thread);
    cactus_rt::SetUpTerminationSignalHandler();

    std::cout << "Testing RT loop until CTRL+C\n";

    app.Start();

    // Wait for Ctrl+C signal
    cactus_rt::WaitForAndHandleTerminationSignal();

    app.RequestStop();
    app.Join();

    // Output average speed
    std::cout << "\nAverage speed: " << rt_thread->GetAverageHz() << " Hz\n";

    return 0;
}
