#include <cactus_rt/rt.h>
#include "moteus.h"

#include <vector>
#include <memory>
#include <iostream>
#include <chrono>
#include <signal.h>
#include <algorithm>
#include <map>
#include <string>
#include <vector>
#include <thread>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

// Global flag for indicating if Ctrl+C was pressed
volatile sig_atomic_t ctrl_c_pressed = 0;

typedef websocketpp::server<websocketpp::config::asio> server;

server ws_server;
std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> ws_connections;

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

// A simple way to get the current time accurately as a double.
static double GetNow()
{
    struct timespec ts = {};
    ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<double>(ts.tv_sec) +
           static_cast<double>(ts.tv_nsec) / 1e9;
}

void broadcastData(const std::string &data)
{
    for (auto hdl : ws_connections)
    {
        ws_server.send(hdl, data, websocketpp::frame::opcode::text);
    }
}

void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg)
{
    // Handle incoming messages if needed
}

void initWebSocket()
{
    ws_server.init_asio();

    ws_server.set_message_handler(&on_message);
    ws_server.set_open_handler([](websocketpp::connection_hdl hdl)
                               { ws_connections.insert(hdl); });
    ws_server.set_close_handler([](websocketpp::connection_hdl hdl)
                                { ws_connections.erase(hdl); });

    ws_server.listen(9002);
    ws_server.start_accept();

    std::thread([]()
                { ws_server.run(); })
        .detach();
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
        cmd_.kp_scale = 1.0;
        cmd_.kd_scale = 1.0;

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

        if (index_ >= torque_commands_.size())
        {
            std::cout << "\nAll Actions Complete. Press Ctrl+C to Exit\n";
            // return true;
        }

        for (size_t i = 0; i < controllers_.size(); i++)
        {
            cmd_.feedforward_torque = torque_commands_[index_][i];
            send_frames.push_back(controllers_[i]->MakePosition(cmd_));
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

        std::ostringstream oss;
        oss << "timestamp:" << index_;
        for (size_t i = 0; i < controllers_.size(); ++i)
        {
            // Query the current position of the motor
            auto maybe_result = controllers_[i]->SetQuery();
            if (maybe_result)
            {
                auto mode = maybe_result->values.mode;
                auto position = maybe_result->values.position;
                auto velocity = maybe_result->values.velocity;
                auto torque = maybe_result->values.torque;
                auto motor_temperature = maybe_result->values.motor_temperature;
                auto trajectory_complete = maybe_result->values.trajectory_complete;
                auto temperature = maybe_result->values.temperature;
                auto fault = maybe_result->values.fault;

                oss << ",motor" << i + 1 << "_mode:" << static_cast<int>(mode);
                oss << ",motor" << i + 1 << "_position:" << position;
                oss << ",motor" << i + 1 << "_velocity:" << velocity;
                oss << ",motor" << i + 1 << "_torque:" << torque;
                oss << ",motor" << i + 1 << "_motor_temperature:" << motor_temperature;
                oss << ",motor" << i + 1 << "_trajectory_complete:" << trajectory_complete;
                oss << ",motor" << i + 1 << "_temperature:" << temperature;
                oss << ",motor" << i + 1 << "_fault:" << static_cast<int>(fault);
            }
        }
        std::string data = oss.str();

        // Broadcast data via WebSocket
        broadcastData(data);

        ::usleep(10);

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
    std::string filename = "../trajGen/trajectory_data.csv";

    std::vector<std::vector<float>> data = readCSV(filename);

    initWebSocket();

    // Real-time thread configuration
    cactus_rt::CyclicThreadConfig config;
    config.period_ns = 150'000;  // Target Time in ns
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
    // std::cin.get();

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
