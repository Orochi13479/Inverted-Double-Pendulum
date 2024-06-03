#include <stdio.h>
#include <unistd.h>
#include <csignal> // For signal handling
#include <iostream>
#include <fstream> // For file I/O
#include <vector>  // For std::vector
#include <memory>  // For std::shared_ptr

#include "moteus.h"

// Global flag for indicating if Ctrl+C was pressed
volatile sig_atomic_t ctrl_c_pressed = 0;

// Signal handler function
void signalHandler(int signal)
{
    ctrl_c_pressed = 1; // Set flag to indicate Ctrl+C was pressed
}

int main(int argc, char **argv)
{
    // Set up signal handler for Ctrl+C (SIGINT)
    std::signal(SIGINT, signalHandler);

    using namespace mjbots;
    // Set up controllers and transport
    moteus::Controller::DefaultArgProcess(argc, argv);
    // auto transport = moteus::Controller::MakeSingletonTransport({});

    // Options for setting up controllers
    mjbots::moteus::Controller::Options options_common;

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

    // Open a CSV file for writing
    std::ofstream csv_file("../motor_data.csv");
    csv_file << "Time,q1,q1_dot,q1_dot_dot,tau1,q2,q2_dot,q2_dot_dot,tau2\n";

    // Get the start time
    auto start = std::chrono::steady_clock::now();

    std::cout << "Press Enter to start recording and CTRL+C to stop";
    std::cin.get();

    // Main loop
    while (!ctrl_c_pressed)
    {
        auto maybe_servo1 = controllers[0]->SetQuery();
        auto maybe_servo2 = controllers[1]->SetQuery();

        const auto &v1 = maybe_servo1->values;
        const auto &v2 = maybe_servo2->values;

        // Get the current time in milliseconds since start
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

        // Write data to the CSV file
        csv_file << elapsed << ","
                 << v1.position << "," << v1.velocity << "," << "0.0" << "," << v1.torque
                 << v2.position << "," << v2.velocity << "," << "0.0" << "," << v2.torque << "\n";
    }

    // Close the CSV file
    csv_file.close();

    std::cout << "File Written" << std::endl;

    ::usleep(50000);

    return 0;
}
