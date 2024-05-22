#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdexcept>

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

int main()
{
    std::string filename = "RTTestTraj.csv";
    std::vector<std::vector<float>> data = readCSV(filename);

    // Print the data (for testing)
    for (const auto &row : data)
    {
        std::cout << "timestamp: " << row[0] << " "
                  << "pos1: " << row[1] << " "
                  << "velocity1: " << row[2] << " "
                  << "acc1: " << row[3] << " "
                  << "torque1: " << row[4] << " "
                  << "pos2: " << row[5] << " "
                  << "velocity2: " << row[6] << " "
                  << "acc2: " << row[7] << " "
                  << "torque2: " << row[8] << std::endl;
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

    return 0;
}
