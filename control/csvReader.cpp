#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <tuple>
#include <stdexcept>

// Arrays to store data for each column
std::vector<float> timestamp, q1, q1_dot, q1_dot_dot, tau1, q2, q2_dot, q2_dot_dot, tau2;

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
        float t, q1_val, q1_dot_val, q1_dot_dot_val, tau1_val, q2_val, q2_dot_val, q2_dot_dot_val, tau2_val;
        char comma;
        if (iss >> t >> comma >> q1_val >> comma >> q1_dot_val >> comma >> q2_val >> comma >> q2_dot_val)
        {
            // Add data to arrays
            timestamp.push_back(t);
            q1.push_back(q1_val);
            q1_dot.push_back(q1_dot_val);
            q1_dot_dot.push_back(q1_dot_dot_val);
            tau1.push_back(tau1_val);
            q2.push_back(q2_val);
            q2_dot.push_back(q2_dot_val);
            q2_dot_dot.push_back(q2_dot_dot_val);
            tau2.push_back(tau2_val);
        }
    }
    std::cout << "Using for loop:" << std::endl;
    for (size_t i = 0; i < tau1.size(); ++i) {
        std::cout << tau1[i] << " ";
    }
    std::cout << std::endl;
}

int main()
{
    // Specify the full path to the CSV file
    std::string filename = "trajectory_data.csv";

    readCSV(filename);

    // Print the data (for testing)
    for (std::size_t i = 0; i < timestamp.size(); ++i)
    {
        std::cout << "timestamp: " << timestamp[i] << " ";
        std::cout << "pos1: " << q1[i] << " ";
        std::cout << "velocity1: " << q1_dot[i] << " ";
        std::cout << "acc1: " << q1_dot_dot[i] << " ";
        std::cout << "torque1: " << tau1[i] << " ";
        std::cout << "pos2: " << q2[i] << " ";
        std::cout << "velocity2: " << q2_dot[i] << " ";
        std::cout << "acc2: " << q2_dot_dot[i] << " ";
        std::cout << "torque2: " << tau2[i] << " ";
        std::cout << std::endl;
    }

    return 0;
}
