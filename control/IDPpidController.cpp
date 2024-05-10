#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <tuple>
#include <stdexcept>
#include "pid.h"

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
}



int main()
{
    // Specify the full path to the CSV file
    std::string filename = "trajectory_data.csv";

    readCSV(filename);


}
