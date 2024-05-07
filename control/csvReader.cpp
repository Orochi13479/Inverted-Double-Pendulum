#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

int main() {
    // Specify the full path to the CSV file
    std::string filename = "Hand_Desgined_traj_gen_draft1.csv";
    std::string filepath = "/home/student/git/Inverted-Double-Pendulum/trajGen/" + filename;

    // Open the file
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filepath << std::endl;
        return 1;
    }

    // Skip the first line (column headings)
    std::string line;
    std::getline(file, line);

    // Arrays to store data for each column
    std::vector<float> timestamp;
    std::vector<float> q1;
    std::vector<float> q1_dot;
    std::vector<float> q2;
    std::vector<float> q2_dot;

    // Read and process the CSV data
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        float t, q1_val, q1_dot_val, q2_val, q2_dot_val;
        char comma;
        if (iss >> t >> comma >> q1_val >> comma >> q1_dot_val >> comma >> q2_val >> comma >> q2_dot_val) {
            // Add data to arrays
            timestamp.push_back(t);
            q1.push_back(q1_val);
            q1_dot.push_back(q1_dot_val);
            q2.push_back(q2_val);
            q2_dot.push_back(q2_dot_val);
        }
    }

    // Print the data (for testing)
    for (std::size_t i = 0; i < timestamp.size(); ++i) {
        std::cout << "timestamp: " << timestamp[i] << " ";
        std::cout << "q1: " << q1[i] << " ";
        std::cout << "q1_dot: " << q1_dot[i] << " ";
        std::cout << "q2: " << q2[i] << " ";
        std::cout << "q2_dot: " << q2_dot[i] << std::endl;
    }

    return 0;
}
