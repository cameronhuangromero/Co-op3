#include <iostream>
#include <fstream>
#include <vector>

int main() {
    std::vector<std::vector<int>> myArray = {
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9}
    };

    std::ofstream outputFile("my_array.csv");

    if (!outputFile.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    for (const auto& row : myArray) {
        for (size_t i = 0; i < row.size(); ++i) {
            outputFile << row[i];
            if (i < row.size() - 1) {
                outputFile << ",";
            }
        }
        outputFile << std::endl;
    }

    outputFile.close();
    std::cout << "CSV file created successfully!" << std::endl;
    return 0;
}
