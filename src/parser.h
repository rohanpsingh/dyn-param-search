#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

namespace parser
{
void parseTrajectoryFile(const std::string & path, const int & rowsize, std::vector<std::vector<double>> & data);
}

