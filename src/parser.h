#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

namespace parser
{
std::vector<std::vector<double>> parseTrajectoryFile(const std::string & path, const int & rowsize);
}

