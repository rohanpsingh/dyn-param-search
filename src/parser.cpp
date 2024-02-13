#include "parser.h"

namespace parser
{

std::vector<std::vector<double>> parseTrajectoryFile(const std::string & path, const int & rowsize)
{
  std::vector<std::vector<double>> data;
  std::ifstream strm(path.c_str());
  if(!strm.is_open())
  {
    throw std::runtime_error("Cannot open file at " + path);
  }
  else
  {
    std::cout << "Reading file at " << path << std::endl;
  }

  std::string line;
  while (std::getline(strm, line))
  {
    if(line.empty())
    {
      continue;
    }
    if(line[0] == '#')
    {
      continue;
    }
    std::stringstream ss(line);
    std::vector<double> row;
    std::string value;
    while (std::getline(ss, value, ','))
    {
      row.push_back(std::stod(value));
    }
    if(row.size() != rowsize)
    {
      throw std::runtime_error("Unexpected line size (Should be "
			       + std::to_string(rowsize) + ". Is " + std::to_string(row.size()) + ").");
    }
    data.push_back(row);
  }
  return data;
}
}
