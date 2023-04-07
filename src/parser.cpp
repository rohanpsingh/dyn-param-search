#include "parser.h"

namespace parser
{

std::vector<std::vector<double>> parseTrajectoryFile(const std::string & path, const int & rowsize)
{
  std::vector<std::vector<double>> data;
  std::ifstream strm(path.c_str());
  if(!strm.is_open())
  {
    std::cout << "Cannot open file at " << path << std::endl;
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
      std::cout << "Unexpected line size for trajectory data (Should be " << rowsize << ". Is " << row.size() << ")." << std::endl;
    }
    data.push_back(row);
  }
  return data;
}
}
