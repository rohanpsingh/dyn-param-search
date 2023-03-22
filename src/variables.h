#pragma once

#include <array>
#include <unordered_map>

enum class Variables
{
  DUMMY_PARAM_1,
  DUMMY_PARAM_2,
};

struct VariableBound
{
  double min;
  double max;
};

using VariableBounds = std::unordered_map<Variables, VariableBound>;

static inline VariableBounds bounds_from_safety()
{
  return {
    {Variables::DUMMY_PARAM_1, {-1.0, 1.0}},
    {Variables::DUMMY_PARAM_2, {-1.0, 1.0}},
  };
}

/** Only the variables that appear in this array are considered for optimization */
static inline const std::array variables{Variables::DUMMY_PARAM_1, Variables::DUMMY_PARAM_2};
