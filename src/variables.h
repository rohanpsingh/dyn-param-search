#pragma once

#include <array>
#include <unordered_map>

enum class Variables
{
  DUMMY_PARAM_01,
  DUMMY_PARAM_02,
  DUMMY_PARAM_03,
  DUMMY_PARAM_04,
  DUMMY_PARAM_05,
  DUMMY_PARAM_06,
  DUMMY_PARAM_07,
  DUMMY_PARAM_08,
  DUMMY_PARAM_09,
  DUMMY_PARAM_10,
  DUMMY_PARAM_11,
  DUMMY_PARAM_12,
  DUMMY_PARAM_13,
  DUMMY_PARAM_14,
  DUMMY_PARAM_15,
  DUMMY_PARAM_16,
  DUMMY_PARAM_17,
  DUMMY_PARAM_18,
  DUMMY_PARAM_19,
  DUMMY_PARAM_20,
  DUMMY_PARAM_21,
  DUMMY_PARAM_22,
  DUMMY_PARAM_23,
  DUMMY_PARAM_24,
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
    {Variables::DUMMY_PARAM_01, {.0, 1}},
    {Variables::DUMMY_PARAM_02, {.0, 1}},
    {Variables::DUMMY_PARAM_03, {.0, 1}},
    {Variables::DUMMY_PARAM_04, {.0, 1}},
    {Variables::DUMMY_PARAM_05, {.0, 1}},
    {Variables::DUMMY_PARAM_06, {.0, 1}},
    {Variables::DUMMY_PARAM_07, {.0, 1}},
    {Variables::DUMMY_PARAM_08, {.0, 1}},
    {Variables::DUMMY_PARAM_09, {.0, 1}},
    {Variables::DUMMY_PARAM_10, {.0, 1}},
    {Variables::DUMMY_PARAM_11, {.0, 1}},
    {Variables::DUMMY_PARAM_12, {.0, 1}},
    {Variables::DUMMY_PARAM_13, {.0, 1}},
    {Variables::DUMMY_PARAM_14, {.0, 1}},
    {Variables::DUMMY_PARAM_15, {.0, 1}},
    {Variables::DUMMY_PARAM_16, {.0, 1}},
    {Variables::DUMMY_PARAM_17, {.0, 1}},
    {Variables::DUMMY_PARAM_18, {.0, 1}},
    {Variables::DUMMY_PARAM_19, {.0, 1}},
    {Variables::DUMMY_PARAM_20, {.0, 1}},
    {Variables::DUMMY_PARAM_21, {.0, 1}},
    {Variables::DUMMY_PARAM_22, {.0, 1}},
    {Variables::DUMMY_PARAM_23, {.0, 1}},
    {Variables::DUMMY_PARAM_24, {.0, 1}},
  };
}

/** Only the variables that appear in this array are considered for optimization */
static inline const std::array variables{
  Variables::DUMMY_PARAM_01,
  Variables::DUMMY_PARAM_02,
  Variables::DUMMY_PARAM_03,
  Variables::DUMMY_PARAM_04,
  Variables::DUMMY_PARAM_05,
  Variables::DUMMY_PARAM_06,
  Variables::DUMMY_PARAM_07,
  Variables::DUMMY_PARAM_08,
  Variables::DUMMY_PARAM_09,
  Variables::DUMMY_PARAM_10,
  Variables::DUMMY_PARAM_11,
  Variables::DUMMY_PARAM_12,
  Variables::DUMMY_PARAM_13,
  Variables::DUMMY_PARAM_14,
  Variables::DUMMY_PARAM_15,
  Variables::DUMMY_PARAM_16,
  Variables::DUMMY_PARAM_17,
  Variables::DUMMY_PARAM_18,
  Variables::DUMMY_PARAM_19,
  Variables::DUMMY_PARAM_20,
  Variables::DUMMY_PARAM_21,
  Variables::DUMMY_PARAM_22,
  Variables::DUMMY_PARAM_23,
  Variables::DUMMY_PARAM_24,
};
