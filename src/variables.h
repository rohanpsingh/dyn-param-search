#pragma once

#include <array>
#include <unordered_map>

#include <mc_mujoco/mj_sim.h>

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
    {Variables::DUMMY_PARAM_01, {0, 10}},
    {Variables::DUMMY_PARAM_02, {0, 10}},
    {Variables::DUMMY_PARAM_03, {9, 10}},
    {Variables::DUMMY_PARAM_04, {2, 3}},
    {Variables::DUMMY_PARAM_05, {1, 2}},
    {Variables::DUMMY_PARAM_06, {6, 7}},
    {Variables::DUMMY_PARAM_07, {2.8, 4}},
    {Variables::DUMMY_PARAM_08, {0.8, 2}},
    {Variables::DUMMY_PARAM_09, {1, 2}},
    {Variables::DUMMY_PARAM_10, {2, 3}},
    {Variables::DUMMY_PARAM_11, {1, 2}},
    {Variables::DUMMY_PARAM_12, {6, 7}},
    {Variables::DUMMY_PARAM_13, {2.8, 4}},
    {Variables::DUMMY_PARAM_14, {0.8, 2}},
    {Variables::DUMMY_PARAM_15, {1, 2}},
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
  /* Variables::DUMMY_PARAM_16, */
  /* Variables::DUMMY_PARAM_17, */
  /* Variables::DUMMY_PARAM_18, */
  /* Variables::DUMMY_PARAM_19, */
  /* Variables::DUMMY_PARAM_20, */
  /* Variables::DUMMY_PARAM_21, */
  /* Variables::DUMMY_PARAM_22, */
  /* Variables::DUMMY_PARAM_23, */
  /* Variables::DUMMY_PARAM_24, */
};

static inline void model_to_value(const mjModel & model, double * value)
{
  value[0] = 5;
  value[1] = 5 ;
  unsigned int body_id = 2; // corresponds to the root body
  for (unsigned int i = 2; i < variables.size(); ++i)
  {
    value[i] = model.body_mass[body_id];
    body_id++;
  }
}

static inline void value_to_model(const double * value, mjModel & model)
{
  for (unsigned int d = 6; d < 18; ++d)
  {
    model.dof_damping[d] = value[0];
    model.dof_frictionloss[d] = value[1];
  }
  unsigned int body_id = 2;
  for (unsigned int i = 2; i < variables.size(); ++i)
  {
    model.body_mass[body_id]  = value[i];
    body_id++;
  }
}
