#pragma once

#include <array>
#include <unordered_map>

#include <mc_mujoco/mj_sim.h>

enum class Variables
{
  MOT_DAMPING,
  MOT_FRICTIONLOSS,
  MASS_BODY_LINK,
  MASS_RLEG_LINK0,
  MASS_RLEG_LINK1,
  MASS_RLEG_LINK2,
  MASS_RLEG_LINK3,
  MASS_RLEG_LINK4,
  MASS_RLEG_LINK5,
  MASS_LLEG_LINK0,
  MASS_LLEG_LINK1,
  MASS_LLEG_LINK2,
  MASS_LLEG_LINK3,
  MASS_LLEG_LINK4,
  MASS_LLEG_LINK5,
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
    {Variables::MOT_DAMPING, {0, 10}},
    {Variables::MOT_FRICTIONLOSS, {0, 10}},
    {Variables::MASS_BODY_LINK, {9, 10}},
    {Variables::MASS_RLEG_LINK0, {2, 3}},
    {Variables::MASS_RLEG_LINK1, {1, 2}},
    {Variables::MASS_RLEG_LINK2, {6, 7}},
    {Variables::MASS_RLEG_LINK3, {2.8, 4}},
    {Variables::MASS_RLEG_LINK4, {0.8, 2}},
    {Variables::MASS_RLEG_LINK5, {1, 2}},
    {Variables::MASS_LLEG_LINK0, {2, 3}},
    {Variables::MASS_LLEG_LINK1, {1, 2}},
    {Variables::MASS_LLEG_LINK2, {6, 7}},
    {Variables::MASS_LLEG_LINK3, {2.8, 4}},
    {Variables::MASS_LLEG_LINK4, {0.8, 2}},
    {Variables::MASS_LLEG_LINK5, {1, 2}},
  };
}

/** Only the variables that appear in this array are considered for optimization */
static inline const std::array variables{
  Variables::MOT_DAMPING,
  Variables::MOT_FRICTIONLOSS,
  Variables::MASS_BODY_LINK,
  Variables::MASS_RLEG_LINK0,
  Variables::MASS_RLEG_LINK1,
  Variables::MASS_RLEG_LINK2,
  Variables::MASS_RLEG_LINK3,
  Variables::MASS_RLEG_LINK4,
  Variables::MASS_RLEG_LINK5,
  Variables::MASS_LLEG_LINK0,
  Variables::MASS_LLEG_LINK1,
  Variables::MASS_LLEG_LINK2,
  Variables::MASS_LLEG_LINK3,
  Variables::MASS_LLEG_LINK4,
  Variables::MASS_LLEG_LINK5,
};

static inline void model_to_value(const mjModel & model, double * value)
{
  value[0] = 5;
  value[1] = 5 ;
  unsigned int body_id = 2;
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
