#pragma once

#include <array>
#include <unordered_map>

#include <mc_mujoco/mj_sim.h>

static inline const std::string robot_prefix = "hrp5_p_";
static inline const std::vector<std::string> mj_body_names = {
  "Body",
  "Rleg_Link0", "Rleg_Link1", "Rleg_Link2", "Rleg_Link3", "Rleg_Link4", "Rleg_Link5",
  "Lleg_Link0", "Lleg_Link1", "Lleg_Link2", "Lleg_Link3", "Lleg_Link4", "Lleg_Link5",
  "Chest_Link2",
};
static inline const std::vector<std::string> mj_motor_names = {
  "RCY_motor", "RCR_motor", "RCP_motor", "RKP_motor", "RAP_motor", "RAR_motor",
  "LCY_motor", "LCR_motor", "LCP_motor", "LKP_motor", "LAP_motor", "LAR_motor",
};

enum class Variables
{
  MOT_DAMPING,
  MOT_FRICTIONLOSS,
  MOT_BEMF_00,
  MOT_BEMF_01,
  MOT_BEMF_02,
  MOT_BEMF_03,
  MOT_BEMF_04,
  MOT_BEMF_05,
  MOT_BEMF_06,
  MOT_BEMF_07,
  MOT_BEMF_08,
  MOT_BEMF_09,
  MOT_BEMF_10,
  MOT_BEMF_11,
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
  MASS_CHEST_LINK2,
  POS_CHEST_LINK2_X,
  POS_CHEST_LINK2_Y,
  POS_CHEST_LINK2_Z,
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
    {Variables::MOT_BEMF_00, {0.0, 0.5}},
    {Variables::MOT_BEMF_01, {0.0, 0.5}},
    {Variables::MOT_BEMF_02, {0.0, 0.5}},
    {Variables::MOT_BEMF_03, {0.0, 0.5}},
    {Variables::MOT_BEMF_04, {0.0, 0.5}},
    {Variables::MOT_BEMF_05, {0.0, 0.5}},
    {Variables::MOT_BEMF_06, {0.0, 0.5}},
    {Variables::MOT_BEMF_07, {0.0, 0.5}},
    {Variables::MOT_BEMF_08, {0.0, 0.5}},
    {Variables::MOT_BEMF_09, {0.0, 0.5}},
    {Variables::MOT_BEMF_10, {0.0, 0.5}},
    {Variables::MOT_BEMF_11, {0.0, 0.5}},
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
    {Variables::MASS_CHEST_LINK2, {20, 28}},
    {Variables::POS_CHEST_LINK2_X, {-0.2, -0.1}},
    {Variables::POS_CHEST_LINK2_Y, {-0.05, 0.05}},
    {Variables::POS_CHEST_LINK2_Z, {0.2, 0.3}},
  };
}

/** Only the variables that appear in this array are considered for optimization */
static inline const std::array variables{
  Variables::MOT_DAMPING,
  Variables::MOT_FRICTIONLOSS,
  Variables::MOT_BEMF_00,
  Variables::MOT_BEMF_01,
  Variables::MOT_BEMF_02,
  Variables::MOT_BEMF_03,
  Variables::MOT_BEMF_04,
  Variables::MOT_BEMF_05,
  Variables::MOT_BEMF_06,
  Variables::MOT_BEMF_07,
  Variables::MOT_BEMF_08,
  Variables::MOT_BEMF_09,
  Variables::MOT_BEMF_10,
  Variables::MOT_BEMF_11,
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
  Variables::MASS_CHEST_LINK2,
  Variables::POS_CHEST_LINK2_X,
  Variables::POS_CHEST_LINK2_Y,
  Variables::POS_CHEST_LINK2_Z,
};

static inline void model_to_value(const mjModel & model, double * value)
{
  value[0] = 5;
  value[1] = 5;
  for (unsigned int i = 2; i < 14; ++i)
  {
    value[i] = 0;
  }
  unsigned int k = 0;
  for (unsigned int i = 14; i < 28; ++i)
  {
    std::string body_name = robot_prefix + mj_body_names[k];
    unsigned int body_id = mj_name2id(&model, mjOBJ_BODY, body_name.c_str());
    value[i] = model.body_mass[body_id];
    k++;
  }

  std::string chest_body_name = robot_prefix + "Chest_Link2";
  unsigned int chest_body_id = mj_name2id(&model, mjOBJ_BODY, chest_body_name.c_str());
  value[28] = model.body_ipos[3*chest_body_id + 0];
  value[29] = model.body_ipos[3*chest_body_id + 1];
  value[30] = model.body_ipos[3*chest_body_id + 2];
}

static inline void value_to_model(const double * value, mjModel & model)
{
  for (unsigned int d = 6; d < 18; ++d)
  {
    model.dof_damping[d] = value[0];
    model.dof_frictionloss[d] = value[1];
  }
  unsigned int k = 0;
  for (unsigned int i = 2; i < 14; ++i)
  {
    std::string mot_name = robot_prefix + mj_motor_names[k];
    unsigned int mot_id = mj_name2id(&model, mjOBJ_ACTUATOR, mot_name.c_str());
    model.actuator_user[mot_id] = value[i];
    k++;
  }
  k = 0;
  for (unsigned int i = 14; i < 28; ++i)
  {
    std::string body_name = robot_prefix + mj_body_names[k];
    unsigned int body_id = mj_name2id(&model, mjOBJ_BODY, body_name.c_str());
    model.body_mass[body_id] = value[i];
    k++;
  }

  std::string chest_body_name = robot_prefix + "Chest_Link2";
  unsigned int chest_body_id = mj_name2id(&model, mjOBJ_BODY, chest_body_name.c_str());
  model.body_ipos[3*chest_body_id + 0] = value[28];
  model.body_ipos[3*chest_body_id + 1] = value[29];
  model.body_ipos[3*chest_body_id + 2] = value[30];
}
