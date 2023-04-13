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
static inline const std::vector<std::string> mj_dof_names = {
  "RCY", "RCR", "RCP", "RKP", "RAP", "RAR",
  "LCY", "LCR", "LCP", "LKP", "LAP", "LAR",
};

class Variables
{
public:
  enum class DAMPING
  {
    MOT_00, MOT_01, MOT_02, MOT_03, MOT_04, MOT_05,
    MOT_06, MOT_07, MOT_08, MOT_09, MOT_10, MOT_11,
  };
  enum class FRICTIONLOSS
  {
    MOT_00, MOT_01, MOT_02, MOT_03, MOT_04, MOT_05,
    MOT_06, MOT_07, MOT_08, MOT_09, MOT_10, MOT_11,
  };
  enum class BEMF
  {
    MOT_00, MOT_01, MOT_02, MOT_03, MOT_04, MOT_05,
    MOT_06, MOT_07, MOT_08, MOT_09, MOT_10, MOT_11,
  };
  enum class MASS
  {
    BODY_LINK,
    RLEG_LINK0, RLEG_LINK1, RLEG_LINK2, RLEG_LINK3, RLEG_LINK4, RLEG_LINK5,
    LLEG_LINK0, LLEG_LINK1, LLEG_LINK2, LLEG_LINK3, LLEG_LINK4, LLEG_LINK5,
    CHEST_LINK2
  };
  enum class COM_POS
  {
    CHEST_LINK2_X, CHEST_LINK2_Y, CHEST_LINK2_Z,
  };
};

struct VariableBound
{
  double min;
  double max;
};

using Variant = std::variant<
  Variables::DAMPING, Variables::FRICTIONLOSS, Variables::BEMF, Variables::MASS, Variables::COM_POS>;
using VariableBounds = std::unordered_map<Variant, VariableBound>;

static inline VariableBounds sanity_bounds()
{
  VariableBounds map;
  for (int i = 0; i < static_cast<int>(Variables::DAMPING::MOT_11)+1; ++i)
  {
    Variables::DAMPING v = static_cast<Variables::DAMPING>(i);
    map[v] = {0, 10};
  }
  for (int i = 0; i < static_cast<int>(Variables::FRICTIONLOSS::MOT_11)+1; ++i)
  {
    Variables::FRICTIONLOSS v = static_cast<Variables::FRICTIONLOSS>(i);
    map[v] = {0, 10};
  }
  for (int i = 0; i < static_cast<int>(Variables::BEMF::MOT_11)+1; ++i)
  {
    Variables::BEMF v = static_cast<Variables::BEMF>(i);
    map[v] = {0, 0.5};
  }
  map[Variables::MASS::BODY_LINK] = {9, 10};
  map[Variables::MASS::RLEG_LINK0] = {2, 3};
  map[Variables::MASS::RLEG_LINK1] = {1, 2};
  map[Variables::MASS::RLEG_LINK2] = {6, 7};
  map[Variables::MASS::RLEG_LINK3] = {2.8, 4};
  map[Variables::MASS::RLEG_LINK4] = {0.8, 2};
  map[Variables::MASS::RLEG_LINK5] = {1, 2};
  map[Variables::MASS::LLEG_LINK0] = {2, 3};
  map[Variables::MASS::LLEG_LINK1] = {1, 2};
  map[Variables::MASS::LLEG_LINK2] = {6, 7};
  map[Variables::MASS::LLEG_LINK3] = {2.8, 4};
  map[Variables::MASS::LLEG_LINK4] = {0.8, 2};
  map[Variables::MASS::LLEG_LINK5] = {1, 2};
  map[Variables::MASS::CHEST_LINK2] = {20, 28};
  map[Variables::COM_POS::CHEST_LINK2_X] = {-0.2, -0.1};
  map[Variables::COM_POS::CHEST_LINK2_Y] = {-0.05, 0.05};
  map[Variables::COM_POS::CHEST_LINK2_Z] = {0.2, 0.3};
  return map;
}

/** Only the variables that appear in this array are considered for optimization */
static inline const std::array<Variant, 53> variables {
  Variables::DAMPING::MOT_00,
  Variables::DAMPING::MOT_01,
  Variables::DAMPING::MOT_02,
  Variables::DAMPING::MOT_03,
  Variables::DAMPING::MOT_04,
  Variables::DAMPING::MOT_05,
  Variables::DAMPING::MOT_06,
  Variables::DAMPING::MOT_07,
  Variables::DAMPING::MOT_08,
  Variables::DAMPING::MOT_09,
  Variables::DAMPING::MOT_10,
  Variables::DAMPING::MOT_11,
  Variables::FRICTIONLOSS::MOT_00,
  Variables::FRICTIONLOSS::MOT_01,
  Variables::FRICTIONLOSS::MOT_02,
  Variables::FRICTIONLOSS::MOT_03,
  Variables::FRICTIONLOSS::MOT_04,
  Variables::FRICTIONLOSS::MOT_05,
  Variables::FRICTIONLOSS::MOT_06,
  Variables::FRICTIONLOSS::MOT_07,
  Variables::FRICTIONLOSS::MOT_08,
  Variables::FRICTIONLOSS::MOT_09,
  Variables::FRICTIONLOSS::MOT_10,
  Variables::FRICTIONLOSS::MOT_11,
  Variables::BEMF::MOT_00,
  Variables::BEMF::MOT_01,
  Variables::BEMF::MOT_02,
  Variables::BEMF::MOT_03,
  Variables::BEMF::MOT_04,
  Variables::BEMF::MOT_05,
  Variables::BEMF::MOT_06,
  Variables::BEMF::MOT_07,
  Variables::BEMF::MOT_08,
  Variables::BEMF::MOT_09,
  Variables::BEMF::MOT_10,
  Variables::BEMF::MOT_11,
  Variables::MASS::BODY_LINK,
  Variables::MASS::RLEG_LINK0,
  Variables::MASS::RLEG_LINK1,
  Variables::MASS::RLEG_LINK2,
  Variables::MASS::RLEG_LINK3,
  Variables::MASS::RLEG_LINK4,
  Variables::MASS::RLEG_LINK5,
  Variables::MASS::LLEG_LINK0,
  Variables::MASS::LLEG_LINK1,
  Variables::MASS::LLEG_LINK2,
  Variables::MASS::LLEG_LINK3,
  Variables::MASS::LLEG_LINK4,
  Variables::MASS::LLEG_LINK5,
  Variables::MASS::CHEST_LINK2,
  Variables::COM_POS::CHEST_LINK2_X,
  Variables::COM_POS::CHEST_LINK2_Y,
  Variables::COM_POS::CHEST_LINK2_Z,
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
  for (unsigned int i = 31; i < 43; ++i)
  {
    value[i] = 0;
  }
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

  k = 0;
  for (unsigned int i = 31; i < 43; ++i)
  {
    std::string dof_name = robot_prefix + mj_dof_names[k];
    unsigned int dof_id = mj_name2id(&model, mjOBJ_JOINT, dof_name.c_str());
    model.dof_armature[dof_id] += value[i];
    k++;
  }
}
