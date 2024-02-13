#pragma once

#include <array>
#include <unordered_map>

#include <mc_mujoco/mj_sim.h>

class Variables
{
public:
  enum class DAMPING
  {
    MOT_00,
  };
  enum class FRICTIONLOSS
  {
    MOT_00,
  };
  enum class ARMATURE
  {
    MOT_00,
  };
  enum class EFFICIENCY
  {
    MOT_00,
  };
};

struct VariableBound
{
  double min;
  double max;
};

using Variant = std::variant<
  Variables::DAMPING, Variables::FRICTIONLOSS, Variables::ARMATURE, Variables::EFFICIENCY>;
using VariableBounds = std::unordered_map<Variant, VariableBound>;

static inline VariableBounds sanity_bounds()
{
  VariableBounds map;
  map[Variables::DAMPING::MOT_00] = {0, 30};
  map[Variables::FRICTIONLOSS::MOT_00] = {0, 30};
  map[Variables::ARMATURE::MOT_00] = {0, 15};
  map[Variables::EFFICIENCY::MOT_00] = {0, 1};
  return map;
}

/** Only the variables that appear in this array are considered for optimization */
static inline const std::array<Variant, 3> variables {
  Variables::DAMPING::MOT_00,
  Variables::FRICTIONLOSS::MOT_00,
  Variables::ARMATURE::MOT_00,
    //Variables::EFFICIENCY::MOT_00,
};

static inline void model_to_value(const std::string & jnt_name, const mjModel & model, double * value)
{
  unsigned int jnt_id = mj_name2id(&model, mjOBJ_JOINT, jnt_name.c_str());
  unsigned int dof_id = model.jnt_dofadr[jnt_id];
  value[0] = model.dof_damping[dof_id];
  value[1] = model.dof_frictionloss[dof_id];
  value[2] = model.dof_armature[dof_id];

  // std::string mot_name = jnt_name + "_motor";
  // unsigned int mot_id = mj_name2id(&model, mjOBJ_ACTUATOR, mot_name.c_str());
  // value[3] = model.actuator_user[mot_id];
}

static inline void value_to_model(const std::string & jnt_name, const double * value, mjModel & model)
{
  unsigned int jnt_id = mj_name2id(&model, mjOBJ_JOINT, jnt_name.c_str());
  unsigned int dof_id = model.jnt_dofadr[jnt_id];
  model.dof_damping[dof_id] = value[0];
  model.dof_frictionloss[dof_id] = value[1];
  model.dof_armature[dof_id] = value[2];

  // std::string mot_name = jnt_name + "_motor";
  // unsigned int mot_id = mj_name2id(&model, mjOBJ_ACTUATOR, mot_name.c_str());
  // model.actuator_user[mot_id] = value[3];
}
