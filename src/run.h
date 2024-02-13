#pragma once

#include "single_joint_variables.h"
#include "parser.h"

#include <cmath>

namespace optimizer
{

/** Runs one evaluation */
double run(const double * value);

/** Returns the initial value based on the initial configuration */
std::array<double, variables.size()> get_x0();

/** Set the initial best score (useful to disable saving on single runs */
void set_best_score(double s);

/** Enable rendering */
void set_render();

/** Set the main robot used in the run function */
void set_main_robot(const std::string & robot, const std::string & joint);

/** Mute all mc_rtc outputs */
void mute_mc_rtc();

/** Restore mc_rtc outputs */
void unmute_mc_rtc();

double euclideanNorm(std::vector<double> v1, std::vector<double> v2);
} // namespace optimizer
