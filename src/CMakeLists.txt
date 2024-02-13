set(run_sources
  run.cpp
  run.h
  variables.h
  single_joint_variables.h
  parser.cpp
  parser.h
)

add_library(run_function OBJECT ${run_sources})
target_link_libraries(run_function PUBLIC mc_mujoco::mc_mujoco)

find_package(OpenMP REQUIRED)

add_executable(${PROJECT_NAME}-cmaes main.cpp)
target_link_libraries(${PROJECT_NAME}-cmaes PUBLIC run_function Boost::program_options libcmaes::cmaes)
set_target_properties(${PROJECT_NAME}-cmaes PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
