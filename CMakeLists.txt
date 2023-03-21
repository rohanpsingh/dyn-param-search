cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
project(dyn-param-search LANGUAGES C CXX VERSION 1.0.0)

find_package(libcmaes REQUIRED)
find_package(mc_mujoco REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

install(CODE "message(\"Nothing to install yet\")")
