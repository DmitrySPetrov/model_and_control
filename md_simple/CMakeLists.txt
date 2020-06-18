cmake_minimum_required( VERSION 3.0 )
project(spacecraft_motion_model CXX)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(.)
include_directories(${EIGEN3_INCLUDE_DIR})

set(SRC motion.cpp)

add_executable(tests tests.cpp ${SRC})
