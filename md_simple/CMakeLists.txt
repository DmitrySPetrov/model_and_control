cmake_minimum_required( VERSION 3.0 )
project(spacecraft_motion_model CXX)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(UnitTest++ 2.0 REQUIRED)

include_directories(.)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${UTPP_INCLUDE_DIRS})

set(SRC motion.cpp)

add_executable(tests tests.cpp ${SRC})
target_link_libraries(tests Eigen3::Eigen UnitTest++)
