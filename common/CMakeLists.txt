cmake_minimum_required(VERSION 3.13.0)
project(common CXX)

set(CMAKE_CXX_STANDARD 14)

find_package(glog REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/common)

add_subdirectory(utils)
add_subdirectory(spline)
add_subdirectory(solver)
add_subdirectory(frenet)
add_subdirectory(geometry)
add_subdirectory(smoothing)
add_subdirectory(thirdparty)

add_library(common INTERFACE)
target_link_libraries(common INTERFACE
  common::utils
  common::spline
  common::solver
  common::frenet
  common::geometry
)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(common_smoothing ${PYTHON_LIBRARIES})