cmake_minimum_required(VERSION 3.13.0)
project(common CXX)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(PythonLibs REQUIRED)


add_executable(spile2d_test tcode/osqp_spline2d_solver_test.cc 
common/smoothing/affine_constraint.cc
common/smoothing/discrete_points_math.cc
common/smoothing/osqp_spline1d_solver.cc
common/smoothing/osqp_spline2d_solver.cc
common/smoothing/polynomialxd.cc
common/smoothing/spline1d_constraint.cc
common/smoothing/spline1d_kernel_helper.cc
common/smoothing/spline1d_kernel.cc
common/smoothing/spline1d_seg.cc
common/smoothing/spline1d.cc
common/smoothing/spline2d_constraint.cc
common/smoothing/spline2d_kernel.cc
common/smoothing/spline2d_seg
common/smoothing/spline2d.cc
common/utils/math.cc
common/solver/osqp/osqp_interface.cc
)

add_executable(spile1d_test tcode/osqp_spline1d_solver_test.cc 
common/smoothing/affine_constraint.cc
common/smoothing/discrete_points_math.cc
common/smoothing/osqp_spline1d_solver.cc
common/smoothing/osqp_spline2d_solver.cc
common/smoothing/polynomialxd.cc
common/smoothing/spline1d_constraint.cc
common/smoothing/spline1d_kernel_helper.cc
common/smoothing/spline1d_kernel.cc
common/smoothing/spline1d_seg.cc
common/smoothing/spline1d.cc
common/smoothing/spline2d_constraint.cc
common/smoothing/spline2d_kernel.cc
common/smoothing/spline2d_seg
common/smoothing/spline2d.cc
common/utils/math.cc
common/solver/osqp/osqp_interface.cc
)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${PYTHON_INCLUDE_DIRS})

target_link_libraries(spile2d_test 
  ${PYTHON_LIBRARIES}
  osqp::osqp
  glog
)
target_link_libraries(spile1d_test 
  ${PYTHON_LIBRARIES}
  osqp::osqp
  glog
)