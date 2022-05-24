/* Copyright 2019 Unity-Drive Inc. All rights reserved */
/*     Author: Kin (kin_eng@163.com)     */

#include "common/smoothing/osqp_spline1d_solver.h"

#include "common/utils/math.h"
#include "common/smoothing/matplotlibcpp.h"

using namespace common;

int main(int argc, char const* argv[]) {
  // starting point
  std::vector<double> x_knots{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  OsqpSpline1dSolver pg(x_knots, 6);

  auto* spline_constraint = pg.mutable_constraint();
  auto* spline_kernel = pg.mutable_kernel();

  std::vector<double> x_coord{0,   0.4, 0.8, 1.2, 1.6, 2,   2.4,
                              2.8, 3.2, 3.6, 4,   4.4, 4.8, 5.2,
                              5.6, 6,   6.4, 6.8, 7.2, 7.6, 8};
  std::vector<double> fx_guide{
      0,       1.8,     3.6,     5.14901, 6.7408,  8.46267, 10.2627,
      12.0627, 13.8627, 15.6627, 17.4627, 19.2627, 21.0627, 22.8627,
      24.6627, 26.4627, 28.2627, 30.0627, 31.8627, 33.6627, 35.4627};
  std::vector<double> lower_bound(x_coord.size(), 0.0);
  std::vector<double> upper_bound(x_coord.size(), 68.4432);
  spline_constraint->AddBoundary(x_coord, lower_bound, upper_bound);
  std::vector<double> speed_lower_bound(x_coord.size(), 0.0);
  std::vector<double> speed_upper_bound(x_coord.size(), 4.5);
  spline_constraint->AddDerivativeBoundary(x_coord, speed_lower_bound,
                                           speed_upper_bound);
  // add jointness smooth constraint, up to jerk level continuous
  spline_constraint->AddThirdDerivativeSmoothConstraint();
  // spline_constraint->AddMonotoneInequalityConstraintAtKnots();
  spline_constraint->AddPointConstraint(0.0, 0.0);
  spline_constraint->AddPointDerivativeConstraint(0.0, 4.2194442749023438);
  spline_constraint->AddPointSecondDerivativeConstraint(0.0,
                                                        1.2431812867484089);
  spline_constraint->AddPointSecondDerivativeConstraint(8.0, 0.0);

  // add kernel (optimize kernel);
  // jerk cost

  spline_kernel->AddThirdOrderDerivativeMatrix(1000.0);
  spline_kernel->AddReferenceLineKernelMatrix(x_coord, fx_guide, 0.4);
  spline_kernel->AddRegularization(1.0);

    auto res = pg.Solve();
    auto res_spline = pg.spline();
  return 0;
}
