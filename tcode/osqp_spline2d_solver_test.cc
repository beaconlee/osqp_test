/* Copyright 2019 Unity-Drive Inc. All rights reserved */
/*     Author: ChengJie (chengjie@unity-drive.com)     */

#include "common/smoothing/osqp_spline2d_solver.h"

#include "common/smoothing/matplotlibcpp.h"
#include "common/utils/math.h"

using namespace common;

int main(int argc, char const* argv[])
{
  // 这里是多项式的参数
  Eigen::VectorXd x_coff(6), y_cofff(6);
  // a 是 x 坐标， b 是 y 坐标
  x_coff << 0, 1.5, 0.4, -0.03, -0.02, 0.01;
  y_cofff << 0.5, 1.5, 0.4, -0.03, 0.05, 0.01;
  std::cout << x_coff << std::endl;

  // 这里定义的是样条函数
  // 使用的是五次多项式来拟合实际的行驶路线
  auto spline = [](const Eigen::VectorXd& coff, const double t) -> double
  {
    return coff[0] + coff[1] * t + coff[2] * t * t + coff[3] * pow(t, 3) +
           coff[4] * pow(t, 4) + coff[5] * pow(t, 5);
  };

  // 函数的一阶导数
  auto spline_1st = [](const Eigen::VectorXd& coff, const double t) -> double
  {
    return coff[1] + 2 * coff[2] * t + 3 * coff[3] * pow(t, 2) +
           4 * coff[4] * pow(t, 3) + 5 * coff[5] * pow(t, 4);
  };

  // 函数的二阶导数
  auto spline_2st = [](const Eigen::VectorXd& coff, const double t) -> double
  {
    return 2 * coff[2] + 6 * coff[3] * t + 12 * coff[4] * pow(t, 2) +
           20 * coff[4] * pow(t, 3);
  };

  // 结点的范围，纵向长度是 0 到 5.0
  // knot（分段点，或称为锚点）：要用knot - 1个多项式来表征整条路径
  std::vector<double> t_knots{0, 10.0};
  // segment（细分小段，即每两个锚点之间还要多分几个小段）：主要用来画曲线 & 设置边界条件的。
  //

  // 创建一个 OSQP 解析器实例
  OsqpSpline2dSolver osqp_spline2d_solver(t_knots, 5);

  // 设置求解函数
  auto mutable_kernel = osqp_spline2d_solver.mutable_kernel();
  // 设置约束
  auto mutable_constraint = osqp_spline2d_solver.mutable_constraint();

  // 添加二维三阶导数矩阵 Add2dThirdOrderDerivativeMatrix
  // 对三阶导数设置权重 0.5  也就是加加速度
  mutable_kernel->Add2dThirdOrderDerivativeMatrix(0.5);

  // 时间坐标
  std::vector<double> t_coord;
  // vector_Eigen = std::vector<T, Eigen::aligned_allocator<T>>;
  // ref_ft 就是 t 时刻对应的 (x, y) 坐标
  vector_Eigen<Eigen::Vector2d> ref_ft;
  // 分别保存 t 时刻， x 和 y 的坐标
  std::vector<double> ref_x, ref_y;
  std::vector<double> lat_tol, lon_tol;
  std::vector<double> ref_theta;

  for(int i = 0; i < 10.0 / 0.25; ++i)
  {
    const double param = i * 0.25;
    //
    t_coord.emplace_back(i * 0.25);
    double x1, y1;
    if(i == 0 || i == 19)
    {
      x1 = spline(x_coff, param);
      y1 = spline(y_cofff, param);
    }
    else
    {
      x1 = spline(x_coff, param);  // + NormalDistribution(0, 0.1)
      y1 = spline(y_cofff, param); // + NormalDistribution(0, 0.1)
    }
    double theta =
        std::atan2(spline_1st(y_cofff, param), spline_1st(x_coff, param));
    std::cout << theta / 3.14 * 180 << std::endl;
    ref_ft.emplace_back(Eigen::Vector2d(x1, y1));
    ref_x.emplace_back(x1);
    ref_y.emplace_back(y1);
    ref_theta.emplace_back(theta);
    lat_tol.emplace_back(0.20);
    lon_tol.emplace_back(0.20);
  }

  mutable_kernel->Add2dReferenceLineKernelMatrix(t_coord, ref_ft, 0.5);



  // 添加 三阶导数平滑约束
  // 添加连续平滑约束 三阶
  mutable_constraint->Add2dThirdDerivativeSmoothConstraint();

  // init point constraint
  // Point Constraint 是添加点约束
  mutable_constraint->Add2dPointConstraint(
      0,
      Eigen::Vector2d(spline(x_coff, 0), spline(y_cofff, 0)));
  // 添加导数约束
  mutable_constraint->Add2dPointDerivativeConstraint(
      0,
      Eigen::Vector2d(spline_1st(x_coff, 0), spline_1st(y_cofff, 0)));

  // end point constraint
  double t_end = t_coord.back();
  mutable_constraint->Add2dPointConstraint(
      t_end,
      Eigen::Vector2d(spline(x_coff, t_end), spline(y_cofff, t_end)));
  mutable_constraint->Add2dPointDerivativeConstraint(
      t_end,
      Eigen::Vector2d(spline_1st(x_coff, t_end), spline_1st(y_cofff, t_end)));

  // 添加横向位置约束
  mutable_constraint->Add2dStationLateralBoundary(t_coord,
                                                  ref_ft,
                                                  ref_theta,
                                                  lon_tol,
                                                  lat_tol);

  // 进行求解
  auto res = osqp_spline2d_solver.Solve();
  // 获取解的结果
  auto res_spline = osqp_spline2d_solver.spline();

  std::vector<double> res_x, res_y;
  std::vector<double> ref_kappa, res_kappa;

  for(auto const t : t_coord)
  {
    res_x.emplace_back(res_spline.x(t));
    res_y.emplace_back(res_spline.y(t));

    ref_kappa.emplace_back(ComputeCurvature(spline_1st(x_coff, t),
                                            spline_2st(x_coff, t),
                                            spline_1st(y_cofff, t),
                                            spline_2st(y_cofff, t)));
    res_kappa.emplace_back(ComputeCurvature(res_spline.DerivativeX(t),
                                            res_spline.SecondDerivativeX(t),
                                            res_spline.DerivativeY(t),
                                            res_spline.SecondDerivativeY(t)));
  }

  namespace plt = matplotlibcpp;
  plt::named_plot("first_ref", ref_x, ref_y, "b-o");
  plt::named_plot("first_res", res_x, res_y, "r-o");
  plt::legend();
  plt::axis("equal");
  plt::figure();
  plt::named_plot("second_ref", t_coord, ref_kappa, "b-o");
  plt::named_plot("second_res", t_coord, res_kappa, "r-o");
  plt::legend();
  plt::show();

  return 0;
}
