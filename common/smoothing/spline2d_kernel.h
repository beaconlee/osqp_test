/* Copyright 2019 Unity-Drive Inc. All rights reserved */
/*     Author: ChengJie (chengjie@unity-drive.com)     */

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cinttypes>
#include <vector>

#include "common/smoothing/spline1d_kernel.h"


namespace common
{
class Spline2dKernel
{
public:
  Spline2dKernel(const std::vector<double>& t_knots,
                 const uint32_t spline_order);

  // 添加正则化
  void AddRegularization(const double regularized_param);
  // 添加二维导数核矩阵
  void Add2dDerivativeKernelMatrix(const double weight);
  // 添加2d二阶导数矩阵
  void Add2dSecondOrderDerivativeMatrix(const double weight);
  // 添加二维三阶导数矩阵
  void Add2dThirdOrderDerivativeMatrix(const double weight);

  // 添加二维三阶导数矩阵
  bool
  Add2dReferenceLineKernelMatrix(const std::vector<double>& t_coord,
                                 const vector_Eigen<Eigen::Vector2d>& ref_ft,
                                 const double weight);
  // 添加二维横向偏移核矩阵
  void
  Add2dLateralOffsetKernelMatrix(const std::vector<double>& t_coord,
                                 const vector_Eigen<Eigen::Vector2d>& ref_ft,
                                 const ::std::vector<double> ref_angle,
                                 const double weight);

  // 添加2d纵向偏移核矩阵
  void Add2dLongitudinalOffsetKernelMatrix(
      const std::vector<double>& t_coord,
      const vector_Eigen<Eigen::Vector2d>& ref_ft,
      const ::std::vector<double> ref_angle,
      const double weight);

  // 查找段起始索引
  size_t FindSegStartIndex(const double t) const;

  const Eigen::MatrixXd& kernel_matrix();
  const Eigen::MatrixXd& gradient();

private:
  /* kernel only contains x's component */
  Spline1dKernel x_kernel_;
  /* kernel only contains y's component */
  Spline1dKernel y_kernel_;
  /* kernel contains both x and y's component */
  Eigen::MatrixXd cooperative_kernel_;

  Eigen::MatrixXd kernel_matrix_;
  Eigen::MatrixXd gradient_;

  std::vector<double> t_knots_;

  uint32_t spline_order_ = 0;
  uint32_t spline_param_num_ = 0;
  uint32_t total_params_ = 0;
};
} // namespace common
