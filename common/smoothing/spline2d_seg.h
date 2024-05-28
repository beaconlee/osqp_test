/* Copyright 2019 Unity-Drive Inc. All rights reserved */
/*     Author: ChengJie (chengjie@unity-drive.com)     */

#pragma once

#include <utility>
#include <vector>

#include "common/smoothing/polynomialxd.h"

namespace common
{

//////////////////////////
/// @brief  Spline2dSeg 存放的是当前路径段的 x,y 以及 x,y 的二阶三阶导
/// 
/// @date 2024-05-28
//////////////////////////
class Spline2dSeg
{
public:
  // order represent the number of parameters (not the highest order);
  // order 代表参数的数量（不是最高顺序）；
  explicit Spline2dSeg(const uint32_t order);
  explicit Spline2dSeg(const std::vector<double>& x_param,
                       const std::vector<double>& y_param);
  ~Spline2dSeg() = default;

  bool SetParams(const std::vector<double>& x_param,
                 const std::vector<double>& y_param);

  std::pair<double, double> operator()(const double t) const;

  double x(const double t) const;
  double y(const double t) const;
  double DerivativeX(const double t) const;
  double DerivativeY(const double t) const;
  double SecondDerivativeX(const double t) const;
  double SecondDerivativeY(const double t) const;
  double ThirdDerivativeX(const double t) const;
  double ThirdDerivativeY(const double t) const;

  const PolynomialXd& spline_func_x() const;
  const PolynomialXd& spline_func_y() const;
  const PolynomialXd& DerivativeX() const;
  const PolynomialXd& DerivativeY() const;
  const PolynomialXd& SecondDerivativeX() const;
  const PolynomialXd& SecondDerivativeY() const;
  const PolynomialXd& ThirdDerivativeX() const;
  const PolynomialXd& ThirdDerivativeY() const;

private:
  PolynomialXd spline_func_x_;
  PolynomialXd spline_func_y_;
  PolynomialXd derivative_x_;
  PolynomialXd derivative_y_;
  PolynomialXd second_derivative_x_;
  PolynomialXd second_derivative_y_;
  PolynomialXd third_derivative_x_;
  PolynomialXd third_derivative_y_;
};
} // namespace common
