/* Copyright 2019 Unity-Drive Inc. All rights reserved */
/*     Author: ChengJie (chengjie@unity-drive.com)     */

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <utility>
#include <vector>

#include "common/smoothing/spline2d_seg.h"

namespace common
{

// 2d样条线
class Spline2d
{
public:
  Spline2d() = default;
  Spline2d(const std::vector<double>& t_knots, const uint32_t order);

  // 获取 t 时刻的位置
  Eigen::Vector2d pos(const double t) const;
  std::pair<double, double> operator()(const double t) const;

  double theta(const double t) const;
  double x(const double t) const;
  double y(const double t) const;

  double DerivativeX(const double t) const;
  double DerivativeY(const double t) const;
  double Derivative(const double t) const;

  double SecondDerivativeX(const double t) const;
  double SecondDerivativeY(const double t) const;

  double ThirdDerivativeX(const double t) const;
  double ThirdDerivativeY(const double t) const;

  bool set_splines(const Eigen::MatrixXd& params, const uint32_t order);
  const Spline2dSeg& smoothing_spline(const uint32_t index) const;
  const std::vector<double>& t_knots() const;
  uint32_t spline_order() const;

  void GetCurvature(const double t, double* kappa, double* dkappa) const;
  double GetCurvature(const double t) const;

private:
  uint32_t find_index(const double x) const;

private:
  std::vector<Spline2dSeg> splines_;
  std::vector<double> t_knots_;
  uint32_t spline_order_;
};

class Spline2dPoint
{
public:
  Spline2dPoint() = default;
  Spline2dPoint(const Spline2d& spline2d, const double t)
    : fx_(spline2d.x(t))
    , fy_(spline2d.y(t))
    , dfx_(spline2d.DerivativeX(t))
    , dfy_(spline2d.DerivativeY(t))
    , ddfx_(spline2d.SecondDerivativeX(t))
    , ddfy_(spline2d.SecondDerivativeY(t))
    , dddfx_(spline2d.ThirdDerivativeX(t))
    , dddfy_(spline2d.ThirdDerivativeY(t))
  {}

  double fx() const
  {
    return fx_;
  }
  double fy() const
  {
    return fy_;
  }
  double dfx() const
  {
    return dfx_;
  }
  double dfy() const
  {
    return dfy_;
  }
  double ddfx() const
  {
    return ddfx_;
  }
  double ddfy() const
  {
    return ddfy_;
  }
  double dddfx() const
  {
    return dddfx_;
  }
  double dddfy() const
  {
    return dddfy_;
  }

private:
  double fx_ = 0.0;
  double fy_ = 0.0;
  double dfx_ = 0.0;
  double dfy_ = 0.0;
  double ddfx_ = 0.0;
  double ddfy_ = 0.0;
  double dddfx_ = 0.0;
  double dddfy_ = 0.0;
};
} // namespace common
