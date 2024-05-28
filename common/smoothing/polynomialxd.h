/* Copyright 2019 Unity-Drive Inc. All rights reserved */
/*     Author: ChengJie (chengjie@unity-drive.com)     */

#pragma once

#include <cinttypes>
#include <vector>

namespace common
{

/**
 * @class PolynomialXd  n 维多项式
 * @brief y = a0 + a1*x + a2*x2^2 + ... + an*xn^n
 */
class PolynomialXd
{
public:
  PolynomialXd() = default;
  explicit PolynomialXd(const std::uint32_t order);
  explicit PolynomialXd(const std::vector<double>& params);

  ~PolynomialXd() = default;

  double operator()(const double x) const;
  double operator[](const std::uint32_t index) const;

  // 设置多项式的次数
  void SetParams(const std::vector<double>& params);

  // 从另一个 PolynomicalXd 求导？
  static PolynomialXd DerivedFrom(const PolynomialXd& base);
  // 从另一个 PolynomicalXd 进行积分
  static PolynomialXd IntegratedFrom(const PolynomialXd& base,
                                     const double intercept = 0.0);

  // order 次   当前是几次多项式
  std::uint32_t order() const;

  // 获取多项式的次数
  const std::vector<double>& params() const;

private:
  std::vector<double> params_;
};
} // namespace common
