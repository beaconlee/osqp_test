/* Copyright 2019 Unity-Drive Inc. All rights reserved */
/*     Author: ChengJie (chengjie@unity-drive.com)     */

#include "common/smoothing/affine_constraint.h"

#include <glog/logging.h>

#include <iostream>
#include <utility>

namespace common {

AffineConstraint::AffineConstraint(const Eigen::MatrixXd& constraint_matrix,
                                   const std::vector<double>& lower_bound,
                                   const std::vector<double>& upper_bound)
    : constraint_matrix_(constraint_matrix),
      lower_bound_(lower_bound),
      upper_bound_(upper_bound) {
  CHECK_EQ(lower_bound_.size(), upper_bound_.size());
  CHECK_EQ(constraint_matrix_.rows(), lower_bound_.size());
}

const Eigen::MatrixXd& AffineConstraint::constraint_matrix() const {
  return constraint_matrix_;
}

const std::vector<double>& AffineConstraint::lower_bound() const {
  return lower_bound_;
}

const std::vector<double>& AffineConstraint::upper_bound() const {
  return upper_bound_;
}

bool AffineConstraint::AddConstraint(const Eigen::MatrixXd& constraint_matrix,
                                     const std::vector<double>& lower_bound,
                                     const std::vector<double>& upper_bound) {
  if (static_cast<uint32_t>(constraint_matrix.rows()) != lower_bound.size() ||
      static_cast<uint32_t>(constraint_matrix.rows()) != upper_bound.size()) {
    LOG(ERROR) << "Fail to add constraint because constraint matrix rows != "
                  "constraint boundary rows.";
    return false;
  }

  if (constraint_matrix_.rows() == 0) {
    constraint_matrix_ = constraint_matrix;
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
    return true;
  }

  if (constraint_matrix_.cols() != constraint_matrix.cols()) {
    LOG(ERROR)
        << "constraint_matrix_ cols and constraint_matrix cols do not match.";
    LOG(ERROR) << "constraint_matrix_.cols() = " << constraint_matrix_.cols();
    LOG(ERROR) << "constraint_matrix.cols() = " << constraint_matrix.cols();
    return false;
  }

  Eigen::MatrixXd expand_constraint_matrix(
      constraint_matrix_.rows() + constraint_matrix.rows(),
      constraint_matrix_.cols());

  /* matrix resize doesn't save memory allocation operation */
  expand_constraint_matrix << constraint_matrix_, constraint_matrix;
  constraint_matrix_ = std::move(expand_constraint_matrix);

  lower_bound_.insert(lower_bound_.end(), lower_bound.begin(),
                      lower_bound.end());
  upper_bound_.insert(upper_bound_.end(), upper_bound.begin(),
                      upper_bound.end());

  return true;
}

void AffineConstraint::Print() const {
  for (size_t i = 0; i < lower_bound_.size(); ++i) {
    printf("%d: lower_bound: %.2f,      upper_bound: %.2f\n", static_cast<int>(i), lower_bound_[i],
           upper_bound_[i]);
  }
}
}  // namespace common
