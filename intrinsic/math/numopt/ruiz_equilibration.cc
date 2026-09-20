// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "intrinsic/math/numopt/ruiz_equilibration.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

namespace {

absl::Status ValidateInput(Eigen::Ref<const eigenmath::VectorXd> c,
                           Eigen::Ref<const eigenmath::MatrixXd> A_ineq,
                           Eigen::Ref<const eigenmath::VectorXd> b_ineq,
                           Eigen::Ref<const eigenmath::MatrixXd> A_eq,
                           Eigen::Ref<const eigenmath::VectorXd> b_eq,
                           const int max_iterations) {
  if (A_ineq.rows() != b_ineq.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The rows of `A_ineq` should match the size of `b_ineq`. Got ",
        A_ineq.rows(), " != ", b_ineq.size(), "."));
  }
  if (A_ineq.rows() > 0 && A_ineq.cols() != c.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The columns of `A_ineq` should match the size of `c`. Got ",
        A_ineq.cols(), " != ", c.size(), "."));
  }
  if (A_eq.rows() != b_eq.size()) {
    return absl::InvalidArgumentError(
        absl::StrCat("The rows of `A_eq` should match the size of `b_eq`. Got ",
                     A_eq.rows(), " != ", b_eq.size(), "."));
  }
  if (A_eq.rows() > 0 && A_eq.cols() != c.size()) {
    return absl::InvalidArgumentError(
        absl::StrCat("The columns of `A_eq` should match the size of `c`. Got ",
                     A_eq.cols(), " != ", c.size(), "."));
  }
  if (max_iterations <= 0) {
    return absl::InvalidArgumentError(
        "The max_iterations parameter must be strictly positive.");
  }
  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<RuizEquilibrationResult> ApplyRuizEquilibration(
    Eigen::Ref<const eigenmath::VectorXd> c,
    Eigen::Ref<const eigenmath::MatrixXd> A_ineq,
    Eigen::Ref<const eigenmath::VectorXd> b_ineq,
    Eigen::Ref<const eigenmath::MatrixXd> A_eq,
    Eigen::Ref<const eigenmath::VectorXd> b_eq, const int max_iterations) {
  INTR_RETURN_IF_ERROR(
      ValidateInput(c, A_ineq, b_ineq, A_eq, b_eq, max_iterations));

  // Prepare space for scaled variables.
  const size_t num_vars = c.size();
  eigenmath::VectorXd c_scaled = c;
  eigenmath::MatrixXd A_ineq_scaled = A_ineq;
  eigenmath::VectorXd b_ineq_scaled = b_ineq;
  eigenmath::MatrixXd A_eq_scaled = A_eq;
  eigenmath::VectorXd b_eq_scaled = b_eq;
  eigenmath::VectorXd columns_scaling = eigenmath::VectorXd::Ones(num_vars);

  // Ruiz numerical tolerance for applying scaling.
  constexpr double kScaleTolerance = 1.0e-9;

  // Computes the row scaling.
  auto scale_rows = [](eigenmath::MatrixXd& matrix,
                       eigenmath::VectorXd& vector) {
    const int rows = matrix.rows();
    for (int i = 0; i < rows; ++i) {
      const double max_coeff = matrix.row(i).cwiseAbs().maxCoeff();
      if (max_coeff > kScaleTolerance) {
        const double factor = std::sqrt(max_coeff);
        const double inv_factor = 1.0 / factor;
        matrix.row(i) *= inv_factor;
        vector(i) *= inv_factor;
      }
    }
  };

  // Ruiz scaling iterations
  const bool has_equalities = A_eq_scaled.rows() > 0;
  const bool has_inequalities = A_ineq_scaled.rows() > 0;
  for (int iter = 0; iter < max_iterations; ++iter) {
    // Row scaling.
    if (has_inequalities) scale_rows(A_ineq_scaled, b_ineq_scaled);
    if (has_equalities) scale_rows(A_eq_scaled, b_eq_scaled);

    // Column scaling.
    for (int j = 0; j < num_vars; ++j) {
      double max_coeff = 0.0;
      if (has_inequalities) {
        max_coeff =
            std::max(max_coeff, A_ineq_scaled.col(j).cwiseAbs().maxCoeff());
      }
      if (has_equalities) {
        max_coeff =
            std::max(max_coeff, A_eq_scaled.col(j).cwiseAbs().maxCoeff());
      }
      if (max_coeff > kScaleTolerance) {
        const double factor = std::sqrt(max_coeff);
        const double inv_factor = 1.0 / factor;
        if (has_inequalities) A_ineq_scaled.col(j) *= inv_factor;
        if (has_equalities) A_eq_scaled.col(j) *= inv_factor;
        c_scaled(j) *= inv_factor;
        columns_scaling(j) *= inv_factor;
      }
    }
  }

  return RuizEquilibrationResult{.c = std::move(c_scaled),
                                 .A_ineq = std::move(A_ineq_scaled),
                                 .b_ineq = std::move(b_ineq_scaled),
                                 .A_eq = std::move(A_eq_scaled),
                                 .b_eq = std::move(b_eq_scaled),
                                 .columns_scaling = std::move(columns_scaling)};
}

}  // namespace intrinsic
