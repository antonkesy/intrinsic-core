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

#include "intrinsic/icon/control/algorithms/linear_kalman_filter.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/linear_systems/discrete_algebraic_riccati_equation.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

namespace {

absl::Status CheckSystemDimensions(const eigenmath::MatrixXd& A,
                                   const eigenmath::MatrixXd& B,
                                   const eigenmath::MatrixXd& C,
                                   const eigenmath::MatrixXd& P) {
  if (A.cols() < 1 || A.cols() != A.rows()) {
    return absl::FailedPreconditionError("State matrix size inconsistent.");
  }

  if (A.cols() < 1 || A.cols() != B.rows()) {
    return absl::FailedPreconditionError("Input matrix size inconsistent.");
  }

  if (C.rows() < 1 || A.cols() != C.cols()) {
    return absl::FailedPreconditionError("Output matrix size inconsistent.");
  }

  if (P.rows() != A.rows() || P.cols() != A.cols()) {
    return absl::FailedPreconditionError(
        "Covariance matrix size inconsistent.");
  }

  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<LinearKalmanFilter> LinearKalmanFilter::Create(
    const eigenmath::MatrixXd& Q, const eigenmath::MatrixXd& R,
    const eigenmath::MatrixXd& A, const eigenmath::MatrixXd& B,
    const eigenmath::MatrixXd& C, const eigenmath::MatrixXd& P_initial,
    const DiscreteAlgebraicRiccatiSolverSettings& dare_settings) {
  INTR_RETURN_IF_ERROR(CheckSystemDimensions(A, B, C, P_initial));
  // Compute the optimal Kalman gain for a stationary linear time-invariant
  // system by solving the Discrete Algebraic Riccati Equation.
  INTR_ASSIGN_OR_RETURN(
      auto dare_solution,
      SolveDiscreteAlgebraicRiccatiEquationIteratively(
          P_initial, Q, R, A.transpose(), C.transpose(), dare_settings));

  return LinearKalmanFilter(A, B, C, dare_solution);
}

absl::StatusOr<LinearKalmanFilter> LinearKalmanFilter::Create(
    const eigenmath::MatrixXd& Q, const eigenmath::MatrixXd& R,
    const eigenmath::MatrixXd& A, const eigenmath::MatrixXd& B,
    const eigenmath::MatrixXd& C,
    const DiscreteAlgebraicRiccatiSolverSettings& dare_settings) {
  eigenmath::MatrixXd P_initial =
      eigenmath::MatrixXd::Identity(A.cols(), A.cols());
  return Create(Q, R, A, B, C, P_initial, dare_settings);
}

LinearKalmanFilter::LinearKalmanFilter(
    const eigenmath::MatrixXd& A, const eigenmath::MatrixXd& B,
    const eigenmath::MatrixXd& C,
    const DiscreteAlgebraicRiccatiEquationSolution& dare_solution)
    : A_(A),
      B_(B),
      C_(C),
      dare_solution_(dare_solution),
      x_est_(eigenmath::VectorXd::Zero(A.rows())),
      y_residual_(eigenmath::VectorXd::Zero(C.rows())) {}

const eigenmath::VectorXd& LinearKalmanFilter::Predict(
    const eigenmath::VectorXd& u) {
  // x_est = A_ * x_est + B * u;
  x_est_.lazyAssign(A_.lazyProduct(x_est_) + B_ * u);
  return x_est_;
}

const eigenmath::VectorXd& LinearKalmanFilter::Correct(
    const eigenmath::VectorXd& y) {
  // x_est_ -= K_.transpose() * (y - C_ * x_est_);
  y_residual_.lazyAssign(y.eval() - C_.lazyProduct(x_est_));
  x_est_.lazyAssign(x_est_ - dare_solution_.K.transpose() * y_residual_);

  return x_est_;
}

}  // namespace intrinsic::icon
