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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_LINEAR_KALMAN_FILTER_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_LINEAR_KALMAN_FILTER_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/linear_systems/discrete_algebraic_riccati_equation.h"

namespace intrinsic::icon {

// Implements a stationary linear time-invariant Kalman filter.
class LinearKalmanFilter {
 public:
  // Creates a steady-state LinearKalmanFilter filter with process covariance
  // `Q`, measurement covariance `R`, linear system dynamics matrices `A` and
  // `B` and linear system output matrix `C`. The `dare_settings` allow to
  // configure solution type and properties of the underlying solver for the
  // Discrete Algebraic Riccati Equation.
  //
  // Returns an error in case of failed DARE convergence or dimension mismatch.
  static absl::StatusOr<LinearKalmanFilter> Create(
      const eigenmath::MatrixXd& Q, const eigenmath::MatrixXd& R,
      const eigenmath::MatrixXd& A, const eigenmath::MatrixXd& B,
      const eigenmath::MatrixXd& C,
      const DiscreteAlgebraicRiccatiSolverSettings& dare_settings =
          DiscreteAlgebraicRiccatiSolverSettings());

  // Creates a steady-state LinearKalmanFilter filter with process covariance
  // `Q`, measurement covariance `R`, linear system dynamics matrices `A` and
  // `B` and linear system output matrix `C`. Takes an initial guess for the
  // covariance matrix `P_initial`. The `dare_settings` allow to configure
  // solution type and properties of the underlying solver for the Discrete
  // Algebraic Riccati Equation.
  //
  // Returns an error in case of failed DARE convergence or dimension mismatch.
  static absl::StatusOr<LinearKalmanFilter> Create(
      const eigenmath::MatrixXd& Q, const eigenmath::MatrixXd& R,
      const eigenmath::MatrixXd& A, const eigenmath::MatrixXd& B,
      const eigenmath::MatrixXd& C, const eigenmath::MatrixXd& P_initial,
      const DiscreteAlgebraicRiccatiSolverSettings& dare_settings =
          DiscreteAlgebraicRiccatiSolverSettings());

  void SetInitialState(const eigenmath::VectorXd& x0) { x_est_ = x0; }

  const eigenmath::VectorXd& GetCurrentState() const { return x_est_; }

  const eigenmath::MatrixXd& GetCovariance() const { return dare_solution_.P; }

  const auto& dare_solution() const { return dare_solution_; }

  //! Linear estimator prediction step.
  const eigenmath::VectorXd& Predict(const eigenmath::VectorXd& u);

  //! Linear estimator correction step.
  const eigenmath::VectorXd& Correct(const eigenmath::VectorXd& y);

 protected:
  // Constructs a Linear Kalman Filter from linear system dynamics matrices A
  // and B, linear system output matrix C, covariance matrix P and a Kalman gain
  // K.
  LinearKalmanFilter(
      const eigenmath::MatrixXd& A, const eigenmath::MatrixXd& B,
      const eigenmath::MatrixXd& C,
      const DiscreteAlgebraicRiccatiEquationSolution& dare_solution);

  // Linear system state matrix.
  eigenmath::MatrixXd A_;  // NOLINT(google3-readability-class-member-naming)
  // Linear system input matrix.
  eigenmath::MatrixXd B_;  // NOLINT(google3-readability-class-member-naming)
  // Linear system observation matrix.
  eigenmath::MatrixXd C_;  // NOLINT(google3-readability-class-member-naming)
  DiscreteAlgebraicRiccatiEquationSolution dare_solution_;

  eigenmath::VectorXd x_est_;       // State estimate.
  eigenmath::VectorXd y_residual_;  // Observation residual.
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_LINEAR_KALMAN_FILTER_H_
