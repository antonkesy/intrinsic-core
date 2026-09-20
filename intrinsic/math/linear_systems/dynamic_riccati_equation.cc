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

#include "intrinsic/math/linear_systems/dynamic_riccati_equation.h"

#include <cstddef>

#include "Eigen/Eigenvalues"
#include "absl/status/status.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

namespace {

absl::Status DimensionsAreConsistent(const eigenmath::MatrixXd& P,
                                     const eigenmath::MatrixXd& Q,
                                     const eigenmath::MatrixXd& R,
                                     const eigenmath::MatrixXd& A,
                                     const eigenmath::MatrixXd& B) {
  const int state_dim = A.rows();
  const int control_dim = R.rows();

  if (state_dim < 1) {
    return absl::FailedPreconditionError("State dimension < 1");
  }

  if (control_dim < 1) {
    return absl::FailedPreconditionError("Control dimension < 1");
  }

  if (A.cols() != A.rows()) {
    return absl::FailedPreconditionError("State matrix must be square.");
  }

  if (P.cols() != P.rows()) {
    return absl::FailedPreconditionError("P matrix must be square.");
  }

  if (P.cols() != state_dim) {
    return absl::FailedPreconditionError(
        "P matrix must match state dimensions.");
  }

  if (Q.cols() != Q.rows()) {
    return absl::FailedPreconditionError("Q matrix must be square.");
  }

  if (Q.cols() != state_dim) {
    return absl::FailedPreconditionError("Q matrix match state dimensions.");
  }

  if (R.cols() != R.rows()) {
    return absl::FailedPreconditionError("R matrix must be square.");
  }

  if (R.cols() != control_dim) {
    return absl::FailedPreconditionError(
        "R matrix must match input dimensions.");
  }

  if (B.cols() != R.rows()) {
    return absl::FailedPreconditionError(
        "Input system and covariance dimensions do not match.");
  }

  if (B.rows() != state_dim) {
    return absl::FailedPreconditionError("Input matrix must match state size.");
  }

  return absl::OkStatus();
}

}  // namespace

absl::Status ComputeNaiveDynamicRiccatiEquationIterate(
    const eigenmath::MatrixXd& P, const eigenmath::MatrixXd& Q,
    const eigenmath::MatrixXd& R, const eigenmath::MatrixXd& A,
    const eigenmath::MatrixXd& B, eigenmath::MatrixXd& P_next,
    eigenmath::MatrixXd& K) {
  INTR_RETURN_IF_ERROR(DimensionsAreConsistent(P, Q, R, A, B));

  const eigenmath::MatrixXd H = R + B.transpose() * P * B;

  K.noalias() = -H.inverse() * B.transpose() * P * A;

  P_next = Q + A.transpose() * P * A;  // .noalias() is omitted here because P
                                       // and P_next could be the same matrix.
  P_next.noalias() -= K.transpose() * H * K;

  // Ensure symmetry of P_next.
  P_next = (P_next + P_next.transpose()).eval() / 2.0;

  return absl::OkStatus();
}

absl::Status ComputeRobustDynamicRiccatiEquationIterate(
    const eigenmath::MatrixXd& P, const eigenmath::MatrixXd& Q,
    const eigenmath::MatrixXd& R, const eigenmath::MatrixXd& A,
    const eigenmath::MatrixXd& B, double epsilon, eigenmath::MatrixXd& P_next,
    eigenmath::MatrixXd& K) {
  INTR_RETURN_IF_ERROR(DimensionsAreConsistent(P, Q, R, A, B));

  Eigen::SelfAdjointEigenSolver<eigenmath::MatrixXd> eigenvalue_solver;
  const size_t control_dim = R.cols();

  eigenmath::MatrixXd H = R + B.transpose() * P * B;

  H = (H + H.transpose()).eval() / 2.0;

  // Compute Hessian Eigen-decomposition.
  eigenvalue_solver.compute(H);
  const eigenmath::MatrixXd& V = eigenvalue_solver.eigenvectors().real();
  const eigenmath::MatrixXd& lambda = eigenvalue_solver.eigenvalues().real();

  // Compute regularized eigenvalue matrix D, make D positive definite.
  eigenmath::MatrixXd D = eigenmath::MatrixXd::Zero(control_dim, control_dim);
  D.diagonal() = lambda.cwiseMax(eigenmath::VectorXd::Zero(control_dim)) +
                 epsilon * eigenmath::VectorXd::Ones(control_dim);

  // Reconstruct regularized Hessian.
  H.noalias() = V * D * V.transpose();

  // Invert D using eigenvalue-wise inversion.
  eigenmath::MatrixXd D_inverse =
      eigenmath::MatrixXd::Zero(control_dim, control_dim);
  D_inverse.diagonal() = D.diagonal().cwiseInverse();
  eigenmath::MatrixXd H_inverse = V * D_inverse * V.transpose();

  K.noalias() = -H_inverse * (B.transpose() * P * A);

  P_next = Q + A.transpose() * P * A;  // .noalias() is omitted here because P
                                       // and P_next could be the same matrix.
  P_next.noalias() -= K.transpose() * H * K;

  // Ensure symmetriy of P_next.
  P_next = (P_next + P_next.transpose()).eval() / 2.0;

  return absl::OkStatus();
}

}  // namespace intrinsic
