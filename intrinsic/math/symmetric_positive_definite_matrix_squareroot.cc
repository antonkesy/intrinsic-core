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

#include "intrinsic/math/symmetric_positive_definite_matrix_squareroot.h"

#include "Eigen/Eigenvalues"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/ipow.h"

namespace intrinsic {

namespace {

constexpr double kSmallThreshold = 1e-6;

}  // namespace

icon::RealtimeStatusOr<eigenmath::MatrixNd>
SymmetricPositiveDefiniteMatrixSquareRoot(const eigenmath::MatrixNd& mat) {
  if (!mat.isApprox(mat.transpose(), kSmallThreshold)) {
    return icon::InvalidArgumentError("Input matrix is not symmetric.");
  }

  // Diagonalize input matrix.
  const Eigen::SelfAdjointEigenSolver<eigenmath::MatrixNd> solver(mat);
  const eigenmath::VectorNd evals = solver.eigenvalues();
  const eigenmath::MatrixNd V = solver.eigenvectors();

  if (evals.minCoeff() < 0.0) {
    return icon::InvalidArgumentError(
        "Input matrix is not positive semi-definite.");
  }

  // Take square-root of eigenvalues.
  eigenmath::VectorNd evals_sqrt = evals.cwiseSqrt();

  // Reconstruct matrix square root from Eigen-basis.
  return eigenmath::MatrixNd(V * evals_sqrt.asDiagonal() * V.inverse());
}

namespace {

// A matrix square root computation routine specific for the entropy-regulated
// wasserstein distance interpolation. Here, due to the fact that the product of
// two different symmetric positive definite matrices is generally not
// symmetric, we need a general eigensolver.
icon::RealtimeStatusOr<eigenmath::MatrixNd> PositiveDefiniteMatrixSquareRoot(
    const eigenmath::MatrixNd& mat) {
  // Diagonalize input matrix.
  const Eigen::EigenSolver<eigenmath::MatrixNd> solver(mat);
  eigenmath::VectorNd evals = solver.eigenvalues().real();
  const eigenmath::MatrixNd V = solver.eigenvectors().real();

  if (solver.eigenvalues().imag().cwiseAbs().maxCoeff() > kSmallThreshold) {
    return icon::InvalidArgumentError(
        "Eigen decomposition has imaginary eigenvalues.");
  }

  if (evals.minCoeff() < kSmallThreshold) {
    return icon::InvalidArgumentError(
        icon::FixedStrCat<icon::RealtimeStatus::kMaxMessageLength>(
            "Input matrix is not positive definite, smallest eigenvalue is ",
            evals.minCoeff()));
  }

  // Take square-root of eigenvalues.
  eigenmath::VectorNd evals_sqrt = evals.cwiseSqrt();

  // Reconstruct matrix square root from Eigen-basis.
  return eigenmath::MatrixNd(V * evals_sqrt.asDiagonal() * V.inverse());
}

}  // namespace

absl::StatusOr<EntropyRegulatedWassersteinDistanceInterpolation>
EntropyRegulatedWassersteinDistanceInterpolation::Create(
    const eigenmath::MatrixNd& mat_1, const eigenmath::MatrixNd& mat_2) {
  // Check if matrix dimensions coherent.
  if (mat_1.rows() != mat_2.rows() || mat_1.cols() != mat_2.cols()) {
    return absl::InvalidArgumentError(
        "The input matrices must have the same dimensions.");
  }

  // Check if input matrices are symmetric.
  if (!mat_1.transpose().isApprox(mat_1, kSmallThreshold) ||
      !mat_2.transpose().isApprox(mat_2, kSmallThreshold)) {
    return absl::InvalidArgumentError("The input matrices must be symmetric.");
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const eigenmath::MatrixNd mat_1_times_mat_2_sqrt,
      PositiveDefiniteMatrixSquareRoot(mat_1 * mat_2));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const eigenmath::MatrixNd mat_2_times_mat_1_sqrt,
      PositiveDefiniteMatrixSquareRoot(mat_2 * mat_1));

  return EntropyRegulatedWassersteinDistanceInterpolation(
      mat_1, mat_2, mat_1_times_mat_2_sqrt, mat_2_times_mat_1_sqrt);
}

icon::RealtimeStatusOr<eigenmath::MatrixNd>
EntropyRegulatedWassersteinDistanceInterpolation::Interpolate(
    double alpha) const {
  if (alpha < 0.0 || alpha > 1.0) {
    return icon::InvalidArgumentError(
        icon::FixedStrCat<icon::RealtimeStatus::kMaxMessageLength>(
            "Alpha must be in [0, 1], but got ", alpha));
  }

  return eigenmath::MatrixNd(
      ::intrinsic::IPow(1.0 - alpha, 2) * mat_1_ +
      ::intrinsic::IPow(alpha, 2) * mat_2_ +
      alpha * (1.0 - alpha) *
          (mat_1_times_mat_2_sqrt_ + mat_2_times_mat_1_sqrt_));
}

}  // namespace intrinsic
