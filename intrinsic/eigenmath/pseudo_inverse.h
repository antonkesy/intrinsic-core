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

#ifndef INTRINSIC_EIGENMATH_PSEUDO_INVERSE_H_
#define INTRINSIC_EIGENMATH_PSEUDO_INVERSE_H_

#include <cstddef>
#include <optional>
#include <tuple>
#include <utility>

#include "Eigen/Core"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/ipow.h"

namespace intrinsic::icon {

// Computes matrix pseudo-inverse with Tikhonov regularization (=damping factor
// `lambda`) by Singular Value Decomposition:
//
//  m = U * S * V^T
//
// where U and V are orthonormal matrices and S is a diagonal matrix.
//
// The pseudo-inverse is computed as
//
//  m^dagger = V * S * U^T * inv(U * S * V^T * V * S * U^T)
//           = V * S * U^T * U * inv(S * S) * U^T
//           = V * S^(-1) * U^T
//
//  For the Tikhonov regularized pseudo-inverse this becomes:
//   m^dagger = V * S^dagger * U^T
//
// where S^dagger is the diagonal matrix with entries
//
//  S^dagger_{ii} = S_{ii} / (S_{ii}^2 + lambda^2)
//
// for i = 0, ..., m.cols()-1.
//
// The pseudo-inverse can be used to find solutions to over- or underconstrained
// systems of equations. For x=ComputePseudoInverse(A)*b, x minimizes
// norm(A*x - b).
// Returns an error status if `fail_if_badly_conditioned` is set to `true`,
// which triggers as soon as the condition number is higher than
// `condition_number_threshold`.
// TODO(b/394004292): Avoid malloc.
inline RealtimeStatusOr<eigenmath::MatrixNMd> ComputePseudoInverse(
    const eigenmath::MatrixNMd& m, const double lambda,
    bool fail_if_badly_conditioned, double condition_number_threshold = 1e12)
    INTRINSIC_SUPPRESS_REALTIME_CHECK {
  using SVD = Eigen::JacobiSVD<eigenmath::MatrixNMd>;
  SVD svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
  SVD::SingularValuesType singular_values = svd.singularValues();

  const double max_singular_value = singular_values(0);
  const double min_singular_value = singular_values(singular_values.size() - 1);
  const double condition_number = max_singular_value / min_singular_value;

  if (fail_if_badly_conditioned &&
      ::intrinsic::AlmostEquals(max_singular_value, 0.0)) {
    return InvalidArgumentError(
        "Matrix is all-zero, cannot compute pseudo-inversion.");
  }

  if (fail_if_badly_conditioned &&
      condition_number >= condition_number_threshold) {
    return InvalidArgumentError("Matrix is ill-conditioned");
  }

  // Compute regularized inverse singular value matrix.
  const double lambda_squared = ::intrinsic::IPow(lambda, 2);
  eigenmath::MatrixNMd S(m.rows(), m.cols());
  S.setZero();
  for (size_t i = 0; i < singular_values.size(); i++) {
    S(i, i) = (singular_values(i)) /
              (singular_values(i) * singular_values(i) + lambda_squared);
  }

  // Compose and return pseudo-inverse.
  return eigenmath::MatrixNMd(svd.matrixV() * S.transpose() *
                              svd.matrixU().transpose());
}

inline icon::RealtimeStatusOr<eigenmath::MatrixNMd> ComputePseudoInverse(
    const eigenmath::MatrixNMd& m, const double lambda = 0.0) {
  return ComputePseudoInverse(m, lambda, /*fail_if_badly_conditioned=*/false);
}

// Computes the smooth right pseudo-inverse and the regularized inverse of the
// Gram matrix.
//
// Returns a tuple containing:
//   - std::get<0>: The smooth right pseudo-inverse matrix
//                  m^dagger = W * M^T * (M * W * M^T)^(-1).
//   - std::get<1>: The regularized inverse of the Gram matrix
//                  (M * W * M^T)^(-1).
//
// See ComputeSmoothRightPseudoInverse for algorithmic details.
inline RealtimeStatusOr<std::tuple<eigenmath::MatrixNMd, eigenmath::MatrixNMd>>
ComputeSmoothRightPseudoInverseAndInverseMatrix(
    const eigenmath::MatrixNMd& M, double condition_number_threshold = 1500,
    double epsilon = 1e-6,
    std::optional<eigenmath::MatrixNMd> W = std::nullopt) {
  using SVD = Eigen::JacobiSVD<eigenmath::MatrixNMd>;

  // If W is not a scalar, it must be a square matrix with appropriate size.
  if (W.has_value()) {
    if (M.cols() != W->cols() || W->rows() != W->cols()) {
      return InvalidArgumentError(
          "Matrix M and weight matrix W have incompatible dimensions.");
    }
  }

  if (M.cols() < M.rows()) {
    return InvalidArgumentError(
        "Matrix M must be square or have more columns than rows.");
  }

  if (condition_number_threshold < 0.0) {
    return InvalidArgumentError(
        "Condition number threshold must be greater or equal to zero.");
  }

  if (epsilon <= 0.0) {
    return InvalidArgumentError(
        "Regularization epsilon must be greater than zero.");
  }

  // Compute SVD of M*W*M^T.
  eigenmath::MatrixNMd M_W_MT;
  if (!W.has_value()) {
    M_W_MT = M * M.transpose();
  } else {
    M_W_MT = M * *W;
    M_W_MT = M_W_MT * M.transpose();
  }
  SVD svd(M_W_MT, Eigen::ComputeFullU | Eigen::ComputeFullV);
  SVD::SingularValuesType singular_values = svd.singularValues();
  const double max_singular_value = singular_values(0);

  if (::intrinsic::AlmostEquals(max_singular_value, 0.0)) {
    return InvalidArgumentError(
        "Matrix is all-zero, cannot compute pseudo-inversion.");
  }

  // Compute regularized inverse singular value matrix.
  eigenmath::MatrixNMd S(M_W_MT.rows(), M_W_MT.cols());
  S.setZero();
  for (size_t i = 0; i < singular_values.size(); i++) {
    double cn_i = max_singular_value / (singular_values(i) + 1.e-10);
    double lambda_i_squared = 0.0;

    if (cn_i > condition_number_threshold) {
      double ratio = condition_number_threshold / cn_i;
      // This is the bisquare smoothing function which has zero derivatives at
      // ratio=0 and ratio=1 and thus a smooth onset and offset of the
      // regularization.
      lambda_i_squared =
          (1 - ratio * ratio) * (1 - ratio * ratio) * (1.0 / epsilon);
    }
    S(i, i) = (singular_values(i)) /
              (singular_values(i) * singular_values(i) + lambda_i_squared);
  }
  // The regularized inverse of the Gram matrix.
  eigenmath::MatrixNMd gram_inverse =
      svd.matrixV() * S * svd.matrixU().transpose();

  // Compose and return pseudo-inverse and inverse Gram matrix.
  if (!W.has_value()) {
    return std::make_tuple(eigenmath::MatrixNMd(M.transpose() * gram_inverse),
                           eigenmath::MatrixNMd(gram_inverse));
  }
  return std::make_tuple(
      eigenmath::MatrixNMd(*W * M.transpose() * gram_inverse),
      eigenmath::MatrixNMd(gram_inverse));
}

// Computes matrix right pseudo-inverse with SVD smoothed regularization,
// including an optional weight matrix 'W'. This follows largely the paper:
//
// Chiaverini, S. (1997). Singularity-robust task-priority redundancy resolution
// for real-time kinematic control of robot manipulators. IEEE Transactions on
// Robotics and Automation, 13(3), 398–410. doi: 10.1109/70.585902
//
// The SVD of the weighted matrix inversion is:
//
//  M * W * M^T = U * S * V^T
//
// where U and V are orthonormal matrices, S is a diagonal matrix, and W is a
// positive definite square weight matrix.
//
// The pseudo-inverse is computed as
//
//  m^dagger = W * M^T * inv(U * S * V^T)
//           = W * M^T * V * inv(S) * U^T
//
// For the smoothed regularized pseudo-inverse this becomes:
//  m^dagger = W * M^T * V * S^dagger * U^T
//
// where S^dagger is the diagonal matrix with entries
//
//  S^dagger_{ii} = S_{ii} / (S_{ii}^2 + lambda_{i}^2)
//
//  cn_{i} = S_{0} / S_{ii}
//
//  lambda_{i}^2 = (1 - (condition_number_threshold / cn_{i})^2)^2 *
//                 (1 / epsilon)  for cn_{i} > condition_number_threshold
//
//  lambda_{i}^2 = 0              for cn_{i} <= condition_number_threshold
//
// for i = 0, ..., m.cols()-1.
//
// The epsilon regularizer is a small positive number to avoid numerical issues,
// equivalent to a ridge regression regularization.
//
// In contrast to the Tikhonov regularization, the lambda_{i} are adapted per
// dimension to the condition number of this dimension, and regularization is
// added in a smooth continuous/differentiable way. This ensures that
// regularization does not create discontinuous jumps in the results.
//
// Returns an error status if M has all zero singular values or matrix M and
// weight matrix W have incompatible dimensions or incorrect input arguments.
//
// Default values for `epsilon` and `condition_number_threshold` are set to
// conservative values.
inline RealtimeStatusOr<eigenmath::MatrixNMd> ComputeSmoothRightPseudoInverse(
    const eigenmath::MatrixNMd& M, double condition_number_threshold = 1500,
    double epsilon = 1e-6,
    std::optional<eigenmath::MatrixNMd> W = std::nullopt) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto pinv_and_inv_matrix,
                                ComputeSmoothRightPseudoInverseAndInverseMatrix(
                                    M, condition_number_threshold, epsilon, W));

  return std::move(std::get<0>(pinv_and_inv_matrix));
}

}  // namespace intrinsic::icon

#endif  // INTRINSIC_EIGENMATH_PSEUDO_INVERSE_H_
