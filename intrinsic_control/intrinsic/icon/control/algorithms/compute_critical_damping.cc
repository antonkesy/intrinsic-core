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

#include "intrinsic/icon/control/algorithms/compute_critical_damping.h"

#include <cmath>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/symmetric_positive_definite_matrix_squareroot.h"

namespace intrinsic::icon {

RealtimeStatusOr<double> ComputeCriticalDamping(double m, double k) {
  if (m <= 0.0) {
    return FailedPreconditionError("Mass must be positive-valued.");
  }

  if (k <= 0.0) {
    return FailedPreconditionError("Stiffness must be positive-valued.");
  }

  return 2.0 * std::sqrt(m * k);
}

RealtimeStatusOr<eigenmath::MatrixNd> ComputeCriticalDamping(
    const eigenmath::MatrixNd& M, const eigenmath::MatrixNd& K) {
  const auto maybe_M_sqrt = SymmetricPositiveDefiniteMatrixSquareRoot(M);
  if (!maybe_M_sqrt.ok()) {
    return FailedPreconditionError(
        icon::FixedStrCat<RealtimeStatus::kMaxMessageLength>(
            "Computing the square root of the inertia matrix failed.",
            maybe_M_sqrt.status().message()));
  }
  const eigenmath::MatrixNd M_sqrt_inv = maybe_M_sqrt.value().inverse();

  const auto maybe_MKM_sqrt =
      SymmetricPositiveDefiniteMatrixSquareRoot(M_sqrt_inv * K * M_sqrt_inv);
  if (!maybe_MKM_sqrt.ok()) {
    return icon::FailedPreconditionError(
        icon::FixedStrCat<RealtimeStatus::kMaxMessageLength>(
            "Invalid matrices for computing optimal damping.",
            maybe_MKM_sqrt.status().message()));
  }
  eigenmath::MatrixNd D = 2.0 * maybe_M_sqrt.value() * maybe_MKM_sqrt.value() *
                          maybe_M_sqrt.value();
  return D;
}

RealtimeStatusOr<eigenmath::MatrixNd>
ComputeCriticalDampingFromInverseInertiaMatrix(const eigenmath::MatrixNd& Minv,
                                               const eigenmath::MatrixNd& K) {
  const auto maybe_Minv_sqrt = SymmetricPositiveDefiniteMatrixSquareRoot(Minv);
  if (!maybe_Minv_sqrt.ok()) {
    return icon::FailedPreconditionError(
        icon::FixedStrCat<RealtimeStatus::kMaxMessageLength>(
            "Computing the square root of the inverse inertia matrix failed.",
            maybe_Minv_sqrt.status().message()));
  }

  const auto maybe_MKM_sqrt = SymmetricPositiveDefiniteMatrixSquareRoot(
      maybe_Minv_sqrt.value() * K * maybe_Minv_sqrt.value());
  if (!maybe_MKM_sqrt.ok()) {
    return icon::FailedPreconditionError(
        icon::FixedStrCat<RealtimeStatus::kMaxMessageLength>(
            "Invalid matrices for computing optimal damping.",
            maybe_MKM_sqrt.status().message()));
  }
  const eigenmath::MatrixNd M_sqrt = maybe_Minv_sqrt.value().inverse();

  eigenmath::MatrixNd D = 2.0 * M_sqrt * maybe_MKM_sqrt.value() * M_sqrt;

  return D;
}

}  // namespace intrinsic::icon
