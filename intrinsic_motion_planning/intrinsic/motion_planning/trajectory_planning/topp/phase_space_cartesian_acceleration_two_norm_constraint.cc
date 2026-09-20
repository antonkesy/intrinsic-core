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

#include "intrinsic/motion_planning/trajectory_planning/topp/phase_space_cartesian_acceleration_two_norm_constraint.h"

#include <algorithm>
#include <cmath>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic {
namespace topp {

absl::StatusOr<PhaseSpaceCartAccTwoNormConstraint>
ConstructPhaseSpaceCartAccTwoNormConstraint(
    const eigenmath::Vector3d& j_qpp_plus_jp_qp,
    const eigenmath::Vector3d& j_qp, const double cartesian_acceleration_limit,
    const bool use_inner_approximation) {
  if (cartesian_acceleration_limit <= 0.0) {
    return absl::InvalidArgumentError(
        "Cartesian acceleration limit must be strictly positive.");
  }

  // The Cartesian acceleration is:
  // cart_acc = j_qpp_plus_jp_qp * b + 0.5 * j_qp * bp

  // Construct the 2x2 ellipse covariance matrix Q components. Regularization
  // prevents matrix rank deficiency during perfectly linear motions.
  constexpr double kEllipseRegularization = 1.0e-9;
  const double q11 = j_qpp_plus_jp_qp.squaredNorm() + kEllipseRegularization;
  const double q22 = 0.25 * j_qp.squaredNorm() + kEllipseRegularization;
  const double q12 = 0.5 * j_qpp_plus_jp_qp.dot(j_qp);

  // Cholesky Decomposition Q = L * L^T
  // This analytically adapts the bounding polygon to the exact rotation and
  // skew of the ellipse, guaranteeing a maximum corner overshoot of ~8.24%.
  const double l11 = std::sqrt(q11);
  const double l21 = q12 / l11;
  const double l22 = std::sqrt(std::max(0.0, q22 - l21 * l21));

  // Adjust limit for inner approximation: cos(pi/8) approx 0.9238795325.
  constexpr double kCosinePiOverEight = 0.9238795325112867;
  const double max_cart_acceleration =
      use_inner_approximation
          ? (cartesian_acceleration_limit * kCosinePiOverEight)
          : cartesian_acceleration_limit;

  // Generate the 4 positive adaptive normal vectors [h0, h1].
  return PhaseSpaceCartAccTwoNormConstraint{
      /*h0=*/eigenmath::Vector4d(l11,                // 0 deg direction
                                 l11 * M_SQRT1_2,    // 45 deg direction
                                 0.0,                // 90 deg direction
                                 -l11 * M_SQRT1_2),  // 135 deg direction
      /*h1=*/
      eigenmath::Vector4d(l21,                        // 0 deg direction
                          (l21 + l22) * M_SQRT1_2,    // 45 deg direction
                          l22,                        // 90 deg direction
                          (-l21 + l22) * M_SQRT1_2),  // 135 deg direction
      /*effective_limit=*/max_cart_acceleration};
}

}  // namespace topp
}  // namespace intrinsic
