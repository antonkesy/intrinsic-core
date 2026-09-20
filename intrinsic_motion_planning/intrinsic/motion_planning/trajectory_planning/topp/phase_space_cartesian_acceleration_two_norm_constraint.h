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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_PHASE_SPACE_CARTESIAN_ACCELERATION_TWO_NORM_CONSTRAINT_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_PHASE_SPACE_CARTESIAN_ACCELERATION_TWO_NORM_CONSTRAINT_H_

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic {
namespace topp {

// Represents a linear half-space approximation of a Cartesian 2-norm limit.
// This constraint maps a 3D spherical continuous limit into an 8-sided 2D
// polygon in the (b, b') phase plane. Due to symmetry, only the 4 positive
// half-spaces are stored. To enforce the bounds, apply both positive and
// negative projections:
//      h0[i] * b + h1[i] * b'  <= effective_limit
//    -(h0[i] * b + h1[i] * b') <= effective_limit
struct PhaseSpaceCartAccTwoNormConstraint {
  // Coefficients for the squared path velocity (b).
  eigenmath::Vector4d h0;

  // Coefficients for the squared path velocity first derivative (b').
  eigenmath::Vector4d h1;

  // The maximum allowable projection limit in the specified normal directions.
  double effective_limit;
};

// Constructs a phase space constraint for Cartesian acceleration.
// Maps the true 3D Cartesian acceleration limit into an adaptive, tightly bound
// octagon in the 2D phase plane via Cholesky decomposition.
// - `j_qpp_plus_jp_qp` and `j_qp` represent products of the Jacobian and its
//   first derivative with the path derivatives. These vectors constitute the
//   basis functions for expressing the Cartesian acceleration in terms of phase
//   variables.
// - `cartesian_acceleration_limit` is the physical limit to enforce (e.g.,
//   m/s^2 for translation and rad/sec^2 for rotation).
// - `use_inner_approximation`: If true, scales the limits to guarantee the
//   polygon corners never exceed the true 2-norm (strict safety at the cost of
//   at most 8% conservativeness overall in some directions).
// Returns the precomputed constraint components.
// For details of the algorithm, refer to:
// go/intrinsic-cholesky-adaptive-phase-space-approximation
absl::StatusOr<PhaseSpaceCartAccTwoNormConstraint>
ConstructPhaseSpaceCartAccTwoNormConstraint(
    const eigenmath::Vector3d& j_qpp_plus_jp_qp,
    const eigenmath::Vector3d& j_qp, const double cartesian_acceleration_limit,
    const bool use_inner_approximation = true);

}  // namespace topp
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_PHASE_SPACE_CARTESIAN_ACCELERATION_TWO_NORM_CONSTRAINT_H_
