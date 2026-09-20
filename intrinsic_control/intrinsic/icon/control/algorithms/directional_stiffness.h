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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_DIRECTIONAL_STIFFNESS_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_DIRECTIONAL_STIFFNESS_H_

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic::icon {

// Defines the stiffness parameters for a given motion direction.
// - 0 along the `translational_motion_direction`, such that the robot can move
// freely along this direction or its opposite direction, without being blocked
// by the stiffness.
// - `translational_stiffness_orthogonal_to_motion_direction` orthogonal to the
// motion direction.
// - `rotational_stiffness_orthogonal_to_motion_direction` along directions
// perpendicular/orthogonal to the motion direction.
struct DirectionalStiffnessInput {
  // Virtual translational stiffness of the controller orthogonal to the motion
  // direction.
  double translational_stiffness_orthogonal_to_motion_direction;

  // Virtual rotational stiffness of the controller orthogonal to translational
  // motion direction.
  double rotational_stiffness_orthogonal_to_motion_direction;
};

// Computes the stiffness matrix for a set of translational motion directions.
//
// `translational_directions` defines up to three motion directions.
// The stiffness matrix is set to zero in all translational directions. Other
// directions are set to the parameters defined in the `stiffness` struct.
//
// `rotational_directions` defines up to three rotational motion directions. The
// stiffness matrix is set to zero in all rotational directions. Other
// directions are set to the parameters defined in the `stiffness` struct.
//
// Returns an error if more than three translational or rotational directions
// are provided.
//
// Returns a stiffness matrix which is expressed in the same frame as where
// `translational_directions` vectors are defined.
absl::StatusOr<eigenmath::Matrix6d> ComputeDirectionalStiffness(
    absl::Span<const eigenmath::Vector3d> translational_directions,
    absl::Span<const eigenmath::Vector3d> rotational_directions,
    const DirectionalStiffnessInput& stiffness);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_DIRECTIONAL_STIFFNESS_H_
