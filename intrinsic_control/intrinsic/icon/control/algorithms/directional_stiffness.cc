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

#include "intrinsic/icon/control/algorithms/directional_stiffness.h"

#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/projection_matrix_utils.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

namespace {

eigenmath::MatrixNMd DirectionMatrix(
    absl::Span<const eigenmath::Vector3d> directions) {
  eigenmath::MatrixNMd direction_matrix =
      eigenmath::MatrixNMd::Zero(3, directions.size());
  for (int i = 0; i < directions.size(); ++i) {
    direction_matrix.col(i) = directions[i].normalized();
  }
  return direction_matrix;
}

// Returns a stiffness matrix where the stiffness in the motion direction is
// zero and the stiffness in the orthogonal direction is
// `stiffness_orthogonal_to_motion_direction`.
absl::StatusOr<eigenmath::Matrix3d> ComputeStiffness(
    const eigenmath::MatrixNMd& direction_matrix,
    double stiffness_orthogonal_to_motion_direction) {
  eigenmath::Matrix3d nullspace_projection_matrix =
      eigenmath::Matrix3d::Identity();
  if (direction_matrix.cols()) {
    INTR_ASSIGN_OR_RETURN(
        nullspace_projection_matrix,
        ComputeOrthogonalNullSpaceProjector(direction_matrix));
  }
  return nullspace_projection_matrix *
         (eigenmath::Matrix3d::Identity() *
          stiffness_orthogonal_to_motion_direction) *
         nullspace_projection_matrix.transpose();
}

// The columns of `direction_matrix` define the motion directions.
absl::StatusOr<eigenmath::Matrix6d> ComputeDirectionalStiffness(
    const eigenmath::MatrixNMd& translational_direction_matrix,
    const eigenmath::MatrixNMd& rotational_direction_matrix,
    const DirectionalStiffnessInput& stiffness) {
  eigenmath::Matrix6d stiffness_matrix = eigenmath::Matrix6d::Zero();

  INTR_ASSIGN_OR_RETURN(
      (stiffness_matrix.block<3, 3>(0, 0)),
      ComputeStiffness(
          translational_direction_matrix,
          stiffness.translational_stiffness_orthogonal_to_motion_direction));
  INTR_ASSIGN_OR_RETURN(
      (stiffness_matrix.block<3, 3>(3, 3)),
      ComputeStiffness(
          rotational_direction_matrix,
          stiffness.rotational_stiffness_orthogonal_to_motion_direction));

  return stiffness_matrix;
}

}  // namespace

absl::StatusOr<eigenmath::Matrix6d> ComputeDirectionalStiffness(
    absl::Span<const eigenmath::Vector3d> translational_directions,
    absl::Span<const eigenmath::Vector3d> rotational_directions,
    const DirectionalStiffnessInput& stiffness) {
  eigenmath::MatrixNMd translational_direction_matrix =
      eigenmath::MatrixNMd::Zero(3, translational_directions.size());

  return ComputeDirectionalStiffness(DirectionMatrix(translational_directions),
                                     DirectionMatrix(rotational_directions),
                                     stiffness);
}

}  // namespace intrinsic::icon
