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

#include "intrinsic/icon/control/algorithms/selection_matrix_helpers.h"

#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/math/projection_matrix_utils.h"

namespace intrinsic::icon {

absl::StatusOr<eigenmath::Matrix6d> ComputeWrenchSelectionMatrixFromBooleans(
    bool translate_along_x, bool translate_along_y, bool translate_along_z,
    bool rotate_around_x, bool rotate_around_y, bool rotate_around_z) {
  eigenmath::Matrix6d P = eigenmath::Matrix6d::Zero();
  P(0, 0) = static_cast<double>(translate_along_x);
  P(1, 1) = static_cast<double>(translate_along_y);
  P(2, 2) = static_cast<double>(translate_along_z);
  P(3, 3) = static_cast<double>(rotate_around_x);
  P(4, 4) = static_cast<double>(rotate_around_y);
  P(5, 5) = static_cast<double>(rotate_around_z);

  if (P.trace() == 0.0) {
    return absl::FailedPreconditionError("At least one DoF must be free.");
  }
  return eigenmath::Matrix6d(P);
}

absl::StatusOr<eigenmath::Matrix6d>
ComputeWrenchSelectionMatrixForCompliantMotionInSubspaceSpannedBy(
    const std::vector<eigenmath::Vector3d>& v_translation,
    const std::vector<eigenmath::Vector3d>& v_rotation) {
  if (v_translation.size() > 3) {
    return absl::FailedPreconditionError(
        "Too many translational basis vectors specified.");
  }
  if (v_rotation.size() > 3) {
    return absl::FailedPreconditionError(
        "Too many rotational basis vectors specified.");
  }
  int ndof = v_translation.size() + v_rotation.size();
  if (ndof == 0) {
    return absl::FailedPreconditionError("At least one DoF must be free.");
  }

  // Fill matrix of basis vectors with all provided translational and rotational
  // DoF.
  eigenmath::Matrix6Nd C = eigenmath::Matrix6Nd::Zero(6, ndof);
  int current_col = 0;
  for (const auto& v_trans : v_translation) {
    C.col(current_col).head(3) = v_trans;
    current_col++;
  }
  for (const auto& v_rot : v_rotation) {
    C.col(current_col).tail(3) = v_rot;
    current_col++;
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(eigenmath::Matrix6d P,
                                ComputeOrthogonalColumnSpaceProjector(C));
  return eigenmath::Matrix6d(P);
}

absl::StatusOr<eigenmath::Matrix6d>
ComputeWrenchSelectionMatrixForCompliantMotionInSubspaceSpannedBy(
    const eigenmath::Matrix6Nd& C) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(eigenmath::Matrix6d P,
                                ComputeOrthogonalColumnSpaceProjector(C));
  return eigenmath::Matrix6d(P);
}

absl::StatusOr<eigenmath::Matrix6d>
ComputeWrenchSelectionMatrixForCompliantMotionInNullspaceOf(
    const eigenmath::Matrix6Nd& C) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(eigenmath::Matrix6d P,
                                ComputeOrthogonalNullSpaceProjector(C));
  return eigenmath::Matrix6d(P);
}

}  // namespace intrinsic::icon
