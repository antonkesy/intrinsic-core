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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_SELECTION_MATRIX_HELPERS_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_SELECTION_MATRIX_HELPERS_H_

#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// Returns a wrench selection matrix which limits wrench-sensitivity to the
// directions indicated by booleans set to 'true'. The returned 6d selection
// matrix projects into the column space of the selected DoF, all remaining DoF
// are eliminated. Returns an absl::FailedPreconditionError in case no DoF were
// selected.
absl::StatusOr<eigenmath::Matrix6d> ComputeWrenchSelectionMatrixFromBooleans(
    bool translate_along_x, bool translate_along_y, bool translate_along_z,
    bool rotate_around_x, bool rotate_around_y, bool rotate_around_z);

// Computes a wrench selection matrix which limits force-sensitivity
// to the subspace spanned by the 3d translation vectors 'v_translation',
// and limits torque-sensitivity to the subspace spanned by the 3d rotation
// vectors 'v_rotation'. The returned 6d selection matrix projects into the
// column space the provided vectors, all remaining DoF are eliminated. Returns
// absl::kFailedPrecondition in case of inconsistent basis vectors.
absl::StatusOr<eigenmath::Matrix6d>
ComputeWrenchSelectionMatrixForCompliantMotionInSubspaceSpannedBy(
    const std::vector<eigenmath::Vector3d>& v_translation,
    const std::vector<eigenmath::Vector3d>& v_rotation = {});

// Computes a wrench selection matrix which limits wrench-sensitivity
// to the subspace spanned by the columns of matrix 'C'. The returned 6d
// selection matrix projects into the column space of 'C'. Returns
// absl::kFailedPrecondition in case of inconsistent basis vectors.
absl::StatusOr<eigenmath::Matrix6d>
ComputeWrenchSelectionMatrixForCompliantMotionInSubspaceSpannedBy(
    const eigenmath::Matrix6Nd& C);

// Computes a wrench selection matrix which eliminates wrench-sensitivity in the
// directions of the columns of matrix 'C'. The returned 6d selection
// matrix projects into the nullspace of 'C'. Returns absl::kFailedPrecondition
// in case of dimension mismatch or inconsistent basis vectors.
absl::StatusOr<eigenmath::Matrix6d>
ComputeWrenchSelectionMatrixForCompliantMotionInNullspaceOf(
    const eigenmath::Matrix6Nd& C);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_SELECTION_MATRIX_HELPERS_H_
