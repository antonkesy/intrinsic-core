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

#ifndef INTRINSIC_GEOMETRY_INTERNAL_DISTANCE_3_COMPUTE_BOUNDED_HAUSDORFF_DISTANCE_H_
#define INTRINSIC_GEOMETRY_INTERNAL_DISTANCE_3_COMPUTE_BOUNDED_HAUSDORFF_DISTANCE_H_

#include "absl/status/statusor.h"
#include "intrinsic/geometry/internal/surface_mesh_3/surface_mesh_3d.h"

namespace intrinsic::geo {
// Returns  an estimate on the Hausdorff distance from `epic_mesh` to
// `epic_reference` that is at most `error_bound` away from the actual Hausdorff
// distance. Neither of the meshes is allowed to be the empty mesh as the
// HausdorffDistance, like any distance, is not well defined.
absl::StatusOr<double> ComputeBoundedHausdorffDistance(
    const EpicSurfaceMesh3& epic_mesh,
    const EpicSurfaceMesh3& epic_reference_mesh,
    const std::optional<double> error_bound = std::nullopt);

}  // namespace intrinsic::geo
#endif  // INTRINSIC_GEOMETRY_INTERNAL_DISTANCE_3_COMPUTE_BOUNDED_HAUSDORFF_DISTANCE_H_
