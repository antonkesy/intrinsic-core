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

#include "intrinsic/geometry/internal/distance_3/compute_bounded_hausdorff_distance.h"

#include <optional>

#include "CGAL/Polygon_mesh_processing/distance.h"
#include "CGAL/tags.h"
#include "absl/log/check.h"
#include "absl/status/statusor.h"
#include "intrinsic/geometry/internal/surface_mesh_3/surface_mesh_3d.h"

namespace intrinsic::geo {
absl::StatusOr<double> ComputeBoundedHausdorffDistance(
    const EpicSurfaceMesh3& epic_mesh,
    const EpicSurfaceMesh3& epic_reference_mesh,
    const std::optional<double> error_bound) {
  // The Hausdorff distance is not well defined if one of the meshes is empty.
  if (epic_mesh.is_empty()) {
    return absl::InvalidArgumentError(
        "Hausdorff distance cannot be determined for empty mesh");
  }
  if (epic_reference_mesh.is_empty()) {
    return absl::InvalidArgumentError(
        "Hausdorff distance cannot be determined for empty mesh");
  }

  if (!error_bound.has_value()) {
    return CGAL::Polygon_mesh_processing::bounded_error_Hausdorff_distance<
        CGAL::Sequential_tag>(epic_mesh, epic_reference_mesh);
  }

  if (*error_bound < 0.0) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Error Bound must be non-negative, %f was provided.", *error_bound));
  }

  return CGAL::Polygon_mesh_processing::bounded_error_Hausdorff_distance<
      CGAL::Sequential_tag>(epic_mesh, epic_reference_mesh, *error_bound);
}

}  // namespace intrinsic::geo
