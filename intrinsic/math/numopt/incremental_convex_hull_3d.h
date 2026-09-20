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

#ifndef INTRINSIC_MATH_NUMOPT_INCREMENTAL_CONVEX_HULL_3D_H_
#define INTRINSIC_MATH_NUMOPT_INCREMENTAL_CONVEX_HULL_3D_H_

#include <array>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic {

// A facet of a convex hull in 3D, described by its supporting plane.
struct ConvexHullFacetPlane3d {
  // Unit normal pointing away from the interior of the hull.
  eigenmath::Vector3d normal;

  // Signed distance of the plane from the origin, so that the facet lies on
  // `normal.dot(x) == offset` and the hull satisfies `normal.dot(x) <= offset`.
  double offset = 0.0;

  // Indices into the input point set of the three points spanning this facet,
  // wound counter-clockwise as seen from outside.
  //
  // `normal` and `offset` are derived from these three points, and that
  // derivation loses accuracy when they are nearly collinear. Callers that can
  // express what they need in terms of the original points should prefer these
  // indices over the plane.
  std::array<int, 3> point_indices = {-1, -1, -1};
};

// Computes the supporting planes of the facets of the convex hull of `points`,
// one entry per facet. Coplanar facets are not merged, so callers must tolerate
// duplicate planes.
//
// Returns an error if there are fewer than four points, if they are degenerate
// (coincident, collinear or coplanar within tolerance), or if rounding makes
// the construction inconsistent.
absl::StatusOr<std::vector<ConvexHullFacetPlane3d>>
ComputeConvexHullFacetPlanes3d(absl::Span<const eigenmath::Vector3d> points);

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_NUMOPT_INCREMENTAL_CONVEX_HULL_3D_H_
