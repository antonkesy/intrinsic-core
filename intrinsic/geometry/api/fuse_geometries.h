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

#ifndef INTRINSIC_GEOMETRY_API_FUSE_GEOMETRIES_H_
#define INTRINSIC_GEOMETRY_API_FUSE_GEOMETRIES_H_

#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/geometry/api/affine_transform_of_geometry.h"

namespace intrinsic::geo {
// Fuse all geometry into one joint representation, aka Merge.
// This is a more efficient version of calling ApplyTransform on each index and
// then calling FuseGeometries with the resulting Geometry vector.
absl::StatusOr<TransformedGeometry> FuseGeometries(
    const std::vector<TransformedGeometry>& geos);

}  // namespace intrinsic::geo
#endif  // INTRINSIC_GEOMETRY_API_FUSE_GEOMETRIES_H_
