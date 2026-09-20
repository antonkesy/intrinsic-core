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

#ifndef INTRINSIC_GEOMETRY_API_CONSTRUCT_SIMPLIFIED_MESH_H_
#define INTRINSIC_GEOMETRY_API_CONSTRUCT_SIMPLIFIED_MESH_H_

#include "absl/status/statusor.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/geometry/internal/util/timeout_predicate.h"
#include "intrinsic/util/thread/stop_token.h"

namespace intrinsic::geo {
// Returns a new geometry object by simplifying the input mesh using edge
// collapse with the quadric error metric. The simplification is controlled by
// the `max_hausdorff_distance`, which specifies the maximum allowed
// geometric error between the original and simplified mesh.
absl::StatusOr<Geometry> ConstructSimplifiedMesh(
    const Geometry& geo, double max_hausdorff_distance,
    const StopToken& stop_token,
    const TimeoutPredicate& custom_early_stop_predicate);

}  // namespace intrinsic::geo
#endif  // INTRINSIC_GEOMETRY_API_CONSTRUCT_SIMPLIFIED_MESH_H_
