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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_ANALYTIC_GEOMETRY_CONSTRUCTIONS_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_ANALYTIC_GEOMETRY_CONSTRUCTIONS_H_

#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/motion_planning/path_planning/analytic_geometry/types.h"

namespace intrinsic {
namespace analytical_geometry {

absl::StatusOr<std::unique_ptr<Plane3d>> PlaneFromThreePoints(
    const Point3d& p1, const Point3d& p2, const Point3d& p3);
}
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_ANALYTIC_GEOMETRY_CONSTRUCTIONS_H_
