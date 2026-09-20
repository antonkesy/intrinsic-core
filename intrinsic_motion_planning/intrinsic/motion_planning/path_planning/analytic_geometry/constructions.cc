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

#include "intrinsic/motion_planning/path_planning/analytic_geometry/constructions.h"

#include <memory>
#include <utility>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/motion_planning/path_planning/analytic_geometry/types.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace analytical_geometry {

absl::StatusOr<std::unique_ptr<Plane3d>> PlaneFromThreePoints(
    const Point3d& p1, const Point3d& p2, const Point3d& p3) {
  eigenmath::Vector3d normal = ((p2 - p1).cross(p3 - p1)).normalized();
  INTR_ASSIGN_OR_RETURN(auto plane, Plane3d::Create(/*origin=*/p1, normal));
  return std::make_unique<Plane3d>(std::move(plane));
}

}  // namespace analytical_geometry
}  // namespace intrinsic
