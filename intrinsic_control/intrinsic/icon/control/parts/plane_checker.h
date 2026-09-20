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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_PLANE_CHECKER_H_
#define INTRINSIC_ICON_CONTROL_PARTS_PLANE_CHECKER_H_

#include <stddef.h>

#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::icon {

// Implements a check for the safety with respect to cartesian limits or safety
// planes. Permits commanded poses which are outside the limits/planes but move
// in the direction of safety for all violated planes.
class PlaneChecker {
 public:
  // Factory to create class using standard CartesianLimits.
  static absl::StatusOr<std::unique_ptr<PlaneChecker>> Create(
      const CartesianLimits& cartesian_limits);

  // Factory to create class using an arbitrary set of safety planes.
  static absl::StatusOr<std::unique_ptr<PlaneChecker>> Create(
      const std::vector<eigenmath::Plane3dAligned>& safety_half_planes);

  // Checks the validity of commanding `commanded_pose` given `previous_pose`.
  RealtimeStatus CheckSetpoint(const Pose3d& previous_pose,
                               const Pose3d& commanded_pose);

 private:
  explicit PlaneChecker(
      std::vector<eigenmath::Plane3dAligned> safety_half_planes);

  // The set of safety planes to check against. Each plane is described by a
  // normal vector and an offset from the origin. A positive
  // signedDistance() from the plane is considered out (unsafe).
  std::vector<eigenmath::Plane3dAligned> safety_half_planes_;
};
}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_PARTS_PLANE_CHECKER_H_
