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

#include "intrinsic/icon/control/parts/plane_checker.h"

#include <memory>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::icon {
namespace {

constexpr double kDirectionDotProductMargin = 1e-9;
constexpr double kNormMargin = 1e-12;

// Create a set of safety half planes from standard Cartesian Limits. These are
// unit vectors pointing outwards offset by the min/max value for the
// translation limits.
std::vector<eigenmath::Plane3dAligned> CalculateHalfPlanesFromCartesianLimits(
    const CartesianLimits& cartesian_limits) {
  std::vector<eigenmath::Plane3dAligned> safety_half_planes({
      eigenmath::Plane3dAligned(
          {{1.0, 0.0, 0.0}, -cartesian_limits.max_translational_position[0]}),
      eigenmath::Plane3dAligned(
          {{-1.0, 0.0, 0.0}, cartesian_limits.min_translational_position[0]}),
      eigenmath::Plane3dAligned(
          {{0.0, 1.0, 0.0}, -cartesian_limits.max_translational_position[1]}),
      eigenmath::Plane3dAligned(
          {{0.0, -1.0, 0.0}, cartesian_limits.min_translational_position[1]}),
      eigenmath::Plane3dAligned(
          {{0.0, 0.0, 1.0}, -cartesian_limits.max_translational_position[2]}),
      eigenmath::Plane3dAligned(
          {{0.0, 0.0, -1.0}, cartesian_limits.min_translational_position[2]}),
  });
  return safety_half_planes;
}

// Checks if `pose` is in violation of the `safety_half_plane`.
bool OutsideHalfPlane(const eigenmath::Plane3dAligned& safety_half_plane,
                      const Pose3d& pose) {
  return safety_half_plane.signedDistance(pose.translation()) > 0;
}

// Checks if `pose` is within (in safety) for all safety half planes in
// `safety_half_planes`.
bool WithinAllHalfPlanes(
    const std::vector<eigenmath::Plane3dAligned>& safety_half_planes,
    const Pose3d& pose) {
  for (const auto& half_plane : safety_half_planes) {
    if (OutsideHalfPlane(half_plane, pose)) return false;
  }
  return true;
}

// If the `previous_pose` violates any safety plane then this checks whether the
// `commanded_pose` does not move further out from *any* of the violated safety
// planes.
RealtimeStatus MovingTowardsSafety(
    const std::vector<eigenmath::Plane3dAligned>& safety_half_planes,
    const Pose3d& previous_pose, const Pose3d& commanded_pose) {
  eigenmath::Vector3d move_direction_vector =
      commanded_pose.translation() - previous_pose.translation();
  // Do not normalize vectors which are near zero, otherwise sometimes the
  // subsequent check fails for a stationary setpoint.
  if (move_direction_vector.norm() > kNormMargin) {
    move_direction_vector.normalize();
  }
  for (const auto& half_plane : safety_half_planes) {
    if (OutsideHalfPlane(half_plane, commanded_pose)) {
      double movement_direction_plane_dot =
          move_direction_vector.dot(half_plane.normal());
      if (movement_direction_plane_dot > kDirectionDotProductMargin) {
        INTRINSIC_RT_LOG(WARNING)
            << "A safety plane violation was detected.\n"
            << "Normal Vector of violated plane: [" << half_plane.normal()[0]
            << ", " << half_plane.normal()[1] << ", " << half_plane.normal()[2]
            << "]\n"
            << "Current position: [" << commanded_pose.translation()[0] << ", "
            << commanded_pose.translation()[1] << ", "
            << commanded_pose.translation()[2] << "]\n"
            << "Previous position: [" << previous_pose.translation()[0] << ", "
            << previous_pose.translation()[1] << ", "
            << previous_pose.translation()[2] << "]\n";
        return InvalidArgumentError(RealtimeStatus::StrCat(
            "Safety plane violated and not returning towards safety. Normal "
            "Vector of violated plane: [",
            half_plane.normal()[0], ", ", half_plane.normal()[1], ", ",
            half_plane.normal()[2], "]"));
      }
    }
  }
  return OkStatus();
}
}  // namespace

RealtimeStatus PlaneChecker::CheckSetpoint(const Pose3d& previous_pose,
                                           const Pose3d& commanded_pose) {
  bool position_within_planes_commanded =
      WithinAllHalfPlanes(safety_half_planes_, commanded_pose);

  if (position_within_planes_commanded) {
    return OkStatus();
  }

  return MovingTowardsSafety(safety_half_planes_, previous_pose,
                             commanded_pose);
}

PlaneChecker::PlaneChecker(
    std::vector<eigenmath::Plane3dAligned> safety_half_planes)
    : safety_half_planes_(std::move(safety_half_planes)) {}

absl::StatusOr<std::unique_ptr<PlaneChecker>> PlaneChecker::Create(
    const CartesianLimits& cartesian_limits) {
  if (!cartesian_limits.IsValid()) {
    return InvalidArgumentError(
        "Tried to create a PlaneChecker with invalid cartesian_limits.");
  }
  return Create(CalculateHalfPlanesFromCartesianLimits(cartesian_limits));
}

absl::StatusOr<std::unique_ptr<PlaneChecker>> PlaneChecker::Create(
    const std::vector<eigenmath::Plane3dAligned>& safety_half_planes) {
  if (safety_half_planes.empty()) {
    return InvalidArgumentError(
        "Tried to create a PlaneChecker with empty safety_half_planes.");
  }
  std::vector<eigenmath::Plane3dAligned> normalized_safety_half_planes =
      safety_half_planes;
  // Normalize all safety plane normals such that they are unit vectors.
  for (auto& plane : normalized_safety_half_planes) {
    plane.normalize();
  }
  // Using `new` to access a non-public constructor.
  return absl::WrapUnique(
      new PlaneChecker(std::move(normalized_safety_half_planes)));
}
}  // namespace intrinsic::icon
