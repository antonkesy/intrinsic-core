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

#ifndef INTRINSIC_KINEMATICS_SKELETON_UTIL_H_
#define INTRINSIC_KINEMATICS_SKELETON_UTIL_H_

#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"

namespace intrinsic {
namespace kinematics {

absl::StatusOr<std::unique_ptr<kinematics::Skeleton>> GetSkeletonForRobot(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot);

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_SKELETON_UTIL_H_
