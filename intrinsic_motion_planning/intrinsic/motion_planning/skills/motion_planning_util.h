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

#ifndef INTRINSIC_MOTION_PLANNING_SKILLS_MOTION_PLANNING_UTIL_H_
#define INTRINSIC_MOTION_PLANNING_SKILLS_MOTION_PLANNING_UTIL_H_

#include <optional>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/motion_planning/proto/v1/motion_planner_service.pb.h"
#include "intrinsic/world/objects/kinematic_object.h"
#include "intrinsic/world/proto/object_world_service.pb.h"

namespace intrinsic {
namespace motion_planning {

// Create a default CartesianLimits object, using reasonable defaults that can
// be as generic limits in absence limits specific to a particular robot or
// application.
CartesianLimits CreateDefaultCartLimits(bool set_infinite_jerk_limits = false);

intrinsic_proto::motion_planning::v1::RobotSpecification
CreateRobotSpecification(
    const world::KinematicObject& robot,
    const std::optional<Eigen::VectorXd>& start_configuration = std::nullopt);

}  // namespace motion_planning
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_SKILLS_MOTION_PLANNING_UTIL_H_
