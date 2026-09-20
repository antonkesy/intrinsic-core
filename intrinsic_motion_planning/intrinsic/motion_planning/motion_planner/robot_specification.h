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

#ifndef INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_ROBOT_SPECIFICATION_H_
#define INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_ROBOT_SPECIFICATION_H_

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/motion_planning/path_planning/planners/linear_cartesian_motion_path_planner_configs.pb.h"
#include "intrinsic/motion_planning/path_planning/planners/pipeline_configs.pb.h"
#include "intrinsic/motion_planning/path_planning/planners/rrt_configs.pb.h"
#include "intrinsic/motion_planning/path_planning/planners/shortcutter_configs.pb.h"
#include "intrinsic/motion_planning/proto/motion_target.pb.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_specification.pb.h"
#include "intrinsic/motion_planning/proto/v1/robot_specification.pb.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"

namespace intrinsic {
// Struct to help storing robot related information.
struct RobotSpecification {
  const object_world::KinematicObject* robot;
  const CartesianLimits cart_limits;
  const JointLimits application_limits;
  const eigenmath::VectorNd start_configuration;

  RobotSpecification(const object_world::KinematicObject* robot,
                     const CartesianLimits& cart_limits,
                     const JointLimits& application_limits,
                     const eigenmath::VectorNd& start_configuration)
      : robot(robot),
        cart_limits(cart_limits),
        application_limits(application_limits),
        start_configuration(start_configuration) {}

  static absl::StatusOr<RobotSpecification> Create(
      const object_world::ObjectWorld& object_world,
      const intrinsic_proto::motion_planning::v1::RobotSpecification&
          robot_specification);
};
}  // namespace intrinsic
#endif  // INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_ROBOT_SPECIFICATION_H_
