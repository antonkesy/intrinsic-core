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

#include "intrinsic/motion_planning/motion_planner/robot_specification.h"

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/motion_planning/path_planning/planners/linear_cartesian_motion_path_planner_configs.pb.h"
#include "intrinsic/motion_planning/path_planning/planners/pipeline_configs.pb.h"
#include "intrinsic/motion_planning/path_planning/planners/rrt_configs.pb.h"
#include "intrinsic/motion_planning/path_planning/planners/shortcutter_configs.pb.h"
#include "intrinsic/motion_planning/proto/motion_planner_service_proto_utils.h"
#include "intrinsic/motion_planning/proto/motion_target.pb.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_planner_service.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_specification.pb.h"
#include "intrinsic/motion_planning/proto/v1/robot_specification.pb.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"

namespace intrinsic {

absl::StatusOr<RobotSpecification> RobotSpecification::Create(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::motion_planning::v1::RobotSpecification&
        robot_specification) {
  // Parse robot from proto
  INTR_ASSIGN_OR_RETURN(
      const object_world::KinematicObject* robot,
      GetRobot(robot_specification.robot_reference(), &object_world));

  // Parse default Cartesian limits
  INTR_ASSIGN_OR_RETURN(CartesianLimits default_cart_limits,
                        robot->GetCartesianLimits());

  // Parse planning and system robot joint limits
  INTR_ASSIGN_OR_RETURN(const JointLimitsXd application_limits_xd,
                        robot->GetJointApplicationLimits());
  // Convert JointLimitsXd to JointLimits.
  INTR_ASSIGN_OR_RETURN(const JointLimits application_limits,
                        ToJointLimits(application_limits_xd));

  // Start configuration. Default from world if not provided by user.
  INTR_ASSIGN_OR_RETURN(eigenmath::VectorXd start_configuration,
                        robot->GetJointPositions());
  if (robot_specification.has_start_configuration()) {
    start_configuration = RepeatedDoubleToVectorXd(
        robot_specification.start_configuration().joints());
  }
  return RobotSpecification(robot, default_cart_limits, application_limits,
                            start_configuration);
}
}  // namespace intrinsic
