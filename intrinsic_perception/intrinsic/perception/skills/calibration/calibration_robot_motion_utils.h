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

#ifndef INTRINSIC_PERCEPTION_SKILLS_CALIBRATION_CALIBRATION_ROBOT_MOTION_UTILS_H_
#define INTRINSIC_PERCEPTION_SKILLS_CALIBRATION_CALIBRATION_ROBOT_MOTION_UTILS_H_

#include <memory>

#include "absl/status/status.h"
#include "intrinsic/icon/skills/util/position_part_util.h"
#include "intrinsic/motion_planning/motion_planner_client.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/perception/skills/calibration/collect_calibration_data.pb.h"
#include "intrinsic/skills/cc/execute_context.h"
#include "intrinsic/util/grpc/channel_interface.h"
#include "intrinsic/world/objects/kinematic_object.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"

namespace intrinsic {
namespace skills {

struct CalibrationRobotMotionConfig {
  world::KinematicObject robot;
  bool disable_collision_checking;
  intrinsic_proto::skills::MotionType motion_type;
  ArmPartInformation arm_part_info;
  std::shared_ptr<ChannelInterface> icon_channel;
  motion_planning::MotionPlannerClient& motion_planner;
};

absl::Status UpdateRobotJointPositions(
    const CalibrationRobotMotionConfig& config,
    world::ObjectWorldClient& world);

absl::Status PlanAndExecuteMotion(
    const intrinsic_proto::motion_planning::v1::GeometricConstraint& waypoint,
    const intrinsic_proto::world::TransformNodeReference& default_tool,
    const intrinsic_proto::world::TransformNodeReference& default_frame,
    const CalibrationRobotMotionConfig& config, ExecuteContext& context);

absl::Status MoveToWaypoint(
    const intrinsic_proto::motion_planning::v1::GeometricConstraint& waypoint,
    const intrinsic_proto::world::TransformNodeReference& default_tool,
    const intrinsic_proto::world::TransformNodeReference& default_frame,
    const CalibrationRobotMotionConfig& config, ExecuteContext& context);

}  // namespace skills
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_SKILLS_CALIBRATION_CALIBRATION_ROBOT_MOTION_UTILS_H_
