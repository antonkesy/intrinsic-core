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

#ifndef INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_MOTION_PLANNER_H_
#define INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_MOTION_PLANNER_H_

#include <memory>
#include <optional>

#include "absl/status/statusor.h"
#include "intrinsic/motion_planning/motion_planner/motion_planner_flags.h"
#include "intrinsic/motion_planning/motion_planner/motion_planner_interface.h"
#include "intrinsic/motion_planning/motion_planner/trajectory_parameterizer.h"
#include "intrinsic/motion_planning/proto/v1/motion_planner_config.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_specification.pb.h"
#include "intrinsic/motion_planning/proto/v1/robot_specification.pb.h"
#include "intrinsic/world/objects/object_world.h"

namespace intrinsic {

// A Motion Planner that samples greedily
class MotionPlanner : public MotionPlannerInterface {
 public:
  // Legacy factory method for backward compatibility.
  // In internal builds, this will instantiate an enterprise motion planner.
  // In OSS builds, it will instantiate an OSS motion planner.
  static absl::StatusOr<std::unique_ptr<MotionPlanner>> Create(
      const MotionPlannerFlags& flags);

  absl::StatusOr<PlanPathResult> PlanPath(
      const object_world::ObjectWorld& world,
      const intrinsic_proto::motion_planning::v1::RobotSpecification&
          robot_specification,
      const intrinsic_proto::motion_planning::v1::MotionSpecification&
          motion_specification,
      const intrinsic_proto::motion_planning::v1::MotionPlannerConfiguration&
          motion_config,
      std::optional<RunTimeMotionPlannerFlags> run_time_flags) const override;

  absl::StatusOr<PlanTrajectoryResult> PlanTrajectory(
      const object_world::ObjectWorld& world,
      const intrinsic_proto::motion_planning::v1::RobotSpecification&
          robot_specification,
      const intrinsic_proto::motion_planning::v1::MotionSpecification&
          motion_specification,
      const intrinsic_proto::motion_planning::v1::MotionPlannerConfiguration&
          motion_config,
      std::optional<RunTimeMotionPlannerFlags> run_time_flags) const override;

 private:
  MotionPlanner(
      const MotionPlannerFlags& flags,
      std::unique_ptr<TrajectoryParameterizer> trajectory_parameterizer);

  const MotionPlannerFlags flags_;
  std::unique_ptr<TrajectoryParameterizer> trajectory_parameterizer_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_MOTION_PLANNER_MOTION_PLANNER_H_
