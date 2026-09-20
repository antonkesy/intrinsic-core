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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_LINEAR_CARTESIAN_MOTION_PATH_PLANNER_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_LINEAR_CARTESIAN_MOTION_PATH_PLANNER_H_

#include <memory>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/motion_planning/path_planning/data_structures/path_planner_graph.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/path_planning/path_planner.h"
#include "intrinsic/motion_planning/path_planning/path_planner_definitions.h"
#include "intrinsic/motion_planning/path_planning/planners/linear_cartesian_motion_path_planner_configs.pb.h"

namespace intrinsic {

// A PathPlanner that plans Cartesian linear motions with blending over a
// series of Cartesian waypoints. To implement the current PathPlanner
// interface, the waypoints are provided as joint configurations.
//
// The initial and final joint configurations of the input path will serve as
// constraints for the plan. The planner will attempt to connect the two
// configurations so that the robot target frame follows a linear Cartesian path
// in the composite manifold SO3xR3. If additional, intermediate joint
// configurations are provided in the path, the planner will use them to
// generate intermediate Cartesian waypoints. Then a sequence of Cartesian
// linear motions with blending around the waypoints is created. Blending can be
// configured separately for translations and rotations. The planner uses
// FinePathIk along the sampled Cartesian path. An error is returned if the
// planner is not able to fulfill the constraint on the final joint
// configuration.
class LinearCartesianMotionPathPlanner : public PathPlanner {
 public:
  using LinearCartesianMotionPathPlannerConfig =
      intrinsic_proto::motion_planning::LinearCartesianMotionPathPlannerConfig;

  static absl::StatusOr<std::unique_ptr<PathPlanner>> Create(
      const std::optional<google::protobuf::Any>& any_config);

  explicit LinearCartesianMotionPathPlanner(
      const LinearCartesianMotionPathPlannerConfig& config);

  absl::StatusOr<std::vector<PointPath>> Plan(
      const PointPath& path, const KinematicsSystemProxy& proxy,
      PathPlannerGraph* graph) const override;

  absl::StatusOr<std::vector<PointPath>> Plan(
      const PointPath& path,
      const icon::ManipulatorKinematics* manipulator_kinematics,
      PathPlannerGraph* graph) const;

  // This path planner does no precomputation so this function is just a no-op.
  absl::Status Precompute(const std::vector<PointPath>& paths_to_plan,
                          const KinematicsSystemProxy& proxy,
                          PathPlannerGraph* graph) const override {
    return absl::OkStatus();
  };

 private:
  LinearCartesianMotionPathPlannerConfig config_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_LINEAR_CARTESIAN_MOTION_PATH_PLANNER_H_
