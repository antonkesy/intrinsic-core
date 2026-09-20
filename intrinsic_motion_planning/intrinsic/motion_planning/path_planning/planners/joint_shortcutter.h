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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_JOINT_SHORTCUTTER_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_JOINT_SHORTCUTTER_H_

#include <memory>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/motion_planning/path_planning/data_structures/path_planner_graph.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/path_planning/path_planner.h"
#include "intrinsic/motion_planning/path_planning/path_planner.pb.h"
#include "intrinsic/motion_planning/path_planning/path_planner_definitions.h"
#include "intrinsic/motion_planning/path_planning/planners/shortcutter_configs.pb.h"

namespace intrinsic {

// If 3 configurations are collinear then the middle point can be safely removed
// without any collision checking. This removes all collinear segments from a
// path.
PointPath RemoveCollinearPoints(const PointPath& path);

// Performs a simple version of shortcutting that greedily attempts to skip as
// many points as possible before adding a waypoint.
//
// This path planner has two modes. The first searches linearly while the second
// utilizes binary search.
class JointShortcutter : public PathPlanner {
 public:
  static absl::StatusOr<std::unique_ptr<PathPlanner>> Create(
      const std::optional<google::protobuf::Any>& any_config);

  explicit JointShortcutter(const proto::JointShortcutterConfig& config);

  absl::StatusOr<std::vector<PointPath>> Plan(
      const PointPath& path, const KinematicsSystemProxy& proxy,
      PathPlannerGraph* graph) const override;

  // No-op.
  absl::Status Precompute(const std::vector<PointPath>& paths_to_plan,
                          const KinematicsSystemProxy& proxy,
                          PathPlannerGraph* graph) const override {
    return absl::OkStatus();
  }

 private:
  proto::JointShortcutterConfig config_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_JOINT_SHORTCUTTER_H_
