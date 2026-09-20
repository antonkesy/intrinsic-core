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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_RRT_CONNECT_PATH_PLANNER_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_RRT_CONNECT_PATH_PLANNER_H_

#include <algorithm>
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
#include "intrinsic/motion_planning/path_planning/planners/rrt_configs.pb.h"

namespace intrinsic {

// A PathPlanner that executes RRT on the input path. This version uses RRT
// connect and a special heuristic to select the next point to expand the
// current tree to.
//
// This planner accepts any path with at least 2 points. The first and last
// points must be valid as determined by the proxy. To handle paths of arbitrary
// length, the planner breaks the initial path on invalid points and edges, and
// performs rrt on these invalid "gaps". Note that in the case of exactly two
// points this reduces exactly to planning between the start and end (assuming
// they themselves are valid).
class RrtConnectPathPlanner : public PathPlanner {
 public:
  static absl::StatusOr<std::unique_ptr<PathPlanner>> Create(
      const std::optional<google::protobuf::Any>& any_config);

  explicit RrtConnectPathPlanner(const proto::RrtPathPlannerConfig& config);

  absl::StatusOr<std::vector<PointPath>> Plan(
      const PointPath& path, const KinematicsSystemProxy& proxy,
      PathPlannerGraph* graph) const override;

  // RRT does no precomputation so this function is just a no-op.
  absl::Status Precompute(const std::vector<PointPath>& paths_to_plan,
                          const KinematicsSystemProxy& proxy,
                          PathPlannerGraph* graph) const override;

 private:
  proto::RrtPathPlannerConfig config_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_RRT_CONNECT_PATH_PLANNER_H_
