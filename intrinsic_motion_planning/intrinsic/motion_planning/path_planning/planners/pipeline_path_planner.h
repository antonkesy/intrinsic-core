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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_PIPELINE_PATH_PLANNER_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_PIPELINE_PATH_PLANNER_H_

#include <memory>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "intrinsic/motion_planning/path_planning/data_structures/path_planner_graph.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/path_planning/path_planner.h"
#include "intrinsic/motion_planning/path_planning/path_planner.pb.h"
#include "intrinsic/motion_planning/path_planning/path_planner_definitions.h"
#include "intrinsic/motion_planning/path_planning/planners/pipeline_configs.pb.h"

namespace intrinsic {

// This class is an implementation of a PathPlanner that itself calls a pipeline
// of PathPlanners sequentially. The PathPlanners are constructed from the
// specifications in the PipelinePathPlannerConfig passed.
class PipelinePathPlanner : public PathPlanner {
 public:
  static absl::StatusOr<std::unique_ptr<PipelinePathPlanner>> Create(
      const proto::PipelinePathPlannerConfig& config);

  static absl::StatusOr<std::unique_ptr<PathPlanner>> Create(
      const std::optional<google::protobuf::Any>& any_config);

  // Note that the construction of the subplanners is handled by pipeline.h. The
  // config is passed here only for the other options.
  PipelinePathPlanner(const proto::PipelinePathPlannerConfig& config,
                      std::vector<std::unique_ptr<PathPlanner>>&& planners);

  // Calls Plan on the instantiated PathPlanners.
  //
  // Returns an error if any of the calls fail.
  absl::StatusOr<std::vector<PointPath>> Plan(
      const PointPath& path, const KinematicsSystemProxy& proxy,
      PathPlannerGraph* graph) const override;

  // Calls Precompute on the instantiated PathPlanners.
  //
  // Returns an error if any of the calls fail.
  absl::Status Precompute(const std::vector<PointPath>& paths_to_plan,
                          const KinematicsSystemProxy& proxy,
                          PathPlannerGraph* graph) const override;

 private:
  proto::PipelinePathPlannerConfig config_;
  // Subplanners.
  std::vector<std::unique_ptr<PathPlanner>> planners_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_PIPELINE_PATH_PLANNER_H_
