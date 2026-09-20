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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_PIPELINE_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_PIPELINE_H_

#include <memory>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/motion_planning/path_planning/path_planner.h"
#include "intrinsic/motion_planning/path_planning/planners/pipeline_configs.pb.h"

namespace intrinsic {

proto::PipelinePathPlannerConfig GetDefaultPipelineConfig();

// This function is used to instantiate a list of registered PathPlanners from a
// generic specification. The result of this function can be passed to the
// PipelinePathPlanner to execute the PathPlanners sequentially.
//
// This function returns an error if any planner fails to instantiate, or the
// implementation cannot find a registered planner to instantiate from the
// configuration.
absl::StatusOr<std::vector<std::unique_ptr<PathPlanner>>>
CreatePlanningPipeline(const proto::PipelinePathPlannerConfig& config);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PLANNERS_PIPELINE_H_
