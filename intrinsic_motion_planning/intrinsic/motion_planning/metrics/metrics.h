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

#ifndef INTRINSIC_MOTION_PLANNING_METRICS_METRICS_H_
#define INTRINSIC_MOTION_PLANNING_METRICS_METRICS_H_

#include "opencensus/stats/stats.h"
#include "opencensus/tags/tag_key.h"

namespace intrinsic {
namespace motion_planning {

// Measures the time spent in PlanTrajectory.
opencensus::stats::MeasureDouble MPSPlanTrajectoryTimeSum();
opencensus::stats::MeasureDouble MPSPlanTrajectoryTimeDist();

opencensus::tags::TagKey CacheHitResultKey();
opencensus::tags::TagKey CallerIDKey();

}  // namespace motion_planning
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_METRICS_METRICS_H_
