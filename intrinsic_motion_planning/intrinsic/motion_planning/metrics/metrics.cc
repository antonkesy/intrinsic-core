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

#include "intrinsic/motion_planning/metrics/metrics.h"

#include "intrinsic/production/external/googleinit/googleinit.h"
#include "opencensus/stats/stats.h"
#include "opencensus/tags/tag_key.h"

namespace intrinsic {
namespace motion_planning {

using ::opencensus::stats::Aggregation;
using ::opencensus::stats::MeasureDouble;
using ::opencensus::stats::MeasureInt64;
using ::opencensus::stats::ViewDescriptor;
using ::opencensus::tags::TagKey;

constexpr char kMPSPlanTrajectoryTimeSumName[] =
    "intrinsic/motion_planning/mps_plan_trajectory_time_sum";
constexpr char kMPSPlanTrajectoryTimeSumDescription[] =
    "Sum of time taken to plan a trajectory";
constexpr char kMPSPlanTrajectoryTimeNameDist[] =
    "intrinsic/motion_planning/mps_plan_trajectory_time_dist";
constexpr char kMPSPlanTrajectoryTimeDistDescription[] =
    "Distribution of time taken to plan a trajectory";

constexpr char kMilliseconds[] = "ms";

MeasureDouble MPSPlanTrajectoryTimeSum() {
  static const auto measure = MeasureDouble::Register(
      kMPSPlanTrajectoryTimeSumName, kMPSPlanTrajectoryTimeSumDescription,
      kMilliseconds);
  return measure;
}

MeasureDouble MPSPlanTrajectoryTimeDist() {
  static const auto measure = MeasureDouble::Register(
      kMPSPlanTrajectoryTimeNameDist, kMPSPlanTrajectoryTimeDistDescription,
      kMilliseconds);
  return measure;
}

TagKey CacheHitResultKey() {
  static const auto key = TagKey::Register("cache_hit_result");
  return key;
}

TagKey CallerIDKey() {
  static const auto key = TagKey::Register("caller_id");
  return key;
}

REGISTER_MODULE_INITIALIZER(motion_planning_metrics, {
  // Call each measure here once to initialize it.
  MPSPlanTrajectoryTimeSum();
  MPSPlanTrajectoryTimeDist();

  ViewDescriptor()
      .set_name(kMPSPlanTrajectoryTimeSumName)
      .set_measure(kMPSPlanTrajectoryTimeSumName)
      .set_description(kMPSPlanTrajectoryTimeSumDescription)
      .add_column(CacheHitResultKey())
      .add_column(CallerIDKey())
      .set_aggregation(Aggregation::Sum())
      .RegisterForExport();
  ViewDescriptor()
      .set_name(kMPSPlanTrajectoryTimeNameDist)
      .set_measure(kMPSPlanTrajectoryTimeNameDist)
      .set_description(kMPSPlanTrajectoryTimeDistDescription)
      .add_column(CacheHitResultKey())
      .add_column(CallerIDKey())
      // Exponential spacing between 1 and 1024 milliseconds
      // [1, 2, 4, ..., 1024] milliseconds
      .set_aggregation(Aggregation::Distribution(
          opencensus::stats::BucketBoundaries::Exponential(10, 1, 2)))
      .RegisterForExport();
});

}  // namespace motion_planning
}  // namespace intrinsic
