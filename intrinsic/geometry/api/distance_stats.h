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

#ifndef INTRINSIC_GEOMETRY_API_DISTANCE_STATS_H_
#define INTRINSIC_GEOMETRY_API_DISTANCE_STATS_H_

#include <algorithm>
#include <cstddef>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/time/time.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"

namespace intrinsic::geo {
// This struct and functions provide some common functionality used to track
// statistics about collision.
struct CollisionStats {
  absl::Duration time_clear;
  int checks_clear = 0;
  int checks_invalid = 0;
  absl::Duration time_invalid;
  int cache_hits = 0;
  int cache_misses = 0;
};

struct SlowCheckStats {
  struct Data {
    absl::Duration duration;
    Pose3d transform;
    double margin;
    bool result;
  };

  Data in_collision;
  Data not_in_collision;
};

struct DistanceCheckStatistics {
  // The total number of checks performed.
  size_t num_checks = 0;

  // The timing of the collision checks when comparing two entities.
  WorldHashMap<std::pair<EntityId, EntityId>, CollisionStats> time_by_pair;

  // The number of fallback mesh/point cloud checks that were performed after an
  // unsupported primitive check was attempted.
  size_t fallback_counter = 0;

  // The total time taken to perform the fallback checks.
  absl::Duration fallback_time_total = absl::ZeroDuration();

  WorldHashMap<std::pair<EntityId, EntityId>, SlowCheckStats> slow_checks;
};

}  // namespace intrinsic::geo

namespace intrinsic {
using ::intrinsic::geo::CollisionStats;
using ::intrinsic::geo::DistanceCheckStatistics;
using ::intrinsic::geo::SlowCheckStats;
}  // namespace intrinsic

#endif  // INTRINSIC_GEOMETRY_API_DISTANCE_STATS_H_
