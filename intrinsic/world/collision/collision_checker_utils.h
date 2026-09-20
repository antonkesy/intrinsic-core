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

#ifndef INTRINSIC_WORLD_COLLISION_CHECKER_UTILS_H_
#define INTRINSIC_WORLD_COLLISION_CHECKER_UTILS_H_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/flags/declare.h"
#include "absl/hash/hash.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/api/distance_stats.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/aggregate_type.h"
#include "intrinsic/util/proto/pb_hash.h"
#include "intrinsic/world/collision/collision_checker_world.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/collision/collision_types.h"
#include "intrinsic/world/collision/rule_matching.h"
#include "intrinsic/world/component/geometry_component.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"

ABSL_DECLARE_FLAG(bool, use_global_collision_checking_stats);

namespace intrinsic {

struct CollisionCheckerCommonData {
  // List of dynamic object ids specific to the context of the collision
  // checker.
  WorldHashSet<PhysicalEntityId> dynamic_object_ids;

  // List of static objects in this Checker. All objects = static + dynamic.
  WorldHashSet<PhysicalEntityId> static_object_ids;

  // The rules, ordered by their priority.
  std::multiset<intrinsic_proto::Rule> rule_set;
};

// Fills the rule set with the provided rules and the world exclusion pairs. The
// returned rules are ordered by their priority.
std::multiset<intrinsic_proto::Rule> FillRuleSet(
    const CollisionCheckerWorld& world,
    const intrinsic_proto::RuleSet& rule_set);

// Populate the CollisionCheckerCommonData that is required to evaluate
// collisions.
CollisionCheckerCommonData GetCollisionCheckerData(
    const WorldHashSet<PhysicalEntityId>& dynamic_object_ids,
    const CollisionCheckerWorld& world,
    const intrinsic_proto::RuleSet& rule_set);

// Extract the actions for pair of two given sets.
WorldHashMap<PhysicalEntityIdPair, intrinsic_proto::world::CollisionAction>
GetActionMap(const std::multiset<intrinsic_proto::Rule>& rules,
             const WorldHashSet<PhysicalEntityId>& set1,
             const WorldHashSet<PhysicalEntityId>& set2);

// Returns the local name of the given entity
std::string GetLocalNameForEntityById(const CollisionCheckerWorld& world,
                                      EntityId id);

// Returns the local name path of the given entity
std::string GetLocalNamePathForEntityById(const CollisionCheckerWorld& world,
                                          EntityId id);

// The objects/functions in this namespace are used to track the performance of
// collision checking, and display the results.
namespace collision_details {

// Update stats with information from a new collision check.
void UpdateStats(const MarginPairConflictStatus& status, absl::Duration time,
                 bool cache_hit, CollisionStats* stats);
void AggregateStats(const CollisionStats& s, CollisionStats* stats);
std::string ToString(const CollisionStats& cs);

// Helper that orders the ids so the pair can be used to track statistics.
std::pair<EntityId, EntityId> MakeOrderedPair(const EntityId& a,
                                              const EntityId& b);

// Common utility that sorts a map of <T, CollisionStats> by total time for the
// pair so that it's easier to find the expensive ones.
template <typename T>
std::vector<std::pair<T, CollisionStats>> SortMap(
    const WorldHashMap<T, CollisionStats>& m) {
  std::vector<std::pair<T, CollisionStats>> out;
  out.reserve(m.size());
  for (const auto& p : m) {
    out.emplace_back(p);
  }
  std::sort(out.begin(), out.end(),
            [](const std::pair<T, CollisionStats>& a,
               const std::pair<T, CollisionStats>& b) {
              return a.second.time_clear + a.second.time_invalid <
                     b.second.time_clear + b.second.time_invalid;
            });
  return out;
}

void LogCollisionCheckingDebugStats(
    const CollisionCheckerWorld& world,
    const DistanceCheckStatistics& distance_stats);

bool EntityHasPointCloud(const CollisionCheckerWorld& world,
                         const PhysicalEntityId entity_id);

}  // namespace collision_details

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_COLLISION_CHECKER_UTILS_H_
