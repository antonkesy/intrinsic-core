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

#ifndef INTRINSIC_WORLD_COAL_COLLISION_CHECKER_H_
#define INTRINSIC_WORLD_COAL_COLLISION_CHECKER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "coal/broadphase/broadphase_collision_manager.h"
#include "coal/collision_object.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/api/affine_transform_of_geometry.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/world/collision/collision_checker.h"
#include "intrinsic/world/collision/collision_checker_utils.h"
#include "intrinsic/world/collision/rule_matching.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/geometry_types.h"
#include "intrinsic/world/proto/collision_checker_config.pb.h"

namespace intrinsic {

struct CoalCollider;

class CoalCollisionChecker : public CollisionChecker {
 public:
  static absl::StatusOr<std::unique_ptr<CoalCollisionChecker>> Create(
      const CollisionCheckerWorld* world,
      const WorldHashSet<PhysicalEntityId>& dynamic_object_ids,
      const intrinsic_proto::RuleSet& rule_set,
      const intrinsic_proto::world::CoalCollisionCheckerConfig& config);

  // We disallow copies of CoalCollisionChecker because this class "points to
  // itself" in the EntityColliders. A default copy constructor would create a
  // broken shallow copy, and we don't yet have need for a full deep copy, so we
  // disable copying for now.
  CoalCollisionChecker(const CoalCollisionChecker&) = delete;
  ~CoalCollisionChecker() override;

  MarginPairConflictStatus IsInCollision(
      CollisionCheckingDebug* collision_debug) const override;

  const std::multiset<intrinsic_proto::Rule>& GetRuleSet() const override {
    return checker_data_.rule_set;
  }

  void SetDistanceCheckStatistics(
      DistanceCheckStatistics* distance_stats) override {
    distance_stats_ = distance_stats;
  }

  struct GeoCacheStats {
    int hits = 0;
    int misses = 0;
    int64_t size_bytes = 0;
  };

  // Empties the global geometry cache for testing. Note that cumulative
  // telemetry counters (hits and misses) are intentionally preserved across
  // clears to reflect production monitoring semantics; tests should inspect
  // metric deltas via GetGlobalGeoCacheStatsForTesting().
  static void ClearGlobalGeoCacheForTesting();
  static std::string GetGlobalGeoCacheDebugStringForTesting();
  static GeoCacheStats GetGlobalGeoCacheStatsForTesting();

  struct ColliderUserData {
    PhysicalEntityId entity_id;
  };

 private:
  explicit CoalCollisionChecker(const CollisionCheckerWorld* world);

  static absl::StatusOr<std::unique_ptr<CoalCollisionChecker>> CreateImpl(
      const CollisionCheckerWorld* world,
      const WorldHashSet<PhysicalEntityId>& dynamic_object_ids,
      const intrinsic_proto::RuleSet& rule_set,
      const intrinsic_proto::world::CoalCollisionCheckerConfig& config);

  const CollisionCheckerWorld& GetCollisionCheckerWorld() const override {
    return world_;
  }

  const CollisionCheckerWorld& world_;
  intrinsic_proto::world::CoalCollisionCheckerConfig config_;
  CollisionCheckerCommonData checker_data_;
  WorldHashMap<PhysicalEntityIdPair, intrinsic_proto::world::CollisionAction>
      dynamic_dynamic_actions_;
  WorldHashMap<PhysicalEntityIdPair, intrinsic_proto::world::CollisionAction>
      dynamic_static_actions_;

  std::vector<CoalCollider> static_coal_colliders_;
  mutable std::vector<CoalCollider> dynamic_coal_colliders_;

  std::vector<ColliderUserData> static_collider_user_data_;
  std::vector<ColliderUserData> dynamic_collider_user_data_;

  // This will contain pointers to elements of static_coal_colliders_.
  std::unique_ptr<coal::BroadPhaseCollisionManager> static_broadphase_;

  // We pad the AABBs of all dynamic objects by this amount. This padding
  // depends on the collision margins defined between the dynamic objects.
  double dynamic_padding_ = 0.0;
};

// Returns the pairs of colliding objects between two sets of physical entity
// IDs in the world using Coal. If either set1 or set2 is empty, it treats that
// set as containing all physical entity IDs in the world.
std::vector<std::pair<PhysicalEntityId, PhysicalEntityId>>
GetCollisionsBetweenSets(const CollisionCheckerWorld& world,
                         const WorldHashSet<PhysicalEntityId>& set1,
                         const WorldHashSet<PhysicalEntityId>& set2,
                         bool check_upper_triangle_only);

// Checks whether two given physical objects are in collision using Coal.
bool AreObjectsInCollision(const CollisionCheckerWorld& world,
                           PhysicalEntityId left_id, PhysicalEntityId right_id);

// Checks whether two TransformedGeometries are in collision using Coal.
bool AreGeometriesInCollision(const TransformedGeometry& left,
                              const TransformedGeometry& right);

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_COAL_COLLISION_CHECKER_H_
