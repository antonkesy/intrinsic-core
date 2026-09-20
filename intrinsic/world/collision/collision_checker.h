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

#ifndef INTRINSIC_WORLD_COLLISION_CHECKER_H_
#define INTRINSIC_WORLD_COLLISION_CHECKER_H_

#include <cstdint>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "gloop/util/gtl/flat_set.h"
#include "intrinsic/geometry/api/distance_stats.h"
#include "intrinsic/motion_planning/proto/v1/motion_planning_error.pb.h"
#include "intrinsic/world/collision/collision_checker_world.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/collision/collision_types.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"

namespace intrinsic {

// Contains debug information related to the collision checking performed for a
// given configuration.
struct CollisionCheckingDebug {
  // Represents a margin violation between some object and a collection of other
  // objects. The right side can be a collection of objects because we merge
  // static geometries together to perform collision checking. At that point we
  // don't know which of the static entities is in collision.
  struct Collision {
    // One of the objects in margin violation.
    PhysicalEntityId left_object;
    // At least one of the entities in right_objects is in margin violation with
    // left_object.
    gtl::flat_set<PhysicalEntityId> right_objects;
    // The desired margin that has been violated. That is, the distance between
    // left_object and one of right_objects is smaller than this value.
    double desired_margin;
    // TODO(b/222300707): Consider adding the computed minimum distance (that is
    // smaller than desired_margin), though this may be tricky to surface.
  };
  // The margin violations that have been discovered. Note that there may be
  // more collisions, since we return early as soon as a collision is
  // discovered.
  std::vector<Collision> collisions;
};

// An interface for checking if collisions are present in the physical world.
class CollisionChecker {
 public:
  virtual ~CollisionChecker() = default;

  // Returns true if the dynamic objects used to create this checker are in
  // collision with any other non excluded objects in the physical world.
  // If non-null, `collision_debug` is filled with collision checking debug
  // info.
  // TODO(b/222300957): Consider adding an option to find all collisions on
  // demand (as opposed to the default behavior of exiting as soon as a
  // collision is found).
  virtual MarginPairConflictStatus IsInCollision(
      CollisionCheckingDebug* collision_debug) const = 0;

  // Returns a string describing `collision_debug` which includes the detected
  // collisions.
  absl::StatusOr<std::string> PrintCollisionCheckingDebugDetailed(
      const CollisionCheckingDebug& collision_debug) const;

  // Returns a tuple with the left entity and vector of right entities for a
  // given collision.
  absl::StatusOr<std::tuple<std::string, std::vector<std::string>>>
  GetCollisionEntitiesMessage(
      const CollisionCheckingDebug::Collision& collision) const;

  // Returns a tuple with the left entity and vector of right entities for a
  // given collision.
  std::tuple<EntityId, std::vector<EntityId>> GetCollisionEntities(
      const CollisionCheckingDebug::Collision& collision) const;

  // Returns a string describing the collision in the format of
  // entity alias name with label.
  // We limit the message to include a maximum of 4 joint configurations.
  absl::StatusOr<std::string> PrintCollisionCheckingDebug(
      const CollisionCheckingDebug& collision_debug) const;

  // Returns the rule set used in this collision checker.
  virtual const std::multiset<intrinsic_proto::Rule>& GetRuleSet() const = 0;

  virtual void SetDistanceCheckStatistics(
      DistanceCheckStatistics* distance_stats) = 0;

 protected:
  mutable DistanceCheckStatistics* distance_stats_ = nullptr;

 private:
  virtual const CollisionCheckerWorld& GetCollisionCheckerWorld() const = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_COLLISION_CHECKER_H_
