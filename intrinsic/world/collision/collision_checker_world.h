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

#ifndef INTRINSIC_WORLD_COLLISION_CHECKER_WORLD_H_
#define INTRINSIC_WORLD_COLLISION_CHECKER_WORLD_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/world/component/geometry_component.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/world.pb.h"

namespace intrinsic {

class CollisionCheckCache;

// Representation of a World that is suitable for use with the CollisionChecker
// implementations
class CollisionCheckerWorld {
 public:
  virtual ~CollisionCheckerWorld() = default;
  // Returns all of the entity ids in this World.
  virtual WorldHashSet<EntityId> GetEntityIds() const = 0;

  // Get a pointer to the entity based on the given id.
  virtual absl::StatusOr<const WorldEntity*> GetEntityById(
      EntityId id) const = 0;

  // Returns a transform between the two given entities. These entity may not be
  // directly connected but still have an indirect connection. This method
  // inspects these connections to calculate a transform that describes the pose
  // between the two given entities. Returns the transform in the form
  // of a_t_b.
  virtual Pose3d GetTransform(AttachmentEntityId a_handle,
                              AttachmentEntityId b_handle) const = 0;

  // Return the list of pairwise object ids that are not collision checked
  // against each other.
  virtual std::vector<std::pair<PhysicalEntityId, PhysicalEntityId>>
  GetExclusionPairs(bool filter_empty_collision_geometry) const = 0;

  // Return the geometry named `kind` associated with an entity of the given
  // id. Returns an error if the id or the geometry is not found.
  virtual absl::StatusOr<NamedGeometrySet> GetGeometryForEntity(
      EntityId id, absl::string_view kind) const = 0;

  // Create a single static spatial tree representative of the input static
  // physical objects in world space.
  virtual NamedGeometrySet GetStaticSpatialTreeInWorldSpace(
      const WorldHashSet<PhysicalEntityId>& static_object_ids) const = 0;

  // Given object_id find its memoized SpatialTree representation in model space
  // and return its key.
  virtual NamedGeometrySet GetSpatialTreeInModelSpace(
      PhysicalEntityId object_id) const = 0;

  // Returns the set of local names in the path from root to the given node
  // combined into a single string. The given node will be at the end of the
  // string and the root will not be part of the string as it is implied.
  virtual absl::StatusOr<std::string> GetLocalNamePathString(
      AttachmentEntityId id, absl::string_view separator) const = 0;

  // Tries to get the name of the object to which this entity belongs, or the
  // name of the entity itself if it is the object.
  //
  // See world.h for more info.
  virtual absl::StatusOr<std::string> TryGetObjectNameForEntity(
      EntityId entity_id) const = 0;

  // Returns the default rule set for the world.
  virtual intrinsic_proto::RuleSet GetDefaultRuleSet() const = 0;

  // Returns the cache for collision checks.
  virtual std::shared_ptr<CollisionCheckCache> GetCachedChecks() const = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_COLLISION_CHECKER_WORLD_H_
