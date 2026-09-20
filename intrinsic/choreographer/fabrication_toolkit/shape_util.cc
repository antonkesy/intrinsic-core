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

#include "intrinsic/choreographer/fabrication_toolkit/shape_util.h"

#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/geometry/api/affine_transform_of_geometry.h"
#include "intrinsic/geometry/api/io.h"
#include "intrinsic/geometry/storage/geometry_serializer.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/macros.h"
#include "intrinsic/world/component/collision_component.h"
#include "intrinsic/world/component/geometry_component.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/geometry_types.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace toolkit {

PhysicalEntityId AddVolumeToWorld(World* world,
                                  const TransformedGeometry& shape,
                                  absl::string_view volume_identifier,
                                  GeometrySerializer* serializer) {
  return AddVolumeToWorld(world, kRootEntityId, shape, volume_identifier,
                          serializer);
}

PhysicalEntityId AddVolumeToWorld(World* world, AttachmentEntityId parent,
                                  const TransformedGeometry& shape,
                                  absl::string_view volume_identifier,
                                  GeometrySerializer* serializer) {
  const EntityId entity_id = world->CreateEntity();

  ASSIGN_OR_DIE(WorldEntity* const entity, world->GetEntityById(entity_id));
  CHECK_OK(entity->SetLocalName(absl::StrCat("volume_", volume_identifier)));

  ASSIGN_OR_DIE(GeometryComponent* const geometry_component,
                entity->GetOrCreateComponent<GeometryComponent>());

  // If a serializer is provided, eagerly convert the `TransformedGeometry` into
  // a `NamedGeometryProtoSet` so that subsequent `World::Serialize()` /
  // `ToProto()` calls on the world or entity can succeed without requiring a
  // serializer.
  if (serializer != nullptr) {
    ASSIGN_OR_DIE(
        const intrinsic_proto::geometry::v1::TransformedGeometry shape_proto,
        intrinsic::ToProto(shape, serializer));
    const NamedGeometryProtoSet proto_set = {{"0", shape_proto}};
    geometry_component->SetGeometry(kKindCollisionGeometry, proto_set);
    geometry_component->SetGeometry(kKindVisualGeometry, proto_set);
  } else {
    const NamedGeometrySet raw_set = {{"0", shape}};
    geometry_component->SetGeometry(kKindCollisionGeometry, raw_set);
    geometry_component->SetGeometry(kKindVisualGeometry, raw_set);
  }

  CHECK_OK(entity->CreateComponent<CollisionComponent>());
  CHECK_OK(world->CreateAttachmentComponent(parent, entity_id, Pose3d()));

  ASSIGN_OR_DIE(const PhysicalEntityId id,
                world->ValidateEntity<PhysicalEntityId>(entity_id));
  return id;
}

}  // namespace toolkit
}  // namespace intrinsic
