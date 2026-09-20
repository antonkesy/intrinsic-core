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

#ifndef INTRINSIC_CHOREOGRAPHER_FABRICATION_TOOLKIT_SHAPE_UTIL_H_
#define INTRINSIC_CHOREOGRAPHER_FABRICATION_TOOLKIT_SHAPE_UTIL_H_

#include "absl/strings/string_view.h"
#include "intrinsic/geometry/api/affine_transform_of_geometry.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace geo {
class GeometrySerializer;
}

namespace toolkit {

// Adds the given volume to the world as a single entity attached to the root,
// returns the new entity id for the volume in the world. If `serializer` is
// provided, the geometry is eagerly converted to a proto representation and
// stored as a `NamedGeometryProtoSet` on the entity's `GeometryComponent`,
// allowing downstream parameterless `World::Serialize()` and `ToProto()` calls.
// If `serializer` is nullptr, raw in-memory geometry is attached.
PhysicalEntityId AddVolumeToWorld(
    World* world, const TransformedGeometry& shape,
    absl::string_view volume_identifier,
    geo::GeometrySerializer* serializer = nullptr);

// Adds the given volume to the world as a single entity attached to `parent`,
// returns the new entity id for the volume in the world. If `serializer` is
// provided, the geometry is eagerly converted to a proto representation and
// stored as a `NamedGeometryProtoSet` on the entity's `GeometryComponent`,
// allowing downstream parameterless `World::Serialize()` and `ToProto()` calls.
// If `serializer` is nullptr, raw in-memory geometry is attached.
PhysicalEntityId AddVolumeToWorld(
    World* world, AttachmentEntityId parent, const TransformedGeometry& shape,
    absl::string_view volume_identifier,
    geo::GeometrySerializer* serializer = nullptr);

}  // namespace toolkit
}  // namespace intrinsic

#endif  // INTRINSIC_CHOREOGRAPHER_FABRICATION_TOOLKIT_SHAPE_UTIL_H_
