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

#ifndef GOOGLE3_INTRINSIC_WORLD_CONVERSION_WORLD_FROM_OBJECT_PROTO_H_
#define GOOGLE3_INTRINSIC_WORLD_CONVERSION_WORLD_FROM_OBJECT_PROTO_H_

#include "absl/status/statusor.h"
#include "intrinsic/world/proto/collision_settings.pb.h"
#include "intrinsic/world/proto/object_world_service.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic::world {

// Converts ListObjectsResponse and CollisionSettings directly to a C++ entity
// World.
absl::StatusOr<World> CreateEntityWorldFromProto(
    const intrinsic_proto::world::ListObjectsResponse& response,
    const intrinsic_proto::world::CollisionSettings& collision_settings = {});

}  // namespace intrinsic::world

#endif  // GOOGLE3_INTRINSIC_WORLD_CONVERSION_WORLD_FROM_OBJECT_PROTO_H_
