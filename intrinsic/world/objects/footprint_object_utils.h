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

#ifndef INTRINSIC_WORLD_OBJECTS_FOOTPRINT_OBJECT_UTILS_H_
#define INTRINSIC_WORLD_OBJECTS_FOOTPRINT_OBJECT_UTILS_H_

#include "absl/functional/any_invocable.h"
#include "absl/status/statusor.h"
#include "intrinsic/skills/proto/footprint.pb.h"
#include "intrinsic/world/objects/object_world.h"

namespace intrinsic {
namespace object_world {

// Replaces all object-based world resources in the given fooprint with
// corresponding entity-based resources. Returns the input as is if it already
// contains entity-based resources and returns an error if the input contains
// both types of world resources.
// The given 'object_world_provider' function will only be invoked if required
// to convert object-based world resources to entity-based resources, and it
// will never be invoked more than once.
absl::StatusOr<intrinsic_proto::skills::Footprint>
ConvertObjectFootprintToEntityFootprint(
    const intrinsic_proto::skills::Footprint& footprint,
    absl::AnyInvocable<absl::StatusOr<const ObjectWorld*>()>&
        object_world_provider);

// Adds missing permissions for objects based on how they are attached in the
// world.
absl::StatusOr<intrinsic_proto::skills::Footprint> AddMissingPermissions(
    const intrinsic_proto::skills::Footprint& footprint,
    absl::AnyInvocable<absl::StatusOr<const ObjectWorld*>()>&
        object_world_provider);

}  // namespace object_world

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_OBJECTS_FOOTPRINT_OBJECT_UTILS_H_
