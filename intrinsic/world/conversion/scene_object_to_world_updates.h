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

#ifndef GOOGLE3_INTRINSIC_WORLD_CONVERSION_SCENE_OBJECT_TO_WORLD_UPDATES_H_
#define GOOGLE3_INTRINSIC_WORLD_CONVERSION_SCENE_OBJECT_TO_WORLD_UPDATES_H_

#include <string>

#include "absl/status/statusor.h"
#include "intrinsic/scene/proto/v1/scene_object_updates.pb.h"
#include "intrinsic/world/proto/object_world_service.pb.h"
#include "intrinsic/world/proto/object_world_updates.pb.h"

namespace intrinsic::world {

// Converts SceneObjectInstanceUpdates into ObjectWorldUpdates.
//
// All updates are assumed to apply to the object identified by target_object
// or to its sub-entities (e.g. frames). The passed world_id will be set for all
// of the generated updates. If world_id is left empty, it will be empty (but
// set) in the resulting updates.
//
// References to objects will always use the object name.
absl::StatusOr<intrinsic_proto::world::ObjectWorldUpdates>
ConvertSceneObjectUpdatesToWorldUpdates(
    absl::string_view world_id,
    const intrinsic_proto::world::Object& target_object,
    const intrinsic_proto::scene_object::v1::SceneObjectInstanceUpdates&
        scene_updates);

}  // namespace intrinsic::world

#endif  // GOOGLE3_INTRINSIC_WORLD_CONVERSION_SCENE_OBJECT_TO_WORLD_UPDATES_H_
