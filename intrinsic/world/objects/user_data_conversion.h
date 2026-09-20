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

#ifndef INTRINSIC_WORLD_OBJECTS_USER_DATA_CONVERSION_H_
#define INTRINSIC_WORLD_OBJECTS_USER_DATA_CONVERSION_H_

#include "absl/status/statusor.h"
#include "intrinsic/scene/proto/v1/scene_object_updates.pb.h"
#include "intrinsic/world/proto/user_data.pb.h"

namespace intrinsic {
namespace object_world {

// Returns SceneObject user data policy corresponding to the given
// WorldObject user data policy.
absl::StatusOr<
    intrinsic_proto::scene_object::v1::UpdateUserData::UpdateUserDataPolicy>
ToSceneObjectUserDataPolicy(
    intrinsic_proto::world::UpdateUserData::UpdateUserDataPolicy policy);

// Returns WorldObject user data policy corresponding to the given SceneObject
// user data policy.
absl::StatusOr<intrinsic_proto::world::UpdateUserData::UpdateUserDataPolicy>
FromSceneObjectUserDataPolicy(
    intrinsic_proto::scene_object::v1::UpdateUserData::UpdateUserDataPolicy
        policy);

}  // namespace object_world
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_OBJECTS_USER_DATA_CONVERSION_H_
