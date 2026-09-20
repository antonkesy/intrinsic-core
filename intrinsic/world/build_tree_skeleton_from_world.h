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

#ifndef INTRINSIC_WORLD_BUILD_TREE_SKELETON_FROM_WORLD_H_
#define INTRINSIC_WORLD_BUILD_TREE_SKELETON_FROM_WORLD_H_

#include <memory>
#include <string>

#include "absl/status/statusor.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

// Returns a unique, human-readable name for `id` in `world`.
//
// Considers only entities below `root` for the uniqueness check.
//
// This name may include the raw EntityId value to disambiguate, in case the
// entity's LocalName is not unique in `world`.
std::string GetUniqueName(const World& world, AttachmentEntityId root,
                          EntityId id);

// Extracts the kinematic tree rooted at `root` from `world`, and builds a
// kinematics::Skeleton that represents that tree.
absl::StatusOr<std::unique_ptr<kinematics::Skeleton>>
BuildTreeSkeletonFromWorld(const World& world, AttachmentEntityId root);

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_BUILD_TREE_SKELETON_FROM_WORLD_H_
