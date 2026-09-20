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

#ifndef INTRINSIC_WORLD_KINEMATICS_BUILDER_H_
#define INTRINSIC_WORLD_KINEMATICS_BUILDER_H_

#include <memory>
#include <string>

#include "absl/status/statusor.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/world/aspects/entity_world_interface.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"

namespace intrinsic {

// Builds a skeleton for the kinematic chain between id1 and id2.
//
// Joints in the chain are ordered to match their order in the robots in the
// `world`, so the root of the chain may be either `id1` or `id2`. When `id1` is
// an ancestor of `id2`, the root of the chain will be `id1`.
absl::StatusOr<std::unique_ptr<kinematics::Skeleton>> BuildChainSkeleton(
    const entity_aspect_world_details::EntityWorld& world,
    AttachmentEntityId id1, AttachmentEntityId id2);

std::string CreateKinematicElementName(EntityId id, const WorldEntity& entity);

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_KINEMATICS_BUILDER_H_
