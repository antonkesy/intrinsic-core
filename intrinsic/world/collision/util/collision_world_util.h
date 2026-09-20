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

#ifndef INTRINSIC_WORLD_UTIL_COLLISION_WORLD_UTIL_H_
#define INTRINSIC_WORLD_UTIL_COLLISION_WORLD_UTIL_H_

#include <memory>
#include <optional>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/geometry/api/axis_aligned_bounding_box_3d.h"
#include "intrinsic/world/collision/collision_checker.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/proto/collision_checker_config.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

// Returns a collision checker that represents all of the physical objects that
// can be moved by the dof view. Using the default rule set stored in the world
// for collision settings. If `collision_checker_config` is unconfigured or
// omitted (CONFIG_NOT_SET), defaults to `kDefaultCollisionCheckerConfigCase`.
absl::StatusOr<std::shared_ptr<CollisionChecker>> GetCollisionCheckerForDofView(
    World* world, const DofKinematicView& dof_view,
    std::optional<intrinsic_proto::RuleSet> rule_set = std::nullopt,
    const intrinsic_proto::world::CollisionCheckerConfig&
        collision_checker_config = {});

// Calculates some bounds that will encompass all current objects with some
// padding. Most code using this should set `add_padding` to true. Set it to
// false when testing the implementation of bounding box computation.
//
// NOTE: We will return a box with miminum size of 3,3,3 even if the world is
// empty. This is to ensure that users of this are able to do something useful
// with the bbox after it has been computed.
absl::StatusOr<AxisAlignedBoundingBox3d> ComputeWorldBoundingBox(
    const World& world, bool add_padding);

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_UTIL_COLLISION_WORLD_UTIL_H_
