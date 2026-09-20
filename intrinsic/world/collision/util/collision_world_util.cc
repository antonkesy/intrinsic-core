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

#include "intrinsic/world/collision/util/collision_world_util.h"

#include <iterator>
#include <memory>
#include <optional>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/api/affine_transform_of_geometry.h"
#include "intrinsic/geometry/api/axis_aligned_bounding_box_3d.h"
#include "intrinsic/geometry/api/compute_axis_aligned_bounding_box_3d.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/collision/collision_checker.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/component/geometry_component.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/geometry_types.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/collision_checker_config.pb.h"
#include "intrinsic/world/util/walk_attachment_tree.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

// This goes through all of the joints in the dof view and gets their full sub
// trees, then takes those full sets and combines them to generate the
// CollisionChecker.
absl::StatusOr<std::shared_ptr<CollisionChecker>> GetCollisionCheckerForDofView(
    World* world, const DofKinematicView& dof_view,
    std::optional<intrinsic_proto::RuleSet> rule_set,
    const intrinsic_proto::world::CollisionCheckerConfig&
        collision_checker_config) {
  const std::vector<JointEntityId>& joint_ids = dof_view.GetJointEntityIds();
  WorldHashSet<AttachmentEntityId> joint_ids_set{joint_ids.begin(),
                                                 joint_ids.end()};

  // Collect all of the sub trees.
  INTR_ASSIGN_OR_RETURN(WorldHashSet<AttachmentEntityId> all_ids,
                        GetAllAttachments(*world, joint_ids_set));
  WorldHashSet<PhysicalEntityId> all_phys_ids;
  for (const auto& id : all_ids) {
    all_phys_ids.emplace(id.value());
  }

  intrinsic_proto::RuleSet updated_rule_set;
  if (rule_set.has_value()) {
    updated_rule_set = rule_set.value();
  }

  INTR_ASSIGN_OR_RETURN(auto link_sets,
                        GetRigidlyAttachedEntitiesMap(*world, joint_ids_set));
  for (const auto& [_, child_link_set] : link_sets) {
    if (child_link_set.size() <= 1) {
      continue;
    }

    for (auto lhs_itr = child_link_set.begin(); lhs_itr != child_link_set.end();
         ++lhs_itr) {
      for (auto rhs_itr = std::next(lhs_itr); rhs_itr != child_link_set.end();
           ++rhs_itr) {
        auto* rule = updated_rule_set.add_rules();
        rule->add_id_1(lhs_itr->value());
        rule->add_id_2(rhs_itr->value());
        rule->mutable_action()->set_is_excluded(true);
      }
    }
  }

  return world->GetCollisionChecker(all_phys_ids, updated_rule_set,
                                    collision_checker_config);
}

absl::StatusOr<AxisAlignedBoundingBox3d> ComputeWorldBoundingBox(
    const World& world, bool add_padding) {
  AxisAlignedBoundingBox3d bounds;
  // Go through all of the objects in the world.
  for (const auto& entity_id :
       world.GetTypedEntityIds<AttachmentComponentType,
                               GeometryComponentType>()) {
    INTR_ASSIGN_OR_RETURN(const auto entity, world.GetEntityById(entity_id));
    INTR_ASSIGN_OR_RETURN(const auto geometry_component,
                          entity->GetComponent<GeometryComponent>());
    auto geo_set_or = geometry_component->GetGeometry(kKindCollisionGeometry);
    if (!geo_set_or.ok() || geo_set_or->empty()) {
      VLOG(1) << "entity id=" << entity_id
              << ", name=" << entity->GetLocalName() << " has no collision geo";
      continue;
    }

    const auto root_t_object = world.GetTransform(kRootEntityId, entity_id);

    // Go through all of the collision geo for the object.
    for (const auto& [_, collision_geo] : *geo_set_or) {
      const auto& obj_t_geo = collision_geo.ref_t_shape();
      const eigenmath::Matrix4d root_t_geo = root_t_object.matrix() * obj_t_geo;

      INTR_ASSIGN_OR_RETURN(
          AxisAlignedBoundingBox3d shape_bounds,
          ComputeAxisAlignedBoundingBox3d(collision_geo.shape()));

      if (shape_bounds.IsEmpty()) {
        LOG(WARNING) << "entity id=" << entity_id
                     << ", name=" << entity->GetLocalName()
                     << " has empty bounds";
        continue;
      }

      for (const auto& p : shape_bounds.GetCorners()) {
        bounds.ExtendBy(Pose3d(root_t_geo) * p);
      }
    }
  }

  // TODO(stoyang): Evaluate if this is still needed.
  if (bounds.IsEmpty()) {
    // We always add a minimum box of 3x3x3 to make sure its non zero.
    bounds.ExtendBy(AxisAlignedBoundingBox3d({-3, -3, -3}, {3, 3, 3}));
  }

  if (!add_padding) {
    return bounds;
  }

  // Finally add 50% padding in each direction.
  static const double kPaddingMultiple = 1.5;
  return AxisAlignedBoundingBox3d(
      bounds.GetCenter() - bounds.GetDiagonal() * kPaddingMultiple,
      bounds.GetCenter() + bounds.GetDiagonal() * kPaddingMultiple);
}

}  // namespace intrinsic
