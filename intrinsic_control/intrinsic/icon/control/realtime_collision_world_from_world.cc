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

#include "intrinsic/icon/control/realtime_collision_world_from_world.h"

#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/rotation_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/api/affine_transform_of.h"
#include "intrinsic/geometry/api/affine_transform_of_geometry.h"
#include "intrinsic/geometry/api/exact_geometry.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/geometry/proto/lazy_exact_geometry.pb.h"
#include "intrinsic/geometry/proto/primitives.pb.h"
#include "intrinsic/geometry/shapes/box.h"
#include "intrinsic/geometry/shapes/capsule.h"
#include "intrinsic/geometry/shapes/cylinder.h"
#include "intrinsic/geometry/shapes/shape_base.h"
#include "intrinsic/geometry/shapes/sphere.h"
#include "intrinsic/geometry/storage/geometry_deserializer.h"
#include "intrinsic/icon/control/collision/collision_world_loader.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/build_tree_skeleton_from_world.h"
#include "intrinsic/world/component/collections_component.h"
#include "intrinsic/world/component/collision_component.h"
#include "intrinsic/world/component/geometry_component.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/geometry_types.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/objects/world_object.h"
#include "intrinsic/world/proto/collision_settings.pb.h"
#include "intrinsic/world/proto/geometry_component.pb.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"
#include "intrinsic/world/proto/object_world_service.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic::icon {
namespace {

constexpr double kEpsilon = 1e-6;

bool HasBlueGeometry(const ExactGeometry& geometry) {
  return !geometry.GetPrimitiveShapes().empty();
}

collision::Transformation PoseToTransformation(const Pose3d& pose) {
  collision::Transformation out;
  out.xyz = {pose.translation().begin(), pose.translation().end()};
  out.rpy.resize(3);
  intrinsic::eigenmath::RotationToRPY(pose.rotationMatrix(), &out.rpy.at(0),
                                      &out.rpy.at(1), &out.rpy.at(2));
  return out;
}

// Returns true if `physical_entity_id` can collide with any of the links in
// `robot_link_ids`.
//
// Put another way, this retrieves the collision exclusions for
// `physical_entity_id` from `world`, and returns true if `robot_link_ids` is a
// subset of them.
bool CanCollideWithRobotLink(
    PhysicalEntityId physical_entity_id, const World& world,
    const absl::flat_hash_set<PhysicalEntityId>& robot_link_ids) {
  // The third macro parameter can convert the return value (represented as
  // `_`). Since something that doesn't exist in `world` cannot collide with
  // anything, much less a robot link, we use this to convert any error status
  // to just `false`.
  INTR_ASSIGN_OR_RETURN(const WorldEntity* physical_entity,
                        world.GetEntityById(physical_entity_id), false);
  // Similar story for GeometryComponent – if our entity doesn't have one, it
  // can't collide, so return false.
  INTR_ASSIGN_OR_RETURN(const auto* geometry_component,
                        physical_entity->GetComponent<GeometryComponent>(),
                        false);
  // Check the collision geometry – if the entity doesn't have any geometry, it
  // can't collide with anything.
  if (!geometry_component->HasGeometry(kKindCollisionGeometry)) {
    return false;
  }
  // Finally, check CollisionComponent. Again, if our entity doesn't have one,
  // it can't collide, so return false.
  INTR_ASSIGN_OR_RETURN(const auto* collision_component,
                        physical_entity->GetComponent<CollisionComponent>(),
                        false);
  if (!collision_component->HasCollisionResponse()) {
    return false;
  }
  bool all_robot_links_excluded = absl::c_all_of(
      robot_link_ids, [&](PhysicalEntityId robot_link_id) -> bool {
        return collision_component->GetExclusions().contains(robot_link_id);
      });
  return !all_robot_links_excluded;
}

absl::StatusOr<intrinsic::collision::Collision> WorldGeometryToRealtimeGeometry(
    const eigenmath::Matrix4d ref_t_shape,
    const shapes::ShapeBase& world_shape) {
  // Decompose the transformation matrix. We only support uniform scaling,
  // translation and rotation, so raise an error if the matrix encodes any other
  // transformation (perspective, shearing or non-uniform scaling).
  INTR_ASSIGN_OR_RETURN((std::pair<Pose3d, eigenmath::Vector3d> pose_and_scale),
                        matrixToPoseAndScale(ref_t_shape));
  double scale = pose_and_scale.second(0);
  // Check for non-uniform scaling – if any element of the scale vector deviates
  // too much from the first element, that's not uniform!
  if ((pose_and_scale.second - eigenmath::Vector3d::Constant(scale))
          .cwiseAbs()
          .maxCoeff() > kEpsilon) {
    return absl::UnimplementedError(
        "Realtime collision checker does not support non-uniform scaling.");
  }

  // Convert the actual geometry definition. Two small obstacles:
  //
  // 1) Each blue_shapes type has an additional local transform. We check that
  // this is (approximately) identity, assuming that the AffineTransformOf has
  // the complete transformation, if any.
  //
  // 2) The collision checker annoyingly uses xyz + rpy transformations, so we
  //    need to convert our Pose3ds to that.
  //    TODO(b/268190697) tracks updating the collision checker to use
  //    quaternions rather than RPY angles.
  intrinsic::collision::Collision collision_shape;
  switch (world_shape.getType()) {
    case intrinsic::shapes::ShapeType::CYLINDER: {
      INTRINSIC_RT_LOG(INFO)
          << "Converting Cylinder geometry to capsule for RT "
             "collision checking.";
      const auto& blue_cylinder =
          world_shape.get<intrinsic::shapes::Cylinder>();
      collision_shape.origin = PoseToTransformation(pose_and_scale.first);
      collision_shape.geometry.cylinder = std::make_unique<collision::Capsule>(
          blue_cylinder.getRadius() * scale, blue_cylinder.getLength() * scale);
      break;
    }
    case intrinsic::shapes::ShapeType::CAPSULE: {
      const auto& blue_capsule = world_shape.get<intrinsic::shapes::Capsule>();
      collision_shape.origin = PoseToTransformation(pose_and_scale.first);
      collision_shape.geometry.cylinder = std::make_unique<collision::Capsule>(
          blue_capsule.getRadius() * scale, blue_capsule.getLength() * scale);
      break;
    }
    case intrinsic::shapes::ShapeType::SPHERE: {
      const auto& blue_sphere = world_shape.get<intrinsic::shapes::Sphere>();
      collision_shape.origin = PoseToTransformation(pose_and_scale.first);
      collision_shape.geometry.sphere =
          std::make_unique<collision::Sphere>(blue_sphere.getRadius() * scale);
      break;
    }
    case intrinsic::shapes::ShapeType::BOX: {
      const auto& blue_box = world_shape.get<intrinsic::shapes::Box>();
      collision_shape.origin = PoseToTransformation(pose_and_scale.first);
      collision_shape.geometry.box =
          std::make_unique<collision::Plane>(std::vector<double>{
              // Even though it's a plane, we need to supply the Z thickness.
              blue_box.getSize().x() * scale, blue_box.getSize().y() * scale,
              blue_box.getSize().z() * scale});
      break;
    }
    default:
      return absl::UnimplementedError(absl::StrCat(
          "Realtime collision checker does not support shape type '",
          shapes::ToString(world_shape.getType()), "'"));
  }
  return collision_shape;
}

bool EntityMatchesReference(
    const intrinsic_proto::world::Entity& entity,
    const ::intrinsic_proto::world::ObjectOrEntityReference& ref) {
  switch (ref.type_case()) {
    case ::intrinsic_proto::world::ObjectOrEntityReference::TypeCase::kEntity:
      return ref.entity().id() == entity.id();
    case ::intrinsic_proto::world::ObjectOrEntityReference::TypeCase::kObject:
      if (ref.object().has_by_name()) {
        return ref.object().by_name().object_name() == entity.object().name();
      }
      return ref.object().id() == entity.object().id();
    case ::intrinsic_proto::world::ObjectOrEntityReference::TypeCase::
        kObjectWithFilter: {
      const auto& object_ref = ref.object_with_filter().reference();
      if (object_ref.has_id()) {
        if (object_ref.id() != entity.object().id()) {
          return false;
        }
      } else if (object_ref.has_by_name()) {
        if (object_ref.by_name().object_name() != entity.object().name()) {
          return false;
        }
      } else {
        return false;
      }

      auto& entity_filter = ref.object_with_filter().entity_filter();
      if (entity_filter.include_all_entities()) {
        return true;
      }

      for (const std::string& entity_name : entity_filter.entity_names()) {
        if (entity_name == entity.name()) {
          return true;
        }
      }

      for (const auto& entity_ref : entity_filter.entity_references()) {
        if (entity_ref.id() == entity.id()) {
          return true;
        }
      }

      if (entity_filter.include_base_entity()) {
        // TODO: Need to support the full ObjectEntityFilter.
        LOG(ERROR) << "include_base_entity is not supported by "
                      "EntityMatchesReference(...)";
      }
      if (entity_filter.include_final_entity()) {
        // TODO: Need to support the full ObjectEntityFilter.
        LOG(ERROR) << "include_final_entity is not supported by "
                      "EntityMatchesReference(...)";
      }

      return false;
    }
    default:
      return false;
  }
}

void AddExclusionsForEntity(
    const intrinsic_proto::world::ObjectReference& object_ref,
    const intrinsic_proto::world::ObjectEntityFilter& filter,
    absl::Span<const world::WorldObject> objects,
    intrinsic::collision::ObjectInfo& collision_info) {
  if (!object_ref.has_by_name() && !object_ref.has_id()) {
    LOG(ERROR) << "ObjectReference does not have a name or id set.";
    return;
  }

  for (const world::WorldObject& object : objects) {
    if (object.Proto().entities().empty()) {
      continue;
    }

    if (object_ref.has_by_name()) {
      if (object.Name().value() != object_ref.by_name().object_name()) {
        continue;
      }
    } else if (object_ref.has_id()) {
      if (object.Id().value() != object_ref.id()) {
        continue;
      }
    }

    absl::flat_hash_set<const intrinsic_proto::world::Entity*>
        entities_to_check;
    if (filter.include_all_entities()) {
      for (const auto& [_, entity] : object.Proto().entities()) {
        entities_to_check.insert(&entity);
      }
    } else {
      if (filter.include_base_entity()) {
        // TODO: Need to support the full ObjectEntityFilter.
        LOG(ERROR) << "include_base_entity is not supported by "
                      "AddExclusionsForEntity(...)";
      }
      if (filter.include_final_entity()) {
        // TODO: Need to support the full ObjectEntityFilter.
        LOG(ERROR) << "include_final_entity is not supported by "
                      "AddExclusionsForEntity(...)";
      }
      for (const std::string& entity_name : filter.entity_names()) {
        for (const auto& [_, entity] : object.Proto().entities()) {
          if (entity.name() == entity_name) {
            entities_to_check.insert(&entity);
          }
        }
      }
      for (const auto& entity_ref : filter.entity_references()) {
        for (const auto& [_, entity] : object.Proto().entities()) {
          if (entity.id() == entity_ref.id()) {
            entities_to_check.emplace(&entity);
          }
        }
      }
    }

    for (const auto* entity : entities_to_check) {
      if (!entity->has_geometry_component()) {
        // No geometry component -> no collisions to exclude
        continue;
      }
      auto collision_geometry =
          entity->geometry_component().named_geometries().find(
              intrinsic::kKindCollisionGeometry);
      if (collision_geometry ==
          entity->geometry_component().named_geometries().end()) {
        // No collision geometry -> no collisions to exclude
        continue;
      }

      // If we reach this line, then
      // 1. We've found the right object
      // 2. The current entity within that object has a geometry component
      // 3. The geometry component has collision geometry
      // 4. The entity matches the provided filter.
      //
      // So add an exclusion for the entity!
      if (absl::c_find(collision_info.links_ignore_collision, entity->id()) ==
          collision_info.links_ignore_collision.end()) {
        collision_info.links_ignore_collision.push_back(entity->id());
      }
    }
  }
}

// Adds exclusions for any entities covered by `ref` to `collision_info`.
//
// If `ref` refers to an object, this function finds that object in `objects`
// and adds exclusions for any entities with collision geometry within it.
void AddExclusionsForRule(
    const ::intrinsic_proto::world::ObjectOrEntityReference& ref,
    absl::Span<const world::WorldObject> objects,
    intrinsic::collision::ObjectInfo& collision_info) {
  switch (ref.type_case()) {
    case ::intrinsic_proto::world::ObjectOrEntityReference::TypeCase::kEntity: {
      if (absl::c_find(collision_info.links_ignore_collision,
                       ref.entity().id()) ==
          collision_info.links_ignore_collision.end()) {
        collision_info.links_ignore_collision.push_back(ref.entity().id());
      }
      break;
    }
    case ::intrinsic_proto::world::ObjectOrEntityReference::TypeCase::kObject: {
      intrinsic_proto::world::ObjectEntityFilter filter;
      filter.set_include_all_entities(true);
      AddExclusionsForEntity(ref.object(), filter, objects, collision_info);
    } break;
    case ::intrinsic_proto::world::ObjectOrEntityReference::TypeCase::
        kObjectWithFilter: {
      AddExclusionsForEntity(ref.object_with_filter().reference(),
                             ref.object_with_filter().entity_filter(), objects,
                             collision_info);
    } break;
    default:
      break;
  }
}

}  // namespace

absl::StatusOr<CollisionWorldWithLinkNames> RealtimeCollisionWorldFromWorld(
    const World& world,
    bool enable_environment_collision) INTRINSIC_NON_REALTIME_ONLY {
  auto robot_collection_entities =
      world.GetTypedEntityIds<RobotCollectionsEntityId>();
  // First, remember all links that are part of a robot.
  absl::flat_hash_set<PhysicalEntityId> robot_link_entities;
  for (const RobotCollectionsEntityId& robot_collection :
       robot_collection_entities) {
    INTR_ASSIGN_OR_RETURN(auto validated_members,
                          world.ValidateCollectionMembers<PhysicalEntityId>(
                              robot_collection, CollectionsComponent::kLinks));
    for (const PhysicalEntityId& member : validated_members) {
      if (bool inserted = robot_link_entities.insert(member).second;
          !inserted) {
        INTR_ASSIGN_OR_RETURN(const WorldEntity* member_entity,
                              world.GetEntityById(member));
        return absl::AlreadyExistsError(
            absl::StrCat("Robot link with ID ", member.id.value(), " (",
                         member_entity->GetLocalName(),
                         ") is part of multiple RobotCollections."));
      }
    }
  }

  absl::flat_hash_set<PhysicalEntityId> entities_to_convert =
      robot_link_entities;
  if (enable_environment_collision) {
    // If we want to take into account collision between non-robot links and
    // robot links, we add those entities that can collide with robot links to
    // `entities_to_convert`. Note that this probably includes everything in
    // `robot_link_entities`, but since `entities_to_convert` is a set, that's
    // not a big deal.
    absl::c_copy_if(
        world.GetTypedEntityIds<PhysicalEntityId>(),
        std::inserter(entities_to_convert, entities_to_convert.end()),
        [&world,
         &robot_link_entities](const PhysicalEntityId& entity_id) -> bool {
          bool can_collide =
              CanCollideWithRobotLink(entity_id, world, robot_link_entities);
          return can_collide;
        });
  }

  // Now, we actually convert the entities to intrinsic::collision::Object
  // instances.
  absl::flat_hash_set<std::string> link_names;
  intrinsic::collision::CollisionModel collision_model;
  intrinsic::collision::CollisionCheckInfo collision_check_info;

  for (const PhysicalEntityId& physical_entity_id : entities_to_convert) {
    INTR_ASSIGN_OR_RETURN(const WorldEntity* physical_entity,
                          world.GetEntityById(physical_entity_id));
    if (bool inserted =
            link_names.insert(physical_entity->GetLocalName()).second;
        !inserted) {
      return absl::AlreadyExistsError(absl::StrCat(
          "Duplicate link name '", physical_entity->GetLocalName(), "'"));
    }

    INTR_ASSIGN_OR_RETURN(const auto* collision_component,
                          physical_entity->GetComponent<CollisionComponent>());
    INTR_ASSIGN_OR_RETURN(const auto* geo_component,
                          physical_entity->GetComponent<GeometryComponent>());
    INTR_ASSIGN_OR_RETURN(NamedGeometrySet collision_geo,
                          geo_component->GetGeometry(kKindCollisionGeometry));

    // This holds the collision geometry.
    intrinsic::collision::Object collision_object;
    // This holds exclusion information (i.e. the names of those links that
    // collision is disabled for).
    intrinsic::collision::ObjectInfo collision_info;
    collision_info.name =
        GetUniqueName(world, kRootEntityId, physical_entity_id);
    collision_object.name =
        GetUniqueName(world, kRootEntityId, physical_entity_id);

    for (const PhysicalEntityId& exclusion_id :
         collision_component->GetExclusions()) {
      std::string excluded_name =
          GetUniqueName(world, kRootEntityId, exclusion_id);
      collision_info.links_ignore_collision.push_back(excluded_name);
    }
    std::vector<intrinsic::collision::Collision> collision_shapes;
    for (const auto& [_, geom] : collision_geo) {
      const ExactGeometry& geometry = geom.shape().GetExactGeometry();
      if (!HasBlueGeometry(geometry)) {
        return absl::FailedPreconditionError(absl::StrCat(
            "Robot link ", physical_entity->GetLocalName(),
            " does not have primitive collision geometry. All collision links "
            "must have only primitive collision geometry. If you think this "
            "link should not be considered for collision in realtime, add "
            "exclusion pairs between it and all robot links."));
      }
      if (geometry.GetPrimitiveShapes().empty()) {
        return absl::FailedPreconditionError(absl::StrCat(
            "Link ", physical_entity->GetLocalName(),
            " does not have any collision geometry. If you think this link "
            "should not be considered for collision in realtime, add exclusion "
            "pairs between it and all robot links."));
      }
      for (const geo::TransformedPrimitiveShapePtr& shape :
           geometry.GetPrimitiveShapes()) {
        INTR_ASSIGN_OR_RETURN(
            collision_shapes.emplace_back(),
            WorldGeometryToRealtimeGeometry(
                geom.ref_t_shape() * shape.ref_t_shape(), *shape.shape()));
      }
    }
    // Put box shapes(planes) in the top because the CollisionWorld is peculiar
    // about where it does and doesn't allow planes).
    const bool has_plane =
        absl::c_count_if(collision_shapes,
                         [](const intrinsic::collision::Collision& collision) {
                           return collision.geometry.box != nullptr;
                         }) != 0;
    collision_object.AddCollisionGeometries(std::move(collision_shapes));
    if (has_plane) {
      collision_model.link.insert(collision_model.link.begin(),
                                  std::move(collision_object));
      collision_check_info.link_info.insert(
          collision_check_info.link_info.begin(), std::move(collision_info));
    } else {
      collision_model.link.push_back(std::move(collision_object));
      collision_check_info.link_info.push_back(std::move(collision_info));
    }
  }
  CollisionWorldWithLinkNames world_with_link_names;

  collision::LoaderStatus status = collision::LoadCollisionWorld(
      std::move(collision_model), std::move(collision_check_info),
      world_with_link_names.collision_world, world_with_link_names.link_names);
  if (status != collision::LoaderStatus::SUCCESS) {
    return absl::InternalError(
        absl::StrCat("Failed to load collision world: ",
                     collision::GetLoaderStatusMessage(status)));
  }
  return std::move(world_with_link_names);
}

absl::StatusOr<CollisionWorldWithLinkNames> RealtimeCollisionWorldFromWorld(
    const world::ObjectWorldClient& world_client,
    const GeometryDeserializer& geometry_deserializer,
    bool enable_environment_collision) INTRINSIC_NON_REALTIME_ONLY {
  INTR_ASSIGN_OR_RETURN(std::vector<world::WorldObject> objects,
                        world_client.ListObjects());
  // This can't easily be a set because protos don't have a default hash
  // operator...
  absl::flat_hash_map<std::string, intrinsic_proto::world::Entity>
      entities_to_convert_by_id;
  for (const auto& object : objects) {
    bool is_physical_object =
        object.Proto().type() ==
        intrinsic_proto::world::ObjectType::PHYSICAL_OBJECT;
    bool is_robot = object.Proto().type() ==
                    intrinsic_proto::world::ObjectType::KINEMATIC_OBJECT;
    // Only add the entities for this object if either
    // * It's a robot
    // * It's a non-robot collision object AND we want to check environmental
    //   collisions
    if (!is_robot && !(enable_environment_collision && is_physical_object)) {
      continue;
    }
    for (const auto& [id, entity] : object.Proto().entities()) {
      // Skip entities that definitely do not have collision geometry. Note that
      // having a geometry component is not a sufficient criterion for having
      // actual collision geometry (that is compatible with the RT collision
      // world), so we do another check later.
      if (entity.has_geometry_component()) {
        entities_to_convert_by_id.emplace(id, entity);
      }
    }
  }

  INTR_ASSIGN_OR_RETURN(
      intrinsic_proto::world::CollisionSettings world_collision_settings,
      world_client.GetCollisionSettings());
  // Now, we actually convert the entities to intrinsic::collision::Object
  // instances.
  absl::flat_hash_set<std::string> link_ids;
  intrinsic::collision::CollisionModel collision_model;
  intrinsic::collision::CollisionCheckInfo collision_check_info;

  for (const auto& [id, entity] : entities_to_convert_by_id) {
    // 1. Get world entity and ensure it's got a unique ID
    if (bool inserted = link_ids.insert(id).second; !inserted) {
      return absl::AlreadyExistsError(absl::StrCat(
          "Duplicate link ID '", id, "' (Entity name is ", entity.name(), ")"));
    }
    INTR_ASSIGN_OR_RETURN(
        std::unique_ptr<GeometryComponent> geometry_component,
        GeometryComponent::FromProto(entity.geometry_component(),
                                     geometry_deserializer));
    // 2. Get collision geometry (not visual geometry!)
    if (!geometry_component->GetGeometryNames().contains(
            kKindCollisionGeometry)) {
      continue;
    }

    INTR_ASSIGN_OR_RETURN(
        NamedGeometrySet collision_geo,
        geometry_component->GetGeometry(kKindCollisionGeometry));

    intrinsic::collision::Object collision_object;
    collision_object.name = entity.id();
    intrinsic::collision::ObjectInfo collision_info;
    collision_info.name = collision_object.name;

    for (const intrinsic_proto::world::CollisionSettings::CollisionRule& rule :
         world_collision_settings.collision_rules()) {
      if (rule.collision_action().has_is_excluded() &&
          !rule.collision_action().is_excluded()) {
        // Skip any rules that *aren't* exclusions
        continue;
      }
      bool entity_matches_left = absl::c_any_of(
          rule.left(),
          [&](const ::intrinsic_proto::world::ObjectOrEntityReference& ref) {
            return EntityMatchesReference(entity, ref);
          });
      if (entity_matches_left && rule.right().empty()) {
        // This means the rule applies to _all_ entities and objects in the
        // world, essentially no collisions for this entity with anything ever.
        for (const world::WorldObject& object : objects) {
          ::intrinsic_proto::world::ObjectOrEntityReference ref;
          *ref.mutable_object() = object.ObjectReference();
          AddExclusionsForRule(ref, objects, collision_info);
        }
        // Skip the rest of the rule handling code – it's a no-op anyway (since
        // `rule.right()` is empty), but it's nice to be explicit about it.
        continue;
      }
      bool entity_matches_right = absl::c_any_of(
          rule.right(),
          [&](const ::intrinsic_proto::world::ObjectOrEntityReference& ref) {
            return EntityMatchesReference(entity, ref);
          });
      // If the entity matches one of the `left` rules, we add exclusions for
      // all entities that match any of the `right` rules.
      if (entity_matches_left) {
        for (const ::intrinsic_proto::world::ObjectOrEntityReference& ref :
             rule.right()) {
          AddExclusionsForRule(ref, objects, collision_info);
        }
      }
      // If the entity matches one of the `right` rules, we add exclusions for
      // all entities that match any of the `left` rules.
      if (entity_matches_right) {
        for (const ::intrinsic_proto::world::ObjectOrEntityReference& ref :
             rule.left()) {
          AddExclusionsForRule(ref, objects, collision_info);
        }
      }
    }

    for (const auto& [_, geo] : collision_geo) {
      const std::vector<geo::TransformedPrimitiveShapePtr>& primitives =
          geo.shape().GetExactGeometry().GetPrimitiveShapes();
      if (primitives.empty()) {
        continue;
      }
      // 3. Convert to collision::Object (including collision exclusion pairs)
      std::vector<collision::Collision> current_geo_collision_shapes;
      current_geo_collision_shapes.reserve(primitives.size());
      for (const geo::TransformedPrimitiveShapePtr& shape : primitives) {
        if (shape.shape() == nullptr) continue;
        INTR_ASSIGN_OR_RETURN(
            collision::Collision collision,
            WorldGeometryToRealtimeGeometry(
                geo.ref_t_shape() * shape.ref_t_shape(), *shape.shape()),
            _ << "for world entity '" << entity.name() << "' (ID: " << id
              << ")");
        current_geo_collision_shapes.push_back(std::move(collision));
      }
      collision_object.AddCollisionGeometries(
          std::move(current_geo_collision_shapes));
    }
    // Put box shapes(planes) at the beginning of the link list because the
    // CollisionWorld is peculiar
    // about where it does and doesn't allow planes).
    const bool has_plane =
        absl::c_count_if(collision_object.collision,
                         [](const intrinsic::collision::Collision& collision) {
                           return collision.geometry.box != nullptr;
                         }) != 0;
    if (has_plane) {
      collision_model.link.insert(collision_model.link.begin(),
                                  std::move(collision_object));
      collision_check_info.link_info.insert(
          collision_check_info.link_info.begin(), std::move(collision_info));
    } else {
      collision_model.link.push_back(std::move(collision_object));
      collision_check_info.link_info.push_back(std::move(collision_info));
    }
  }
  CollisionWorldWithLinkNames world_with_link_names;

  collision::LoaderStatus status = collision::LoadCollisionWorld(
      std::move(collision_model), std::move(collision_check_info),
      world_with_link_names.collision_world, world_with_link_names.link_names);
  if (status != collision::LoaderStatus::SUCCESS) {
    return absl::InternalError(
        absl::StrCat("Failed to load collision world: ",
                     collision::GetLoaderStatusMessage(status)));
  }
  return std::move(world_with_link_names);
}

}  // namespace intrinsic::icon
