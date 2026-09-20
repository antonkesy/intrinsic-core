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

#include "intrinsic/icon/utils/world_utils.h"

#include <algorithm>
#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/strings/substitute.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/link.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/string_type.h"
#include "intrinsic/world/component/ppr_component.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/frame.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/world_object.h"
#include "intrinsic/world/proto/kinematics_component.pb.h"
#include "intrinsic/world/proto/object_world_service.pb.h"
#include "intrinsic/world/proto/physics_component.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic::icon {

namespace {

using PPRRobotCollectionsEntityId =
    TypedEntityId<CollectionsComponentType, RobotComponentType,
                  PPRComponentType>;

std::vector<RobotCollectionsEntityId> GetRobotCollectionsWithInstanceName(
    const World& world, absl::string_view resource_instance_name) {
  std::vector<PPRRobotCollectionsEntityId> entities =
      world.GetTypedEntityIds<PPRRobotCollectionsEntityId>();
  {
    auto component_name_does_not_match =
        [&world,
         &resource_instance_name](PPRRobotCollectionsEntityId id) -> bool {
      // Calling value() without checking the status, and dereferencing the
      // pointer is safe here: All members of `entities` must have a non-null
      // PPRComponent, otherwise GetTypedEntityIds() would not have returned
      // them.
      return world.GetComponentByEntityId<PPRComponent>(id)
                 .value()
                 ->ResourceName() != resource_instance_name;
    };
    entities.erase(std::remove_if(entities.begin(), entities.end(),
                                  component_name_does_not_match),
                   entities.end());
  }
  // PPRRobotCollectionsEntityId casts to RobotCollectionsEntityId, but we need
  // to explicitly construct a new vector to do that.
  return {entities.begin(), entities.end()};
}

std::vector<std::string> GetLocalNames(
    const World& world, absl::Span<const RobotCollectionsEntityId> entities) {
  std::vector<std::string> local_names;
  absl::c_transform(entities, std::back_inserter(local_names),
                    [&world](EntityId id) -> std::string {
                      return world.GetLocalNameForEntityById(id);
                    });
  return local_names;
}

INTRINSIC_DEFINE_STRING_TYPE_AS(EntityIdString,
                                intrinsic::SharedPtrStringRepresentation);
INTRINSIC_DEFINE_STRING_TYPE_AS(FrameIdString,
                                intrinsic::SharedPtrStringRepresentation);

// Composite ID to make it easier to look up a given Entity or Frame.
// Looking up an Entity or Frame is a two-step process:
// 1. Look up the WorldObject with ID `object_id` in `WorldGraph::id_to_object`
// 2. If `sub_id` holds an `EntityIdString`, then search its `Proto()` for an
//    Entity that matches `sub_id`.
//    If `sub_id` holds a `FrameIdString`, then search its `Proto()` for a Frame
//    that matches `sub_id`.
struct EntityOrFrameId {
  std::string parent_object_id;
  std::variant<EntityIdString, FrameIdString> sub_id;

  // Needs operator== and AbslHashValue to go into a flat_hash_set
  friend bool operator==(const EntityOrFrameId& lhs,
                         const EntityOrFrameId& rhs);

  template <typename H>
  friend H AbslHashValue(H h, const EntityOrFrameId& id) {
    struct HashVisitor {
      std::string operator()(const EntityIdString& eid) {
        return absl::StrCat("eid_", eid.value());
      }
      std::string operator()(const FrameIdString& fid) {
        return absl::StrCat("fid_", fid.value());
      }
    };
    return H::combine(std::move(h), id.parent_object_id,
                      std::visit(HashVisitor{}, id.sub_id));
  }
};

bool operator==(const EntityOrFrameId& lhs, const EntityOrFrameId& rhs) {
  return lhs.parent_object_id == rhs.parent_object_id &&
         lhs.sub_id == rhs.sub_id;
}

struct WorldGraph {
  // Map from a World ID to the corresponding WorldObject
  absl::flat_hash_map<std::string, world::WorldObject> id_to_object;
  // Map from the ID of a World thing (Object, Entity or Frame) to the IDs of
  // its children
  absl::flat_hash_map<std::string, absl::flat_hash_set<EntityOrFrameId>>
      id_to_child_ids;
};

struct GetIdVisitor {
  std::optional<std::string> operator()(const EntityIdString& eid) {
    return eid.value();
  }
  std::optional<std::string> operator()(const FrameIdString& fid) {
    return fid.value();
  }
};

std::string GetIdForChildLookup(const EntityOrFrameId& wtid) {
  return std::visit(GetIdVisitor{}, wtid.sub_id)
      .value_or(wtid.parent_object_id);
}

}  // namespace

absl::StatusOr<std::unique_ptr<const DofKinematicView>>
GetUniqueDofKinematicView(const World& world,
                          std::optional<std::string> resource_instance_name) {
  std::vector<RobotCollectionsEntityId> robot_collections;
  if (resource_instance_name.has_value()) {
    robot_collections = GetRobotCollectionsWithInstanceName(
        world, resource_instance_name.value());
  } else {
    robot_collections = world.GetTypedEntityIds<RobotCollectionsEntityId>();
  }
  if (robot_collections.empty()) {
    return absl::NotFoundError(absl::Substitute(
        "There are no RobotCollections$0 in the world.",
        (resource_instance_name.has_value()
             ? absl::Substitute(" with resource instance name '$0'",
                                *resource_instance_name)
             : "")));
  }
  if (robot_collections.size() > 1) {
    return absl::FailedPreconditionError(absl::Substitute(
        "There is more than one RobotCollection$0 in the world. Consider "
        "disambiguating by resource name. Resource names: [$1]",
        (resource_instance_name.has_value()
             ? absl::Substitute(" with resource instance name '$0'",
                                *resource_instance_name)
             : ""),
        absl::StrJoin(GetLocalNames(world, robot_collections), ", ")));
  }
  return world.GetDofKinematicView(robot_collections.front());
}

absl::StatusOr<std::unique_ptr<const DofKinematicView>>
GetDofKinematicViewByLocalName(
    const World& world, absl::string_view local_name,
    std::optional<std::string> resource_instance_name) {
  std::vector<RobotCollectionsEntityId> robot_collections;
  if (resource_instance_name.has_value()) {
    robot_collections = GetRobotCollectionsWithInstanceName(
        world, resource_instance_name.value());
  } else {
    robot_collections = world.GetTypedEntityIds<RobotCollectionsEntityId>();
  }
  if (robot_collections.empty()) {
    return absl::NotFoundError(absl::Substitute(
        "There are no RobotCollections$0 in the world.",
        (resource_instance_name.has_value()
             ? absl::Substitute(" with resource instance name '$0'",
                                *resource_instance_name)
             : "")));
  }

  {
    auto local_name_does_not_match =
        [&world, &local_name](RobotCollectionsEntityId id) -> bool {
      return world.GetLocalNameForEntityById(id) != local_name;
    };
    robot_collections.erase(
        std::remove_if(robot_collections.begin(), robot_collections.end(),
                       local_name_does_not_match),
        robot_collections.end());
  }

  if (robot_collections.empty()) {
    return absl::NotFoundError(absl::Substitute(
        "There are no RobotCollections$0 in the world that match the local "
        "name '$1'. Maybe try to look up by alias instead?",
        (resource_instance_name.has_value()
             ? absl::Substitute(" with resource instance name '$0'",
                                *resource_instance_name)
             : ""),
        local_name));
  }
  if (robot_collections.size() > 1) {
    return absl::FailedPreconditionError(absl::Substitute(
        "There are multiple RobotCollections$0 in the world that match the "
        "local name '$1'. Maybe disambiguate by alias instead?",
        (resource_instance_name.has_value()
             ? absl::Substitute(" with resource instance name '$0'",
                                *resource_instance_name)
             : ""),
        local_name));
  }
  return world.GetDofKinematicView(*robot_collections.begin());
}

absl::StatusOr<std::unique_ptr<const DofKinematicView>>
GetDofKinematicViewByAlias(const World& world, absl::string_view alias) {
  INTR_ASSIGN_OR_RETURN(EntityId id, world.FindByAlias(alias));
  INTR_ASSIGN_OR_RETURN(RobotCollectionsEntityId robot_id,
                        world.ValidateEntity<RobotCollectionsEntityId>(id));
  return world.GetDofKinematicView(robot_id);
}

WorldGraph BuildWorldGraph(absl::Span<const world::WorldObject> all_objects) {
  WorldGraph graph;
  graph.id_to_object.reserve(all_objects.size());
  for (const auto& current_object : all_objects) {
    const intrinsic_proto::world::Object& current_object_proto =
        current_object.Proto();
    graph.id_to_object.emplace(current_object_proto.id(), current_object);

    for (const auto& [_, entity] : current_object_proto.entities()) {
      if (entity.parent_id().empty()) {
        // This is the root entity
        continue;
      }
      graph.id_to_child_ids[entity.parent_id()].insert(EntityOrFrameId{
          .parent_object_id = current_object_proto.id(),
          .sub_id = EntityIdString{entity.id()},
      });
    }

    // Frames are a special case. They can either be derived from an entity, or
    // not.
    // * If a frame is derived from an entity, then we can skip it here. We've
    //   already added the entity to the world graph above, and we add an
    //   "alias" in `AddFrameIdsToSkeletonIdMap()` later.
    //   We can detect this case by finding an entity in the same object that
    //   shares a name with the frame.
    // * If a frame is *not* derived from an entity, we need to add it to the
    //   graph so that it is represented in the output Skeleton.
    for (const auto& frame : current_object_proto.frames()) {
      // Search the parent object's `entities` map for an entity with the same
      // name as `frame`.
      auto entity = absl::c_find_if(
          current_object_proto.entities(), [&](const auto& id_and_entity) {
            return id_and_entity.second.name() == frame.name();
          });
      if (entity != current_object_proto.entities().end()) {
        // There *is* a corresponding entity. We've already added that to the
        // graph, so we can add a duplicate entry to the World ID <-> Skeleton
        // ID map later (in `AddFrameIdsToSkeletonIdMap()`).
        continue;
      }
      // There's *no* corresponding entity, so we add the frame itself to the
      // graph.
      graph.id_to_child_ids[current_object_proto.root_entity_id()].insert(
          EntityOrFrameId{
              .parent_object_id = current_object_proto.id(),
              .sub_id = FrameIdString{frame.id()},
          });
    }
  }
  return graph;
}

absl::StatusOr<kinematics::Joint::Parameters>
JointParamsFromKinematicsComponent(
    const intrinsic_proto::world::KinematicsComponent& kinematics_component) {
  // Populate joint parameters.
  kinematics::Joint::Parameters params;
  switch (kinematics_component.motion_type()) {
    case intrinsic_proto::world::KinematicsComponent::MOTION_TYPE_REVOLUTE:
      params.type = kinematics::Joint::REVOLUTE;
      break;
    case intrinsic_proto::world::KinematicsComponent::MOTION_TYPE_PRISMATIC:
      params.type = kinematics::Joint::PRISMATIC;
      break;
    case intrinsic_proto::world::KinematicsComponent::MOTION_TYPE_FIXED:
      params.type = kinematics::Joint::FIXED;
      break;
    default:
      return intrinsic::UnimplementedErrorBuilder()
             << "Unhandled joint motion type "
             << kinematics_component.motion_type();
  }
  if (kinematics_component.has_axis()) {
    params.axis = intrinsic_proto::FromProto(kinematics_component.axis());
  } else {
    // Axis defaults to unit Z (see
    // http://intrinsic/world/public/proto/kinematics_component.proto;l=69;rcl=625285539)
    params.axis = eigenmath::Vector3d::UnitZ();
  }

  if (kinematics_component.has_system_limits()) {
    if (kinematics_component.system_limits().has_fixed_limits()) {
      params.system_limits.position.lower =
          kinematics_component.system_limits().fixed_limits().lower();
      params.system_limits.position.upper =
          kinematics_component.system_limits().fixed_limits().upper();
    }
    params.system_limits.velocity =
        kinematics_component.system_limits().velocity();
    params.system_limits.acceleration =
        kinematics_component.system_limits().acceleration();
    params.system_limits.jerk = kinematics_component.system_limits().jerk();
    params.system_limits.effort = kinematics_component.system_limits().effort();
  }
  if (kinematics_component.has_application_limits()) {
    if (kinematics_component.application_limits().has_fixed_limits()) {
      params.soft_limits.position.lower =
          kinematics_component.application_limits().fixed_limits().lower();
      params.soft_limits.position.upper =
          kinematics_component.application_limits().fixed_limits().upper();
    }
    params.soft_limits.velocity =
        kinematics_component.application_limits().velocity();
    params.soft_limits.acceleration =
        kinematics_component.application_limits().acceleration();
    params.soft_limits.jerk = kinematics_component.application_limits().jerk();
    params.soft_limits.effort =
        kinematics_component.application_limits().effort();
  }
  return params;
}

absl::StatusOr<kinematics::Link::Parameters> LinkParamsFromPhysicsComponent(
    const intrinsic_proto::world::PhysicsComponent& physics_component) {
  kinematics::Link::Parameters link_parameters;
  // = {
  //     .center_of_gravity = physics_component.->GetThisTCenterOfMass(),
  //     .inertia = physics_component->GetInertiaMatrix(),
  //     .mass = physics_component->GetMassKg()};
  if (physics_component.has_this_t_center_of_mass()) {
    INTR_ASSIGN_OR_RETURN(
        link_parameters.center_of_gravity,
        intrinsic_proto::FromProto(physics_component.this_t_center_of_mass()));
  }
  if (physics_component.has_inertia()) {
    INTR_ASSIGN_OR_RETURN(
        link_parameters.inertia,
        intrinsic_proto::FromProto(physics_component.inertia()));
  }
  link_parameters.mass = physics_component.mass_kg();

  return link_parameters;
}

struct AddWorldThingVisitor {
  absl::Status operator()(const EntityIdString& entity_id) {
    auto id_and_obj = world_graph.id_to_object.find(object_id);
    if (id_and_obj == world_graph.id_to_object.end()) {
      return absl::NotFoundError(absl::StrCat("Cannot find parent object '",
                                              object_id, "' of Entity '",
                                              entity_id.value(), "'"));
    }
    const world::WorldObject& object = id_and_obj->second;
    auto id_and_entity = object.Proto().entities().find(entity_id.value());
    if (id_and_entity == object.Proto().entities().end()) {
      return absl::NotFoundError(absl::StrCat(
          "Entity '", entity_id.value(), "' should be a child of Object '",
          object.Name().value(), "' (", object_id, "), but '",
          object.Name().value(), "' does not list '", entity_id.value(),
          "' as a child Entity"));
    }
    const intrinsic_proto::world::Entity& entity = id_and_entity->second;
    // If the current element is the root of our subtree, use kInvalidElementId
    // for the Skeleton parent – that ID works as a "root" ID.
    intrinsic::kinematics::ElementId parent_skeleton_id =
        kinematics::kInvalidElementId;
    if (entity.id() != subtree_root) {
      // Try to find the Skeleton ID of `entity`'s parent
      auto parent_world_id_and_skeleton_id =
          world_id_to_skeleton_id.find(entity.parent_id());
      if (parent_world_id_and_skeleton_id == world_id_to_skeleton_id.end()) {
        return absl::InternalError(absl::StrCat(
            "Failed to find Skeleton ElementId for parent '",
            entity.parent_id(), "' of Entity '", entity.name(), "' (",
            entity.id(),
            "). This is an internal error, please file a bug report."));
      }
      parent_skeleton_id = parent_world_id_and_skeleton_id->second;
    }

    if (entity.has_kinematics_component()) {
      // If the Entity has a KinematicsComponent, it is a Joint
      INTR_ASSIGN_OR_RETURN(
          intrinsic::Pose parent_t_inboard,
          intrinsic_proto::FromProto(
              entity.kinematics_component().parent_t_inboard()));
      INTR_ASSIGN_OR_RETURN(
          auto joint_params,
          JointParamsFromKinematicsComponent(entity.kinematics_component()));
      INTR_ASSIGN_OR_RETURN(
          auto skeleton_id,
          skeleton.CreateJoint(entity.id(), std::move(joint_params),
                               parent_skeleton_id, parent_t_inboard));
      if (bool inserted =
              world_id_to_skeleton_id.try_emplace(entity.id(), skeleton_id)
                  .second;
          !inserted) {
        return absl::AlreadyExistsError(
            absl::StrCat("Cannot build Skeleton from World: Tried to build "
                         "more than one Skeleton element for World Entity '",
                         entity.name(), "' ()", entity.id(), ")"));
      }
    } else if (entity.has_physics_component()) {
      // If the Entity has a PhysicsComponent, it is a Link
      INTR_ASSIGN_OR_RETURN(intrinsic::Pose parent_t_this,
                            intrinsic_proto::FromProto(entity.parent_t_this()));
      INTR_ASSIGN_OR_RETURN(auto link_params, LinkParamsFromPhysicsComponent(
                                                  entity.physics_component()));
      INTR_ASSIGN_OR_RETURN(
          auto skeleton_id,
          skeleton.CreateLink(entity.id(), std::move(link_params),
                              parent_skeleton_id, parent_t_this));
      if (bool inserted =
              world_id_to_skeleton_id.try_emplace(entity.id(), skeleton_id)
                  .second;
          !inserted) {
        return absl::AlreadyExistsError(
            absl::StrCat("Cannot build Skeleton from World: Tried to build "
                         "more than one Skeleton element for World Entity '",
                         entity.name(), "' ()", entity.id(), ")"));
      }
    } else {
      // Otherwise, it is a CoordinateFrame (for Skeleton purposes at least. It
      // might actually be a Sensor, but Skeleton doesn't model those
      // explicitly).
      intrinsic::Pose parent_t_this = intrinsic::Pose::Identity();
      if (entity.has_parent_t_this()) {
        INTR_ASSIGN_OR_RETURN(
            parent_t_this, intrinsic_proto::FromProto(entity.parent_t_this()));
      }
      INTR_ASSIGN_OR_RETURN(
          auto skeleton_id,
          skeleton.CreateCoordinateFrame(entity.id(), parent_skeleton_id,
                                         parent_t_this));
      if (bool inserted =
              world_id_to_skeleton_id.try_emplace(entity.id(), skeleton_id)
                  .second;
          !inserted) {
        return absl::AlreadyExistsError(
            absl::StrCat("Cannot build Skeleton from World: Tried to build "
                         "more than one Skeleton element for World Entity '",
                         entity.name(), "' ()", entity.id(), ")"));
      }
    }
    return absl::OkStatus();
  }

  absl::Status operator()(const FrameIdString& frame_id) {
    auto id_and_obj = world_graph.id_to_object.find(object_id);
    if (id_and_obj == world_graph.id_to_object.end()) {
      return absl::NotFoundError(absl::StrCat("Cannot find parent object '",
                                              object_id, "' of Frame '",
                                              frame_id.value(), "'"));
    }
    const world::WorldObject& object = id_and_obj->second;
    auto frame_it =
        absl::c_find_if(object.Proto().frames(),
                        [&](const intrinsic_proto::world::Frame& frame_proto) {
                          return frame_proto.id() == frame_id.value();
                        });
    if (frame_it == object.Proto().frames().end()) {
      return absl::NotFoundError(
          absl::StrCat("Frame '", frame_id.value(),
                       "' should be a child of Object '", object.Name().value(),
                       "' (", object_id, "), but '", object.Name().value(),
                       "' does not list '", frame_id.value(), "' as a Frame"));
    }
    const intrinsic_proto::world::Frame& frame = *frame_it;
    // If the current element is the root of our subtree, use kInvalidElementId
    // for the Skeleton parent – that ID works as a "root" ID.
    intrinsic::kinematics::ElementId parent_skeleton_id =
        kinematics::kInvalidElementId;
    if (frame_id.value() != subtree_root) {
      // Try to find the Skeleton ID of `entity`'s parent
      if (frame.has_parent_frame()) {
        auto parent_world_id_and_skeleton_id =
            world_id_to_skeleton_id.find(frame.parent_frame().id());
        if (parent_world_id_and_skeleton_id == world_id_to_skeleton_id.end()) {
          return absl::InternalError(absl::StrCat(
              "Failed to find Skeleton ElementId for parent '",
              frame.parent_frame().id(), "' of Frame '", frame.name(), "' (",
              frame.id(),
              "). This is an internal error, please file a bug report."));
        }
        parent_skeleton_id = parent_world_id_and_skeleton_id->second;

      } else {
        // If we don't have a parent frame, we have a parent object. But an
        // object does not have a pose by itself, so look for the object's root
        // entity instead.

        auto parent_world_id_and_object =
            world_graph.id_to_object.find(frame.object().id());
        if (parent_world_id_and_object == world_graph.id_to_object.end()) {
          return absl::InternalError(absl::StrCat(
              "Failed to find parent Object '", frame.object().name(), "' (",
              frame.object().id(), ") of Frame '", frame.name(), "' (",
              frame.id(),
              "). This is an internal error, please file a bug report."));
        }
        auto parent_world_id_and_skeleton_id = world_id_to_skeleton_id.find(
            parent_world_id_and_object->second.Proto().root_entity_id());
        if (parent_world_id_and_skeleton_id == world_id_to_skeleton_id.end()) {
          return absl::InternalError(absl::StrCat(
              "Failed to find Skeleton ElementId for root Entity of parent "
              "Object '",
              frame.object().name(), "' (", frame.object().id(), ") of Frame '",
              frame.name(), "' (", frame.id(),
              "). This is an internal error, please file a bug report."));
        }
        parent_skeleton_id = parent_world_id_and_skeleton_id->second;
      }
    }

    // Otherwise, it is a CoordinateFrame (for Skeleton purposes at least. It
    // might actually be a Sensor, but Skeleton doesn't model those
    // explicitly).
    INTR_ASSIGN_OR_RETURN(intrinsic::Pose parent_t_this,
                          intrinsic_proto::FromProto(frame.parent_t_this()));
    INTR_ASSIGN_OR_RETURN(auto skeleton_id,
                          skeleton.CreateCoordinateFrame(
                              frame.id(), parent_skeleton_id, parent_t_this));
    if (bool inserted =
            world_id_to_skeleton_id.try_emplace(frame.id(), skeleton_id).second;
        !inserted) {
      return absl::AlreadyExistsError(
          absl::StrCat("Cannot build Skeleton from World: Tried to build "
                       "more than one Skeleton element for World Frame '",
                       frame.name(), "' (", frame.id(), ")"));
    }

    return absl::OkStatus();
  }

  absl::string_view object_id;
  const WorldGraph& world_graph;
  absl::string_view subtree_root;
  kinematics::Skeleton& skeleton;
  absl::flat_hash_map<std::string, kinematics::ElementId>&
      world_id_to_skeleton_id;
};

// Looks up `current_id` in `world_graph`, and adds the
// corresponding element to `skeleton`. That element can one of the following
// three:
// * A Link (for Entities with a PhysicsComponent)
// * A Joint (for Entities with a KinematicsComponent)
// * A CoordinateFrame (for all other Entities)
//
// Note that, as far as Objects go, we only care about the "root" object,
// because that is special and things might list it as their parent. All other
// Objects are just wrappers around the Entities that we really care about.
//
// Returns an error if `world_graph` or the underlying World data is
// inconsistent.
absl::Status AddWorldThingToSkeleton(
    const WorldGraph& world_graph, const EntityOrFrameId& current_id,
    absl::string_view subtree_root, kinematics::Skeleton& skeleton,
    absl::flat_hash_map<std::string, kinematics::ElementId>&
        world_id_to_skeleton_id) {
  return std::visit(
      AddWorldThingVisitor{
          .object_id = current_id.parent_object_id,
          .world_graph = world_graph,
          .subtree_root = subtree_root,
          .skeleton = skeleton,
          .world_id_to_skeleton_id = world_id_to_skeleton_id,
      },
      current_id.sub_id);
}

absl::Status AddFrameIdsToSkeletonIdMap(
    absl::Span<const world::WorldObject> objects,
    absl::flat_hash_map<std::string, kinematics::ElementId>&
        world_id_to_skeleton_id) {
  // For each object...
  for (const auto& object : objects) {
    // Iterate over its Frames
    // For each Frame...
    for (const world::Frame& frame : object.Frames()) {
      if (world_id_to_skeleton_id.contains(frame.Id().value())) {
        // Frame ID is already present in Skeleton ID map, move along.
        continue;
      }
      // Frame ID is not yet in the Skeleton ID map. This means that there is an
      // *entity* that corresponds to this frame, and the entity *is* in the
      // map.
      //
      // Let's find that entity, by first finding the frame's parent object.
      auto parent_object =
          absl::c_find_if(objects, [&](const world::WorldObject& obj) {
            return obj.Id() == frame.ObjectId();
          });
      if (parent_object == objects.end()) {
        return absl::InternalError(absl::StrCat(
            "Failed to find parent object '", frame.ObjectName().value(),
            "' for Frame '", frame.Name().value(), "' of Object '",
            object.Name().value(), "'. Please file a bug"));
      }
      // Search `parent_object` for an Entity with the same name as `frame`
      auto entity_it = absl::c_find_if(
          parent_object->Proto().entities(), [&](const auto& id_and_entity) {
            return id_and_entity.second.name() == frame.Name().value();
          });
      if (entity_it == parent_object->Proto().entities().end()) {
        return absl::InternalError(
            absl::StrCat("Failed to find corresponding Entity for Frame '",
                         frame.Name().value(), "' of Object '",
                         object.Name().value(), "'. Please file a bug"));
      }
      // Look up the Skeleton ID for the Entity.
      auto entity_id_and_skeleton_id =
          world_id_to_skeleton_id.find(entity_it->second.id());
      if (entity_id_and_skeleton_id == world_id_to_skeleton_id.end()) {
        // If we can't find a Skeleton ID for this Entity, that means the Entity
        // isn't part of this particular Skeleton, so continue.
        continue;
      }
      // Save the Skeleton ID before inserting, because the insertion
      // invalidates the iterator we got from `find()`.
      kinematics::ElementId skeleton_id = entity_id_and_skeleton_id->second;

      // Map the Frame's World ID to the same Skeleton ID
      if (bool inserted = world_id_to_skeleton_id
                              .try_emplace(frame.Id().value(), skeleton_id)
                              .second;
          !inserted) {
        return absl::AlreadyExistsError(
            absl::StrCat("The ID of frame '", frame.Name().value(), "' (",
                         frame.Id().value(), ") in Object '",
                         object.Name().value(), "' is not unique"));
      }
    }
  }
  return absl::OkStatus();
}

// Builds a Skeleton from `world_graph`, starting at `root_entity_id`, and
// terminating at leaf nodes of `world_graph`, or members of `leaf_entity_ids`.
//
// Extracts information about joint and physics parameters from the World data,
// and transforms them into equivalent Skeleton data.
//
// This includes information about
// * Joint types
// * Joint limits
// * Link parameters (mass, center of gravity and inertial tensor)
// * Compatible IK solvers, if any, for chains within the Skeleton
//
// The return value includes
// * A Skeleton object
// * A map from World IDs to the Skeleton ID of the equivalent Element. Note
//   that some World IDs might resolve to the same Skeleton ID (in particular,
//   users can refer to Coordinate Frames either by a Frame's World ID or an
//   Entity's World ID)
absl::StatusOr<SkeletonAndIdMap> SkeletonFromWorldGraph(
    absl::string_view name, const WorldGraph& world_graph,
    const EntityOrFrameId& root_id, bool include_children,
    absl::Span<const world::WorldObject> all_objects) {
  auto skeleton = std::make_unique<kinematics::Skeleton>(name);
  absl::flat_hash_map<std::string, kinematics::ElementId>
      world_id_to_skeleton_id;
  // We use this as a stack for upcoming entities / frames by only pushing
  // to/popping from the back of it.
  std::vector<EntityOrFrameId> stack{root_id};

  absl::flat_hash_set<std::string> object_in_skeleton_ids;
  while (!stack.empty()) {
    // Make a copy of `current` so that we can pop it off the `stack`
    const EntityOrFrameId current = stack.back();
    // Pop here, because we add children to `stack` on the back, so we can't
    // easily find `current` to remove it later
    stack.pop_back();

    // Look up the ID of the Object or Entity that we're at.
    // We use this to look up any children.
    std::string id_for_child_lookup = GetIdForChildLookup(current);
    object_in_skeleton_ids.insert(current.parent_object_id);

    // Handle current element
    INTR_RETURN_IF_ERROR(AddWorldThingToSkeleton(
        world_graph, current, GetIdForChildLookup(root_id), *skeleton,
        world_id_to_skeleton_id));

    // Add children to `stack` - see below for conditions.
    auto children = world_graph.id_to_child_ids.find(id_for_child_lookup);
    if (children != world_graph.id_to_child_ids.end()) {
      for (const EntityOrFrameId& child : children->second) {
        // We want to visit a child entity/frame (i.e. process *its* children)
        // if one of the following is true:
        // * `include_children` is true (in this case, we'll visit *all*
        //   entities below `root_id`)
        // * the child entity's parent_object_id (meaning "ID of the object this
        //   entity or frame belongs to") is the same as that of the entity at
        //   `root_id`.
        if (include_children ||
            child.parent_object_id == root_id.parent_object_id) {
          stack.push_back(child);
        }
      }
    }
  }

  // Create a list of objects that are represented in the output skeleton.
  // Along the way, we do final per-object updates to the Skeleton, i.e. adding
  // IK solvers.
  std::vector<world::WorldObject> objects_in_skeleton;
  for (const world::WorldObject& object : all_objects) {
    if (!object_in_skeleton_ids.contains(object.Proto().id())) {
      continue;
    }
    objects_in_skeleton.push_back(object);
    // Add IK solvers for this object to the Skeleton. We couldn't have done
    // this before, because we need to be sure that the base and tip entities
    // for each solver are already represented in the skeleton.
    for (const ::intrinsic_proto::world::KinematicObjectComponent::IkSolver&
             ik_solver :
         object.Proto().kinematic_object_component().ik_solvers()) {
      // Look up skeleton IDs for base and tip
      auto base_id = world_id_to_skeleton_id.find(ik_solver.base_entity_id());
      if (base_id == world_id_to_skeleton_id.end()) {
        return absl::InternalError(absl::Substitute(
            "World is inconsistent: Object '$0' claims to have an IK "
            "solver from base '$1' to tip '$2', but the base does not "
            "exist.",
            object.Name().value(), ik_solver.base_entity_id(),
            ik_solver.tip_entity_id()));
      }
      auto tip_id = world_id_to_skeleton_id.find(ik_solver.tip_entity_id());
      if (tip_id == world_id_to_skeleton_id.end()) {
        return absl::InternalError(absl::Substitute(
            "World is inconsistent: Object '$0' claims to have an IK "
            "solver from base '$1' to tip '$2', but the tip does not "
            "exist.",
            object.Name().value(), ik_solver.base_entity_id(),
            ik_solver.tip_entity_id()));
      }

      INTR_RETURN_IF_ERROR(skeleton->SetSolverKey(
          base_id->second, tip_id->second, ik_solver.kinematic_solver_key()));
    }
  }

  // Finally, amend the world_id_to_skeleton_id map so that all Frame IDs map to
  // the Skeleton ID for the Entity that the Frame is derived from.
  //
  // Note that we only add frames for objects that are actually in the Skeleton!
  INTR_RETURN_IF_ERROR(
      AddFrameIdsToSkeletonIdMap(objects_in_skeleton, world_id_to_skeleton_id));

  return SkeletonAndIdMap{
      .skeleton = std::move(skeleton),
      .world_id_to_skeleton_id = std::move(world_id_to_skeleton_id),
      .kinematic_objects_in_skeleton = std::move(objects_in_skeleton),
  };
}

absl::StatusOr<SkeletonAndIdMap> GetWholeWorldTreeSkeleton(
    const world::ObjectWorldClient& world_client) {
  INTR_ASSIGN_OR_RETURN(std::vector<world::WorldObject> all_objects,
                        world_client.ListObjects());
  WorldGraph world_graph = BuildWorldGraph(all_objects);

  INTR_ASSIGN_OR_RETURN(auto root, world_client.GetRootObject());

  return SkeletonFromWorldGraph(
      /*name=*/"world",
      /*world_graph=*/world_graph,
      /*root_id=*/
      EntityOrFrameId{
          .parent_object_id = root.Proto().id(),
          .sub_id = EntityIdString{root.Proto().root_entity_id()},
      },
      /*include_children=*/true,
      /*all_objects=*/all_objects);
}

absl::StatusOr<SkeletonAndIdMap> GetSkeletonForObject(
    const world::ObjectWorldClient& world_client, WorldObjectName object_name,
    bool include_children) {
  INTR_ASSIGN_OR_RETURN(std::vector<world::WorldObject> all_objects,
                        world_client.ListObjects());
  WorldGraph world_graph = BuildWorldGraph(all_objects);

  auto object_it =
      absl::c_find_if(all_objects, [&](const world::WorldObject& object) {
        return object.Name() == object_name;
      });
  if (object_it == all_objects.end()) {
    return absl::NotFoundError(absl::StrCat(
        "Cannot find a World object called '", object_name.value(), "'"));
  }
  const world::WorldObject& root_object = *object_it;

  return SkeletonFromWorldGraph(
      /*name=*/"world",
      /*world_graph=*/world_graph,
      /*root_id=*/
      EntityOrFrameId{
          .parent_object_id = root_object.Proto().id(),
          .sub_id = EntityIdString{root_object.Proto().root_entity_id()},
      },
      /*include_children=*/include_children,
      /*all_objects=*/all_objects);
}

}  // namespace intrinsic::icon
