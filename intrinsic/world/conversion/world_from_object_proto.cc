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

#include "intrinsic/world/conversion/world_from_object_proto.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/collision/util/make_rule_set.h"
#include "intrinsic/world/component/attachment_component.h"
#include "intrinsic/world/component/collections_component.h"
#include "intrinsic/world/component/collections_member_component.h"
#include "intrinsic/world/component/collision_component.h"
#include "intrinsic/world/component/geometry_component.h"
#include "intrinsic/world/component/kinematics_component.h"
#include "intrinsic/world/component/physics_component.h"
#include "intrinsic/world/component/ppr_component.h"
#include "intrinsic/world/component/robot_component.h"
#include "intrinsic/world/component/sensor_component.h"
#include "intrinsic/world/component/simulation_component.h"
#include "intrinsic/world/component/user_data_component.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_creation_utils.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/proto/collision_settings.pb.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"
#include "intrinsic/world/proto/object_world_service.pb.h"
#include "intrinsic/world/proto/robot_component.pb.h"
#include "intrinsic/world/proto/user_data_component.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic::world {

using ::google::protobuf::Map;
using ::google::protobuf::RepeatedPtrField;
using ::intrinsic::AttachmentComponent;
using ::intrinsic::AttachmentEntityId;
using ::intrinsic::CollectionsComponent;
using ::intrinsic::CollectionsEntityId;
using ::intrinsic::CollectionsMemberComponent;
using ::intrinsic::CollectionsMemberEntityId;
using ::intrinsic::CollisionComponent;
using ::intrinsic::EntityId;
using ::intrinsic::GeometryComponent;
using ::intrinsic::KinematicsComponent;
using ::intrinsic::kInvalidEntityId;
using ::intrinsic::kRootEntityId;
using ::intrinsic::MakeRuleSet;
using ::intrinsic::ObjectWorldResourceId;
using ::intrinsic::PhysicsComponent;
using ::intrinsic::Pose3d;
using ::intrinsic::PPRComponent;
using ::intrinsic::RobotCollectionsEntityId;
using ::intrinsic::RobotComponent;
using ::intrinsic::RootObjectId;
using ::intrinsic::RootObjectName;
using ::intrinsic::SensorComponent;
using ::intrinsic::SimulationComponent;
using ::intrinsic::UserDataComponent;
using ::intrinsic::World;
using ::intrinsic::WorldEntity;
using ::intrinsic::WorldHashSet;
using ::intrinsic::eigenmath::VectorXd;
using ::intrinsic::object_world::KinematicObject;
using ::intrinsic::object_world::ObjectWorld;
using ::intrinsic::object_world::ObjectWorldResourceIdToEntityId;
using ::intrinsic::object_world::WorldObject;
using ::intrinsic_proto::FromProtoNormalized;
using ::intrinsic_proto::JointLimits;
using ::intrinsic_proto::RuleSet;
using ::intrinsic_proto::world::CollisionSettings;
using ::intrinsic_proto::world::Entity;
using ::intrinsic_proto::world::EntityReference;
using ::intrinsic_proto::world::Frame;
using ::intrinsic_proto::world::KinematicObjectComponent;
using ::intrinsic_proto::world::ListObjectsResponse;
using ::intrinsic_proto::world::NamedJointConfiguration;
using ::intrinsic_proto::world::Object;
using ::intrinsic_proto::world::ObjectComponent;
using ::intrinsic_proto::world::ObjectOrEntityReference;
using ::intrinsic_proto::world::ObjectReference;
using ::intrinsic_proto::world::ObjectReferenceWithEntityFilter;
using ::intrinsic_proto::world::ObjectType;
using CollectionsComponentProto =
    ::intrinsic_proto::world::CollectionsComponent;
using ObjectEntityFilterProto = ::intrinsic_proto::world::ObjectEntityFilter;
using RobotComponentProto = ::intrinsic_proto::world::RobotComponent;
using UserDataComponentProto = ::intrinsic_proto::world::UserDataComponent;

namespace {

using CollectionMembersMap =
    absl::flat_hash_map<CollectionsComponentProto::CollectionType,
                        std::vector<CollectionsMemberEntityId>>;

// Sorts objects deterministically with ROOT always placed first, followed by
// numeric ResourceId ordering (e.g. ofid_1, ofid_2).
absl::StatusOr<std::vector<const Object*>> SortObjects(
    const RepeatedPtrField<Object>& objects) {
  std::vector<std::pair<EntityId, const Object*>> parsed_objects;
  parsed_objects.reserve(objects.size());
  for (const Object& obj : objects) {
    if (obj.type() == ObjectType::ROOT || obj.id() == RootObjectId().value() ||
        obj.id() == RootObjectName().value()) {
      parsed_objects.emplace_back(kRootEntityId, &obj);
      continue;
    }
    INTR_ASSIGN_OR_RETURN(
        EntityId parsed_id,
        ObjectWorldResourceIdToEntityId(ObjectWorldResourceId(obj.id())));
    parsed_objects.emplace_back(parsed_id, &obj);
  }
  std::sort(parsed_objects.begin(), parsed_objects.end(),
            [](const std::pair<EntityId, const Object*>& a,
               const std::pair<EntityId, const Object*>& b) {
              if (a.first == kRootEntityId && b.first != kRootEntityId) {
                return true;
              }
              if (b.first == kRootEntityId && a.first != kRootEntityId) {
                return false;
              }
              return a.first < b.first;
            });
  std::vector<const Object*> sorted_objects;
  sorted_objects.reserve(parsed_objects.size());
  for (const auto& [_, obj] : parsed_objects) {
    sorted_objects.push_back(obj);
  }
  return sorted_objects;
}

// Sorts frames deterministically based on numeric ResourceId ordering.
absl::StatusOr<std::vector<const Frame*>> SortFrames(
    const RepeatedPtrField<Frame>& frames) {
  std::vector<std::pair<EntityId, const Frame*>> parsed_frames;
  parsed_frames.reserve(frames.size());
  for (const Frame& frame_proto : frames) {
    if (frame_proto.id().empty()) {
      return absl::InvalidArgumentError(
          absl::StrCat("Frame '", frame_proto.name(), "' has an empty id."));
    }
    INTR_ASSIGN_OR_RETURN(EntityId parsed_id,
                          ObjectWorldResourceIdToEntityId(
                              ObjectWorldResourceId(frame_proto.id())));
    parsed_frames.emplace_back(parsed_id, &frame_proto);
  }
  std::sort(parsed_frames.begin(), parsed_frames.end(),
            [](const std::pair<EntityId, const Frame*>& a,
               const std::pair<EntityId, const Frame*>& b) {
              return a.first < b.first;
            });
  std::vector<const Frame*> sorted_frames;
  sorted_frames.reserve(parsed_frames.size());
  for (const auto& [_, frame] : parsed_frames) {
    sorted_frames.push_back(frame);
  }
  return sorted_frames;
}

// Sorts entity map entries deterministically based on numeric ResourceId
// ordering after validating that each entity's id matches its map key.
absl::StatusOr<std::vector<const Entity*>> SortEntities(
    const Map<std::string, Entity>& entities) {
  std::vector<std::pair<EntityId, const Entity*>> parsed_entities;
  parsed_entities.reserve(entities.size());
  for (const auto& [entity_id_str, entity_proto] : entities) {
    if (entity_proto.id().empty()) {
      return absl::InvalidArgumentError(
          absl::StrCat("Entity '", entity_proto.name(), "' has an empty id."));
    }
    if (entity_proto.id() != entity_id_str) {
      return absl::InvalidArgumentError(
          absl::StrCat("Entity key '", entity_id_str,
                       "' does not match entity id '", entity_proto.id(), "'"));
    }
    INTR_ASSIGN_OR_RETURN(EntityId parsed_id,
                          ObjectWorldResourceIdToEntityId(
                              ObjectWorldResourceId(entity_proto.id())));
    parsed_entities.emplace_back(parsed_id, &entity_proto);
  }
  std::sort(parsed_entities.begin(), parsed_entities.end(),
            [](const std::pair<EntityId, const Entity*>& a,
               const std::pair<EntityId, const Entity*>& b) {
              return a.first < b.first;
            });
  std::vector<const Entity*> sorted_entities;
  sorted_entities.reserve(parsed_entities.size());
  for (const auto& [_, ent] : parsed_entities) {
    sorted_entities.push_back(ent);
  }
  return sorted_entities;
}

// Looks up the C++ EntityId corresponding to an ObjectWorld resource ID string.
// Returns a NotFoundError if the entity does not exist in the world.
absl::StatusOr<EntityId> FindEntityId(const World& world,
                                      absl::string_view proto_id) {
  INTR_ASSIGN_OR_RETURN(
      EntityId entity_id,
      ObjectWorldResourceIdToEntityId(ObjectWorldResourceId(proto_id)));
  if (!world.HasEntity(entity_id)) {
    return absl::NotFoundError(
        absl::StrCat("Entity ID not found in world: ", proto_id));
  }
  return entity_id;
}

// Populates an existing C++ WorldEntity from its public Entity proto and
// optional Frame proto, binding it as a member of the object collection.
absl::StatusOr<CollectionsComponentProto::CollectionType> EntityFromProto(
    const Entity& entity_proto, const Frame* frame_proto,
    const Object& object_proto, World& world) {
  if (entity_proto.id().empty()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Entity '", entity_proto.name(), "' has an empty id."));
  }
  INTR_ASSIGN_OR_RETURN(EntityId entity_id,
                        FindEntityId(world, entity_proto.id()));
  INTR_ASSIGN_OR_RETURN(EntityId object_entity_id,
                        FindEntityId(world, object_proto.id()));
  INTR_ASSIGN_OR_RETURN(WorldEntity * entity, world.GetEntityById(entity_id));
  INTR_RETURN_IF_ERROR(entity->SetLocalName(entity_proto.name()));

  // Attachment
  if (!entity_proto.parent_id().empty()) {
    INTR_ASSIGN_OR_RETURN(EntityId parent_entity_world_id,
                          FindEntityId(world, entity_proto.parent_id()));
    Pose3d parent_t_this = Pose3d::Identity();
    if (entity_proto.has_parent_t_this()) {
      INTR_ASSIGN_OR_RETURN(parent_t_this,
                            FromProtoNormalized(entity_proto.parent_t_this()));
    }
    INTR_RETURN_IF_ERROR(world
                             .CreateAttachmentComponent(
                                 AttachmentEntityId(parent_entity_world_id),
                                 entity_id, parent_t_this)
                             .status());
  } else if (frame_proto != nullptr) {
    EntityId parent_entity_world_id = kInvalidEntityId;
    if (!frame_proto->parent_frame().id().empty()) {
      INTR_ASSIGN_OR_RETURN(
          parent_entity_world_id,
          FindEntityId(world, frame_proto->parent_frame().id()));
    } else if (!object_proto.root_entity_id().empty()) {
      INTR_ASSIGN_OR_RETURN(parent_entity_world_id,
                            FindEntityId(world, object_proto.root_entity_id()));
    } else {
      INTR_RET_CHECK_FAIL()
          << "Cannot resolve parent for frame entity " << entity_proto.name()
          << " (neither parent_frame nor object root_entity_id specified).";
    }

    Pose3d parent_t_this = Pose3d::Identity();
    if (frame_proto->has_parent_t_this()) {
      INTR_ASSIGN_OR_RETURN(parent_t_this,
                            FromProtoNormalized(frame_proto->parent_t_this()));
    }
    INTR_RETURN_IF_ERROR(world
                             .CreateAttachmentComponent(
                                 AttachmentEntityId(parent_entity_world_id),
                                 entity_id, parent_t_this)
                             .status());
  }

  // Geometry
  if (entity_proto.has_geometry_component()) {
    INTR_RETURN_IF_ERROR(entity->CreateComponent<GeometryComponent>());
    INTR_ASSIGN_OR_RETURN(GeometryComponent * geo_comp,
                          entity->GetComponent<GeometryComponent>());
    INTR_RETURN_IF_ERROR(
        geo_comp->UpdateFromProto(entity_proto.geometry_component()));
  }

  // Physics
  if (entity_proto.has_physics_component()) {
    INTR_RETURN_IF_ERROR(entity->CreateComponentFromProto<PhysicsComponent>(
        entity_proto.physics_component()));
  }

  // Kinematics (Joint)
  if (entity_proto.has_kinematics_component()) {
    INTR_RETURN_IF_ERROR(entity->CreateComponentFromProto<KinematicsComponent>(
        entity_proto.kinematics_component()));
  }

  // Sensor
  if (entity_proto.has_sensor_component()) {
    INTR_RETURN_IF_ERROR(entity->CreateComponentFromProto<SensorComponent>(
        entity_proto.sensor_component()));
  }

  // PPR
  if (entity_proto.has_ppr_component()) {
    INTR_RETURN_IF_ERROR(entity->CreateComponent<PPRComponent>());
    INTR_ASSIGN_OR_RETURN(PPRComponent * ppr_comp,
                          entity->GetComponent<PPRComponent>());
    INTR_RETURN_IF_ERROR(
        ppr_comp->UpdateFromProto(entity_proto.ppr_component()));
  }

  // Determine collection type
  CollectionsComponentProto::CollectionType col_type;
  if (entity_proto.has_kinematics_component()) {
    col_type = CollectionsComponentProto::COLLECTION_TYPE_JOINTS;
  } else if (entity_proto.has_sensor_component()) {
    col_type = CollectionsComponentProto::COLLECTION_TYPE_SENSORS;
  } else if (frame_proto != nullptr) {
    col_type =
        frame_proto->is_attachment_frame()
            ? CollectionsComponentProto::COLLECTION_TYPE_ATTACHMENT_FRAMES
            : CollectionsComponentProto::COLLECTION_TYPE_COORDINATE_FRAMES;
  } else {
    // If not a joint, sensor, or frame, treat as a link (e.g. dummy/massless
    // link)
    col_type = CollectionsComponentProto::COLLECTION_TYPE_LINKS;
  }

  if (col_type == CollectionsComponentProto::COLLECTION_TYPE_LINKS) {
    // Ensure all 5 components required by LinkEntityId are present
    if (!entity_proto.has_geometry_component()) {
      INTR_RETURN_IF_ERROR(entity->CreateComponent<GeometryComponent>());
    }
    INTR_RETURN_IF_ERROR(entity->CreateComponent<CollisionComponent>());
    if (!entity_proto.has_physics_component()) {
      INTR_RETURN_IF_ERROR(entity->CreateComponent<PhysicsComponent>());
    }
  }

  INTR_RETURN_IF_ERROR(entity->CreateComponent<CollectionsMemberComponent>());
  INTR_ASSIGN_OR_RETURN(CollectionsMemberComponent * member_comp,
                        entity->GetComponent<CollectionsMemberComponent>());
  INTR_RETURN_IF_ERROR(member_comp->AddParentCollection(
      CollectionsEntityId(object_entity_id), col_type));

  return col_type;
}

// Populates a standalone Frame entity (present in object.frames() but not in
// object.entities()).
absl::StatusOr<CollectionsComponentProto::CollectionType>
EntityFromStandaloneFrame(const Frame& frame_proto, const Object& object_proto,
                          World& world) {
  if (frame_proto.id().empty()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Frame '", frame_proto.name(), "' has an empty id."));
  }
  INTR_ASSIGN_OR_RETURN(EntityId frame_entity_id,
                        FindEntityId(world, frame_proto.id()));
  INTR_ASSIGN_OR_RETURN(EntityId object_entity_id,
                        FindEntityId(world, object_proto.id()));
  INTR_ASSIGN_OR_RETURN(WorldEntity * frame_entity,
                        world.GetEntityById(frame_entity_id));
  INTR_RETURN_IF_ERROR(frame_entity->SetLocalName(frame_proto.name()));

  EntityId parent_entity_world_id = kInvalidEntityId;
  if (!frame_proto.parent_frame().id().empty()) {
    INTR_ASSIGN_OR_RETURN(parent_entity_world_id,
                          FindEntityId(world, frame_proto.parent_frame().id()));
  } else if (!object_proto.root_entity_id().empty()) {
    INTR_ASSIGN_OR_RETURN(parent_entity_world_id,
                          FindEntityId(world, object_proto.root_entity_id()));
  } else {
    INTR_RET_CHECK_FAIL()
        << "Cannot resolve parent for standalone frame " << frame_proto.name()
        << " (neither parent_frame nor object root_entity_id specified).";
  }

  Pose3d parent_t_this = Pose3d::Identity();
  if (frame_proto.has_parent_t_this()) {
    INTR_ASSIGN_OR_RETURN(parent_t_this,
                          FromProtoNormalized(frame_proto.parent_t_this()));
  }
  INTR_RETURN_IF_ERROR(
      world
          .CreateAttachmentComponent(AttachmentEntityId(parent_entity_world_id),
                                     frame_entity_id, parent_t_this)
          .status());

  CollectionsComponentProto::CollectionType col_type =
      frame_proto.is_attachment_frame()
          ? CollectionsComponentProto::COLLECTION_TYPE_ATTACHMENT_FRAMES
          : CollectionsComponentProto::COLLECTION_TYPE_COORDINATE_FRAMES;

  INTR_RETURN_IF_ERROR(
      frame_entity->CreateComponent<CollectionsMemberComponent>());
  INTR_ASSIGN_OR_RETURN(
      CollectionsMemberComponent * member_comp,
      frame_entity->GetComponent<CollectionsMemberComponent>());
  INTR_RETURN_IF_ERROR(member_comp->AddParentCollection(
      CollectionsEntityId(object_entity_id), col_type));

  return col_type;
}

// Populates object-level components (PPR, Simulation, Gripper, UserData) on the
// collection parent entity from the ObjectComponent proto.
absl::Status ObjectComponentFromProto(const ObjectComponent& proto,
                                      World& world,
                                      EntityId object_entity_world_id) {
  INTR_ASSIGN_OR_RETURN(WorldEntity * object_entity,
                        world.GetEntityById(object_entity_world_id));

  // PPR
  if (proto.has_ppr_component()) {
    INTR_RETURN_IF_ERROR(object_entity->CreateComponent<PPRComponent>());
    INTR_ASSIGN_OR_RETURN(PPRComponent * ppr_comp,
                          object_entity->GetComponent<PPRComponent>());
    INTR_RETURN_IF_ERROR(ppr_comp->UpdateFromProto(proto.ppr_component()));
  }

  // Simulation
  if (proto.has_simulation_component()) {
    INTR_RETURN_IF_ERROR(object_entity->CreateComponent<SimulationComponent>());
    INTR_ASSIGN_OR_RETURN(SimulationComponent * sim_comp,
                          object_entity->GetComponent<SimulationComponent>());
    INTR_RETURN_IF_ERROR(
        sim_comp->UpdateFromProto(proto.simulation_component()));
  }

  // User Data
  if (!proto.user_data().empty()) {
    INTR_RETURN_IF_ERROR(object_entity->CreateComponent<UserDataComponent>());
    INTR_ASSIGN_OR_RETURN(UserDataComponent * user_data_comp,
                          object_entity->GetComponent<UserDataComponent>());
    UserDataComponentProto user_data_proto;
    *user_data_proto.mutable_user_data_protos() = proto.user_data();
    INTR_RETURN_IF_ERROR(user_data_comp->UpdateFromProto(user_data_proto));
  }

  return absl::OkStatus();
}

// Populates coordinate and attachment frame entities defined directly under the
// ROOT object and configures their attachments relative to the root entity or
// other root frames.
absl::Status PopulateRootObject(const Object& root_object_proto, World& world) {
  for (const Frame& frame_proto : root_object_proto.frames()) {
    INTR_ASSIGN_OR_RETURN(EntityId frame_entity_world_id,
                          FindEntityId(world, frame_proto.id()));
    INTR_ASSIGN_OR_RETURN(WorldEntity * frame_entity,
                          world.GetEntityById(frame_entity_world_id));
    INTR_RETURN_IF_ERROR(frame_entity->SetLocalName(frame_proto.name()));

    // Attachment
    EntityId parent_entity_world_id = kRootEntityId;
    if (!frame_proto.parent_frame().id().empty()) {
      INTR_ASSIGN_OR_RETURN(
          parent_entity_world_id,
          FindEntityId(world, frame_proto.parent_frame().id()));
    }
    Pose3d parent_t_this = Pose3d::Identity();
    if (frame_proto.has_parent_t_this()) {
      INTR_ASSIGN_OR_RETURN(parent_t_this,
                            FromProtoNormalized(frame_proto.parent_t_this()));
    }
    INTR_RETURN_IF_ERROR(world
                             .CreateAttachmentComponent(
                                 AttachmentEntityId(parent_entity_world_id),
                                 frame_entity_world_id, parent_t_this)
                             .status());
  }
  return absl::OkStatus();
}

// Configures the RobotComponent on a kinematic object entity (IK solvers, named
// joint configurations, cartesian limits, payload, joint positions, and joint
// limits).
absl::Status PopulateRobotComponent(const Object& object_proto, World& world,
                                    EntityId object_entity_world_id,
                                    RobotComponent* robot_comp) {
  RobotComponentProto internal_robot_proto;
  const KinematicObjectComponent& kinematic_comp =
      object_proto.kinematic_object_component();

  for (const KinematicObjectComponent::IkSolver& solver :
       kinematic_comp.ik_solvers()) {
    RobotComponentProto::IkSolver* internal_solver =
        internal_robot_proto.add_ik_solvers();
    internal_solver->set_kinematic_solver_key(solver.kinematic_solver_key());
    if (!solver.base_entity_id().empty() && solver.base_entity_id() != "0") {
      INTR_ASSIGN_OR_RETURN(EntityId base_id,
                            FindEntityId(world, solver.base_entity_id()));
      internal_solver->set_base_uid(base_id.value());
    } else {
      internal_solver->set_base_uid(kInvalidEntityId.value());
    }
    if (!solver.tip_entity_id().empty() && solver.tip_entity_id() != "0") {
      INTR_ASSIGN_OR_RETURN(EntityId tip_id,
                            FindEntityId(world, solver.tip_entity_id()));
      internal_solver->set_tip_uid(tip_id.value());
    } else {
      internal_solver->set_tip_uid(kInvalidEntityId.value());
    }
  }

  for (const NamedJointConfiguration& config :
       kinematic_comp.named_joint_configurations()) {
    RobotComponentProto::NamedDofConfiguration* internal_config =
        internal_robot_proto.add_named_dof_configurations();
    internal_config->set_name(config.name());
    for (double val : config.joint_positions()) {
      internal_config->add_dof_values(val);
    }
  }

  if (kinematic_comp.has_cartesian_limits()) {
    *internal_robot_proto.mutable_cartesian_limits() =
        kinematic_comp.cartesian_limits();
  }

  if (kinematic_comp.has_mounted_payload()) {
    *internal_robot_proto.mutable_mounted_payload() =
        kinematic_comp.mounted_payload();
  }

  internal_robot_proto.set_are_kinematics_updated(
      kinematic_comp.are_kinematics_updated());

  INTR_RETURN_IF_ERROR(robot_comp->UpdateFromProto(internal_robot_proto));

  const RepeatedPtrField<std::string>& joint_entity_ids =
      kinematic_comp.joint_entity_ids();
  const google::protobuf::RepeatedField<double>& joint_positions =
      kinematic_comp.joint_positions();

  for (int i = 0; i < joint_entity_ids.size(); ++i) {
    INTR_ASSIGN_OR_RETURN(EntityId joint_entity_world_id,
                          FindEntityId(world, joint_entity_ids[i]));
    INTR_ASSIGN_OR_RETURN(KinematicsComponent * kinematics,
                          world.GetComponentByEntityId<KinematicsComponent>(
                              joint_entity_world_id));

    if (i < joint_positions.size()) {
      kinematics->SetRawValue(joint_positions[i]);
    }

    // Initialize targets with current values
    std::pair<double, double> current_system_fixed =
        kinematics->GetSystemRawValueFixedLimits();
    double target_system_fixed_lower = current_system_fixed.first;
    double target_system_fixed_upper = current_system_fixed.second;
    double target_system_vel = kinematics->GetSystemVelocityLimit();
    double target_system_acc = kinematics->GetSystemAccelerationLimit();
    double target_system_jerk = kinematics->GetSystemJerkLimit();
    double target_system_effort = kinematics->GetSystemEffortLimit();

    std::pair<double, double> current_app_fixed =
        kinematics->GetApplicationRawValueFixedLimits();
    double target_app_fixed_lower = current_app_fixed.first;
    double target_app_fixed_upper = current_app_fixed.second;
    double target_app_vel = kinematics->GetApplicationVelocityLimit();
    double target_app_acc = kinematics->GetApplicationAccelerationLimit();
    double target_app_jerk = kinematics->GetApplicationJerkLimit();
    double target_app_effort = kinematics->GetApplicationEffortLimit();

    bool has_system = kinematic_comp.has_joint_system_limits();
    bool has_app = kinematic_comp.has_joint_application_limits();

    if (has_system) {
      const JointLimits& limits = kinematic_comp.joint_system_limits();
      if (limits.has_min_position() &&
          limits.min_position().values_size() > i) {
        target_system_fixed_lower = limits.min_position().values(i);
      }
      if (limits.has_max_position() &&
          limits.max_position().values_size() > i) {
        target_system_fixed_upper = limits.max_position().values(i);
      }
      if (limits.has_max_velocity() &&
          limits.max_velocity().values_size() > i) {
        target_system_vel = limits.max_velocity().values(i);
      }
      if (limits.has_max_acceleration() &&
          limits.max_acceleration().values_size() > i) {
        target_system_acc = limits.max_acceleration().values(i);
      }
      if (limits.has_max_jerk() && limits.max_jerk().values_size() > i) {
        target_system_jerk = limits.max_jerk().values(i);
      }
      if (limits.has_max_effort() && limits.max_effort().values_size() > i) {
        target_system_effort = limits.max_effort().values(i);
      }
    }

    if (has_app) {
      const JointLimits& limits = kinematic_comp.joint_application_limits();
      if (limits.has_min_position() &&
          limits.min_position().values_size() > i) {
        target_app_fixed_lower = limits.min_position().values(i);
      }
      if (limits.has_max_position() &&
          limits.max_position().values_size() > i) {
        target_app_fixed_upper = limits.max_position().values(i);
      }
      if (limits.has_max_velocity() &&
          limits.max_velocity().values_size() > i) {
        target_app_vel = limits.max_velocity().values(i);
      }
      if (limits.has_max_acceleration() &&
          limits.max_acceleration().values_size() > i) {
        target_app_acc = limits.max_acceleration().values(i);
      }
      if (limits.has_max_jerk() && limits.max_jerk().values_size() > i) {
        target_app_jerk = limits.max_jerk().values(i);
      }
      if (limits.has_max_effort() && limits.max_effort().values_size() > i) {
        target_app_effort = limits.max_effort().values(i);
      }
    }

    // Apply system limits
    if (has_system) {
      INTR_RETURN_IF_ERROR(kinematics->SetSystemRawValueFixedLimits(
          target_system_fixed_lower, target_system_fixed_upper,
          /*enforce_limits=*/false));
      INTR_RETURN_IF_ERROR(kinematics->SetSystemVelocityLimit(
          target_system_vel, /*enforce_limits=*/false));
      INTR_RETURN_IF_ERROR(kinematics->SetSystemAccelerationLimit(
          target_system_acc, /*enforce_limits=*/false));
      INTR_RETURN_IF_ERROR(kinematics->SetSystemJerkLimit(
          target_system_jerk, /*enforce_limits=*/false));
      INTR_RETURN_IF_ERROR(kinematics->SetSystemEffortLimit(
          target_system_effort, /*enforce_limits=*/false));
    }

    // Apply application limits
    if (has_app) {
      INTR_RETURN_IF_ERROR(kinematics->SetApplicationRawValueFixedLimits(
          target_app_fixed_lower, target_app_fixed_upper,
          /*enforce_limits=*/false));
      INTR_RETURN_IF_ERROR(kinematics->SetApplicationVelocityLimit(
          target_app_vel, /*enforce_limits=*/false));
      INTR_RETURN_IF_ERROR(kinematics->SetApplicationAccelerationLimit(
          target_app_acc, /*enforce_limits=*/false));
      INTR_RETURN_IF_ERROR(kinematics->SetApplicationJerkLimit(
          target_app_jerk, /*enforce_limits=*/false));
      INTR_RETURN_IF_ERROR(kinematics->SetApplicationEffortLimit(
          target_app_effort, /*enforce_limits=*/false));
    }
  }
  return absl::OkStatus();
}

// 1. Sets names, aliases, and creates base collection/robot components on the
// object's collection entity.
absl::Status InitializeCollectionEntity(const Object& object_proto,
                                        EntityId object_entity_world_id,
                                        World& world) {
  INTR_ASSIGN_OR_RETURN(WorldEntity * object_entity,
                        world.GetEntityById(object_entity_world_id));
  INTR_RETURN_IF_ERROR(object_entity->SetLocalName(object_proto.name()));
  if (object_proto.name_is_global_alias()) {
    INTR_RETURN_IF_ERROR(object_entity->SetAlias(object_proto.name()));
  }
  INTR_RETURN_IF_ERROR(object_entity->CreateComponent<CollectionsComponent>());
  if (object_proto.type() == ObjectType::KINEMATIC_OBJECT) {
    INTR_RETURN_IF_ERROR(object_entity->CreateComponent<RobotComponent>());
  }
  return absl::OkStatus();
}

// 2. Populates all child entities (links, joints, sensors, and frames) and
// returns the collection members map.
absl::StatusOr<CollectionMembersMap> PopulateChildEntities(
    const Object& object_proto, World& world) {
  CollectionMembersMap local_collection_members;
  INTR_ASSIGN_OR_RETURN(std::vector<const Frame*> sorted_frames,
                        SortFrames(object_proto.frames()));

  // Map each entity ID to its corresponding frame proto (if any).
  absl::flat_hash_map<EntityId, const Frame*> frame_by_entity_id;
  for (const Frame* frame_proto : sorted_frames) {
    INTR_ASSIGN_OR_RETURN(EntityId frame_entity_world_id,
                          ObjectWorldResourceIdToEntityId(
                              ObjectWorldResourceId(frame_proto->id())));
    frame_by_entity_id[frame_entity_world_id] = frame_proto;
  }

  // 1. Populate all entities from object_proto.entities()
  INTR_ASSIGN_OR_RETURN(std::vector<const Entity*> sorted_entities,
                        SortEntities(object_proto.entities()));
  for (const Entity* entity_proto : sorted_entities) {
    INTR_ASSIGN_OR_RETURN(EntityId entity_id,
                          FindEntityId(world, entity_proto->id()));
    auto frame_it = frame_by_entity_id.find(entity_id);
    const Frame* frame_proto =
        (frame_it != frame_by_entity_id.end()) ? frame_it->second : nullptr;

    INTR_ASSIGN_OR_RETURN(
        CollectionsComponentProto::CollectionType col_type,
        EntityFromProto(*entity_proto, frame_proto, object_proto, world));
    local_collection_members[col_type].push_back(
        CollectionsMemberEntityId(entity_id));
  }

  // 2. Populate any standalone frames that were not in object_proto.entities()
  for (const Frame* frame_proto : sorted_frames) {
    INTR_ASSIGN_OR_RETURN(EntityId frame_entity_id,
                          FindEntityId(world, frame_proto->id()));
    INTR_ASSIGN_OR_RETURN(WorldEntity * frame_entity,
                          world.GetEntityById(frame_entity_id));
    if (frame_entity->HasComponent<CollectionsMemberComponent>()) {
      continue;
    }
    INTR_ASSIGN_OR_RETURN(
        CollectionsComponentProto::CollectionType col_type,
        EntityFromStandaloneFrame(*frame_proto, object_proto, world));
    local_collection_members[col_type].push_back(
        CollectionsMemberEntityId(frame_entity_id));
  }

  return local_collection_members;
}

// 3. Preserves base link and joint ordering in the collection members.
absl::StatusOr<CollectionMembersMap> OrderCollectionMembers(
    const Object& object_proto, const World& world,
    CollectionMembersMap collection_members) {
  // Preserve base link ordering
  if (!object_proto.root_entity_id().empty()) {
    INTR_ASSIGN_OR_RETURN(EntityId root_entity_world_id,
                          FindEntityId(world, object_proto.root_entity_id()));
    CollectionsMemberEntityId root_member_id(root_entity_world_id);
    std::vector<CollectionsMemberEntityId>& links =
        collection_members[CollectionsComponent::kLinks];
    auto it = std::find(links.begin(), links.end(), root_member_id);
    if (it != links.end()) {
      std::rotate(links.begin(), it, it + 1);
    }
  }

  // Preserve joint ordering
  if (object_proto.type() == ObjectType::KINEMATIC_OBJECT &&
      object_proto.has_kinematic_object_component()) {
    const KinematicObjectComponent& kinematic_comp =
        object_proto.kinematic_object_component();
    std::vector<CollectionsMemberEntityId>& joints =
        collection_members[CollectionsComponent::kJoints];
    std::vector<CollectionsMemberEntityId> ordered_joints;
    ordered_joints.reserve(joints.size());
    WorldHashSet<CollectionsMemberEntityId> added_joints;

    for (const std::string& joint_proto_id :
         kinematic_comp.joint_entity_ids()) {
      INTR_ASSIGN_OR_RETURN(EntityId joint_entity_world_id,
                            FindEntityId(world, joint_proto_id));
      CollectionsMemberEntityId member_id(joint_entity_world_id);
      ordered_joints.push_back(member_id);
      added_joints.insert(member_id);
    }

    for (const CollectionsMemberEntityId& joint_id : joints) {
      if (!added_joints.contains(joint_id)) {
        ordered_joints.push_back(joint_id);
        added_joints.insert(joint_id);
      }
    }
    joints = std::move(ordered_joints);
  }
  return collection_members;
}

// 4. Finalizes collection memberships, RobotComponent, and ObjectComponent on
// the collection entity.
absl::Status FinalizeObjectComponents(
    const Object& object_proto, EntityId object_entity_world_id, World& world,
    const CollectionMembersMap& collection_members) {
  INTR_ASSIGN_OR_RETURN(WorldEntity * object_entity,
                        world.GetEntityById(object_entity_world_id));
  INTR_ASSIGN_OR_RETURN(CollectionsComponent * col_comp,
                        object_entity->GetComponent<CollectionsComponent>());
  for (const auto& [type, members] : collection_members) {
    INTR_RETURN_IF_ERROR(col_comp->SetCollectionMembers(type, members));
  }

  if (object_proto.type() == ObjectType::KINEMATIC_OBJECT) {
    INTR_ASSIGN_OR_RETURN(RobotComponent * robot_comp,
                          object_entity->GetComponent<RobotComponent>());
    INTR_RETURN_IF_ERROR(PopulateRobotComponent(
        object_proto, world, object_entity_world_id, robot_comp));
  }

  if (object_proto.has_object_component()) {
    INTR_RETURN_IF_ERROR(ObjectComponentFromProto(
        object_proto.object_component(), world, object_entity_world_id));
  }

  return absl::OkStatus();
}

// Populates all entities and components for an individual Object in the World:
// 1. Sets names, aliases, and CollectionsComponent on the collection entity.
// 2. Populates all child entities (links, joints, sensors, frames) and returns
// their collection memberships.
// 3. Orders collection members (base link first, kinematic joint ordering).
// 4. Finalizes collection members, RobotComponent, and ObjectComponent.
absl::Status PopulateObjectEntities(const Object& object_proto, World& world) {
  if (object_proto.type() == ObjectType::ROOT) {
    return PopulateRootObject(object_proto, world);
  }

  INTR_ASSIGN_OR_RETURN(EntityId object_entity_world_id,
                        FindEntityId(world, object_proto.id()));

  INTR_RETURN_IF_ERROR(
      InitializeCollectionEntity(object_proto, object_entity_world_id, world));

  INTR_ASSIGN_OR_RETURN(CollectionMembersMap unsorted_collection_members,
                        PopulateChildEntities(object_proto, world));

  INTR_ASSIGN_OR_RETURN(
      CollectionMembersMap sorted_collection_members,
      OrderCollectionMembers(object_proto, world,
                             std::move(unsorted_collection_members)));

  INTR_RETURN_IF_ERROR(FinalizeObjectComponents(
      object_proto, object_entity_world_id, world, sorted_collection_members));

  return absl::OkStatus();
}

// Converts public CollisionSettings into a RuleSet and applies it as the
// default rule set on the C++ World using MakeRuleSet.
absl::Status PopulateCollisionSettings(
    const CollisionSettings& collision_settings,
    const ObjectWorld& object_world, World& world) {
  INTR_ASSIGN_OR_RETURN(RuleSet rule_set,
                        MakeRuleSet(collision_settings, object_world));
  return world.SetDefaultRuleSet(rule_set);
}

// Creates external attachment relationships (reparenting) between each non-root
// object's root entity and its designated parent entity in the World.
absl::Status PopulateExternalAttachments(
    const RepeatedPtrField<Object>& objects, World& world) {
  absl::flat_hash_map<std::string, const Object*> object_proto_map;
  for (const Object& obj : objects) {
    object_proto_map[obj.id()] = &obj;
  }
  for (const Object& obj : objects) {
    if (obj.type() == ObjectType::ROOT) {
      continue;
    }

    // Resolve parent entity ID
    EntityId parent_entity_world_id = kRootEntityId;
    if (obj.has_parent_entity() && !obj.parent_entity().id().empty()) {
      INTR_ASSIGN_OR_RETURN(parent_entity_world_id,
                            FindEntityId(world, obj.parent_entity().id()));
    } else if (obj.has_parent() && !obj.parent().id().empty()) {
      auto parent_proto_it = object_proto_map.find(obj.parent().id());
      if (parent_proto_it == object_proto_map.end()) {
        return absl::NotFoundError(
            absl::StrCat("Parent object ", obj.parent().id(),
                         " not found for object ", obj.name()));
      }
      if (!parent_proto_it->second->root_entity_id().empty()) {
        INTR_ASSIGN_OR_RETURN(
            parent_entity_world_id,
            FindEntityId(world, parent_proto_it->second->root_entity_id()));
      } else {
        INTR_ASSIGN_OR_RETURN(parent_entity_world_id,
                              FindEntityId(world, obj.parent().id()));
      }
    }

    // Resolve child root entity ID
    absl::string_view root_entity_proto_id = obj.root_entity_id();
    if (!root_entity_proto_id.empty()) {
      INTR_ASSIGN_OR_RETURN(EntityId child_root_entity_world_id,
                            FindEntityId(world, root_entity_proto_id));

      INTR_ASSIGN_OR_RETURN(WorldEntity * child_root_entity,
                            world.GetEntityById(child_root_entity_world_id));
      if (!child_root_entity->HasComponent<AttachmentComponent>()) {
        Pose3d parent_t_this = Pose3d::Identity();
        if (obj.has_object_component() &&
            obj.object_component().has_parent_t_this()) {
          INTR_ASSIGN_OR_RETURN(
              parent_t_this,
              FromProtoNormalized(obj.object_component().parent_t_this()));
        }
        INTR_RETURN_IF_ERROR(world
                                 .CreateAttachmentComponent(
                                     AttachmentEntityId(parent_entity_world_id),
                                     child_root_entity_world_id, parent_t_this)
                                 .status());
      }
    }
  }
  return absl::OkStatus();
}

// Helper to create an entity preserving the ID parsed from the string
// resource ID, or returning the existing entity if already created.
absl::StatusOr<EntityId> CreateEntityFromResourceId(World& world,
                                                    absl::string_view id_str) {
  INTR_ASSIGN_OR_RETURN(EntityId parsed_id, ObjectWorldResourceIdToEntityId(
                                                ObjectWorldResourceId(id_str)));
  if (parsed_id == kRootEntityId || world.HasEntity(parsed_id)) {
    return parsed_id;
  }
  return world.CreateEntityWithId(parsed_id);
}

// Pre-allocates entity IDs for the ROOT object and all its root frames.
absl::Status AllocateRootObjectEntities(const Object& root_object_proto,
                                        World& world) {
  INTR_ASSIGN_OR_RETURN(std::vector<const Frame*> sorted_frames,
                        SortFrames(root_object_proto.frames()));
  for (const Frame* frame_proto : sorted_frames) {
    INTR_RETURN_IF_ERROR(
        CreateEntityFromResourceId(world, frame_proto->id()).status());
  }
  return absl::OkStatus();
}

// Pre-allocates entity IDs for a non-root Object (object entity, child
// entities, and frames).
absl::Status AllocateObjectEntities(const Object& object_proto, World& world) {
  INTR_RETURN_IF_ERROR(
      CreateEntityFromResourceId(world, object_proto.id()).status());

  INTR_ASSIGN_OR_RETURN(std::vector<const Entity*> sorted_entities,
                        SortEntities(object_proto.entities()));
  for (const Entity* entity_proto : sorted_entities) {
    INTR_RETURN_IF_ERROR(
        CreateEntityFromResourceId(world, entity_proto->id()).status());
  }

  INTR_ASSIGN_OR_RETURN(std::vector<const Frame*> sorted_frames,
                        SortFrames(object_proto.frames()));
  for (const Frame* frame_proto : sorted_frames) {
    INTR_RETURN_IF_ERROR(
        CreateEntityFromResourceId(world, frame_proto->id()).status());
  }
  return absl::OkStatus();
}

// Pass 0: Pre-allocates all objects, child entities, and frames across all
// objects so that all entity IDs exist before building components and
// attachments. Objects, entities, and frames are sorted deterministically so
// that C++ entity IDs are assigned in the exact same order regardless of
// protobuf ordering or network serialization.
absl::Status PreallocateEntities(const RepeatedPtrField<Object>& objects,
                                 World& world) {
  INTR_ASSIGN_OR_RETURN(std::vector<const Object*> sorted_objects,
                        SortObjects(objects));
  bool root_allocated = false;
  for (const Object* obj : sorted_objects) {
    if (obj->type() == ObjectType::ROOT ||
        obj->id() == RootObjectId().value() ||
        obj->id() == RootObjectName().value()) {
      if (root_allocated) {
        return absl::InvalidArgumentError(
            "Multiple ROOT objects found in ListObjectsResponse");
      }
      root_allocated = true;
      INTR_RETURN_IF_ERROR(AllocateRootObjectEntities(*obj, world));
      continue;
    }
    INTR_RETURN_IF_ERROR(AllocateObjectEntities(*obj, world));
  }
  return absl::OkStatus();
}

// Constructs a C++ World from repeated Object protos and CollisionSettings
// through sequential passes (self-contained object population, external
// attachments, and collision settings).
absl::StatusOr<World> ConstructEntityWorld(
    const RepeatedPtrField<Object>& objects,
    const CollisionSettings& collision_settings) {
  World world = World::CreateEmptyWorld();

  // Pass 0: Pre-allocate all objects, child entities, and frames so that all
  // IDs exist across all objects.
  INTR_RETURN_IF_ERROR(PreallocateEntities(objects, world));

  // Pass 1: Populate each object self-contained (entities, frames, components,
  // collections)
  for (const Object& obj : objects) {
    INTR_RETURN_IF_ERROR(PopulateObjectEntities(obj, world));
  }

  // Pass 2: External attachments (Reparenting across objects)
  INTR_RETURN_IF_ERROR(PopulateExternalAttachments(objects, world));

  INTR_ASSIGN_OR_RETURN(std::unique_ptr<ObjectWorld> object_world,
                        ObjectWorld::CreateView(world));

  // Pass 3: Collision Settings (cross-object / world-level)
  INTR_RETURN_IF_ERROR(
      PopulateCollisionSettings(collision_settings, *object_world, world));

  for (const WorldObject* obj : object_world->GetObjects()) {
    absl::StatusOr<KinematicObject*> kin_obj =
        object_world->GetKinematicObject(obj->GetId());
    if (kin_obj.ok()) {
      INTR_ASSIGN_OR_RETURN(VectorXd joint_positions,
                            (*kin_obj)->GetJointPositions());
      INTR_RETURN_IF_ERROR(
          (*kin_obj)->SetJointPositions(joint_positions,
                                        /*enforce_limits=*/false));
    }
  }

  return world;
}

}  // namespace

absl::StatusOr<World> CreateEntityWorldFromProto(
    const ListObjectsResponse& response,
    const CollisionSettings& collision_settings) {
  return ConstructEntityWorld(response.objects(), collision_settings);
}

}  // namespace intrinsic::world
