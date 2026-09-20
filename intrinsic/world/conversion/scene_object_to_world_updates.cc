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

#include "intrinsic/world/conversion/scene_object_to_world_updates.h"

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/scene/proto/v1/scene_object_updates.pb.h"
#include "intrinsic/scene/util/geometry_update_util.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/geometry_types.h"
#include "intrinsic/world/proto/collision_action.pb.h"
#include "intrinsic/world/proto/collision_settings.pb.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"
#include "intrinsic/world/proto/object_world_service.pb.h"
#include "intrinsic/world/proto/object_world_updates.pb.h"
#include "intrinsic/world/proto/user_data.pb.h"

namespace intrinsic::world {

namespace {

using ::intrinsic_proto::scene_object::v1::CartesianLimitsUpdate;
using ::intrinsic_proto::scene_object::v1::CreateFrameUpdate;
using ::intrinsic_proto::scene_object::v1::DeleteEntityUpdate;
using ::intrinsic_proto::scene_object::v1::EntityPoseUpdate;
using ::intrinsic_proto::scene_object::v1::GeometryUpdate;
using ::intrinsic_proto::scene_object::v1::RenameEntityUpdate;
using ::intrinsic_proto::scene_object::v1::SceneObjectInstanceUpdate;
using ::intrinsic_proto::scene_object::v1::SceneObjectInstanceUpdates;
using ::intrinsic_proto::scene_object::v1::SceneObjectUpdate;
using ::intrinsic_proto::scene_object::v1::SceneObjectUpdates;
using ::intrinsic_proto::scene_object::v1::SetIKSolversUpdate;
using ::intrinsic_proto::scene_object::v1::SetNamedConfigurationsUpdate;
using ::intrinsic_proto::scene_object::v1::UpdateCollisionRules;
using ::intrinsic_proto::scene_object::v1::UpdateJointsRequest;
using ::intrinsic_proto::scene_object::v1::UpdateSimulationProperties;
using ::intrinsic_proto::world::ObjectReference;
using ::intrinsic_proto::world::ObjectWorldUpdate;
using ::intrinsic_proto::world::ObjectWorldUpdates;

// Helper to create an ObjectReference by name.
ObjectReference ObjectRefByName(absl::string_view object_name) {
  ObjectReference ref;
  ref.mutable_by_name()->set_object_name(std::string(object_name));
  return ref;
}

// Helper to get joint names in order from a WorldObject.
std::vector<std::string> GetOrderedJointNames(
    const intrinsic_proto::world::Object& target_object) {
  std::vector<std::string> ordered_names;
  const auto& kinematic_comp = target_object.kinematic_object_component();
  for (const auto& joint_id : kinematic_comp.joint_entity_ids()) {
    auto it = target_object.entities().find(joint_id);
    if (it != target_object.entities().end()) {
      ordered_names.push_back(it->second.name());
    }
  }
  return ordered_names;
}

absl::Status ProcessSetNamedConfigurations(
    absl::string_view world_id,
    const intrinsic_proto::world::Object& target_object,
    const SetNamedConfigurationsUpdate& named_update,
    ObjectWorldUpdates& world_updates) {
  // First clear all existing updates that set named joint configs.
  if (named_update.clear_all_named_configurations()) {
    for (auto& update : *world_updates.mutable_updates()) {
      if (!update.has_update_kinematic_object_properties()) {
        continue;
      }

      auto* properties_req =
          update.mutable_update_kinematic_object_properties();
      properties_req->clear_named_joint_configurations_to_set();
      properties_req->clear_named_joint_configurations_to_remove();
    }
  }

  auto* world_update = world_updates.add_updates();
  auto* properties_req =
      world_update->mutable_update_kinematic_object_properties();
  properties_req->set_world_id(std::string(world_id));
  *properties_req->mutable_object() = ObjectRefByName(target_object.name());

  std::vector<std::string> ordered_joint_names =
      GetOrderedJointNames(target_object);

  if (named_update.clear_all_named_configurations()) {
    for (const auto& config : target_object.kinematic_object_component()
                                  .named_joint_configurations()) {
      properties_req->add_named_joint_configurations_to_remove(config.name());
    }
  } else {
    for (const auto& name : named_update.named_configurations_to_remove()) {
      properties_req->add_named_joint_configurations_to_remove(name);
    }
  }
  for (const auto& config : named_update.named_configurations_to_set()) {
    auto* new_config = properties_req->add_named_joint_configurations_to_set();
    new_config->set_name(config.name());
    for (const auto& joint_name : ordered_joint_names) {
      auto it = config.joint_positions().find(joint_name);
      if (it != config.joint_positions().end()) {
        new_config->add_joint_positions(it->second);
      } else {
        new_config->add_joint_positions(0.0);
      }
    }
  }
  return absl::OkStatus();
}

absl::Status ProcessCartesianLimits(
    absl::string_view world_id,
    const intrinsic_proto::world::Object& target_object,
    const CartesianLimitsUpdate& limits_update,
    ObjectWorldUpdates& world_updates) {
  auto* world_update = world_updates.add_updates();
  auto* properties_req =
      world_update->mutable_update_kinematic_object_properties();
  properties_req->set_world_id(std::string(world_id));
  *properties_req->mutable_object() = ObjectRefByName(target_object.name());

  auto* cart_limits = properties_req->mutable_cartesian_limits();
  for (double val : limits_update.min_translational_position())
    cart_limits->add_min_translational_position(val);
  for (double val : limits_update.max_translational_position())
    cart_limits->add_max_translational_position(val);
  for (double val : limits_update.min_translational_velocity())
    cart_limits->add_min_translational_velocity(val);
  for (double val : limits_update.max_translational_velocity())
    cart_limits->add_max_translational_velocity(val);
  for (double val : limits_update.min_translational_acceleration())
    cart_limits->add_min_translational_acceleration(val);
  for (double val : limits_update.max_translational_acceleration())
    cart_limits->add_max_translational_acceleration(val);
  for (double val : limits_update.min_translational_jerk())
    cart_limits->add_min_translational_jerk(val);
  for (double val : limits_update.max_translational_jerk())
    cart_limits->add_max_translational_jerk(val);

  if (limits_update.has_max_rotational_velocity())
    cart_limits->set_max_rotational_velocity(
        limits_update.max_rotational_velocity());
  if (limits_update.has_max_rotational_acceleration())
    cart_limits->set_max_rotational_acceleration(
        limits_update.max_rotational_acceleration());
  if (limits_update.has_max_rotational_jerk())
    cart_limits->set_max_rotational_jerk(limits_update.max_rotational_jerk());
  return absl::OkStatus();
}

absl::Status ProcessUpdateJoints(
    absl::string_view world_id,
    const intrinsic_proto::world::Object& target_object,
    const UpdateJointsRequest& update_joints,
    ObjectWorldUpdates& world_updates) {
  if (update_joints.joint_positions_size() > 0) {
    return absl::UnimplementedError("Joint value updates are not supported.");
  }

  if (update_joints.joint_application_limits().empty() &&
      update_joints.joint_system_limits().empty()) {
    return absl::OkStatus();
  }

  auto* world_update = world_updates.add_updates();
  auto* req = world_update->mutable_update_object_joints();
  req->set_world_id(std::string(world_id));
  *req->mutable_object() = ObjectRefByName(target_object.name());

  std::vector<std::string> ordered_joint_names =
      GetOrderedJointNames(target_object);

  for (const auto& name : ordered_joint_names) {
    req->add_joint_names(name);
  }

  if (!update_joints.joint_application_limits().empty()) {
    for (const auto& name : ordered_joint_names) {
      if (!update_joints.joint_application_limits().contains(name)) {
        return absl::InvalidArgumentError(
            absl::StrCat("Missing joint application limits for joint: ", name));
      }
    }
  }

  if (!update_joints.joint_system_limits().empty()) {
    for (const auto& name : ordered_joint_names) {
      if (!update_joints.joint_system_limits().contains(name)) {
        return absl::InvalidArgumentError(
            absl::StrCat("Missing joint system limits for joint: ", name));
      }
    }
  }

  // If any of the limits would cause the object to enter an invalid state, then
  // throw an error.
  const auto& target_joint_positions =
      target_object.kinematic_object_component().joint_positions();
  for (int i = 0; i < target_joint_positions.size(); ++i) {
    const std::string& joint_name = ordered_joint_names[i];
    if (!update_joints.joint_application_limits().empty()) {
      const auto& joint_application_limit =
          update_joints.joint_application_limits().at(joint_name);

      if ((joint_application_limit.has_max_position() &&
           target_joint_positions.at(i) >
               joint_application_limit.max_position()) ||
          (joint_application_limit.has_min_position() &&
           target_joint_positions.at(i) <
               joint_application_limit.min_position())) {
        return absl::InvalidArgumentError(absl::StrCat(
            "Unable to convert application limits update for joint '",
            joint_name, "' because limit value (", target_joint_positions.at(i),
            ") exceeds new proposed limits [",
            joint_application_limit.min_position(), ", ",
            joint_application_limit.max_position(), "]."));
      }
    }

    if (!update_joints.joint_system_limits().empty()) {
      const auto& joint_system_limit =
          update_joints.joint_system_limits().at(joint_name);

      if ((joint_system_limit.has_max_position() &&
           target_joint_positions.at(i) > joint_system_limit.max_position()) ||
          (joint_system_limit.has_min_position() &&
           target_joint_positions.at(i) < joint_system_limit.min_position())) {
        return absl::InvalidArgumentError(absl::StrCat(
            "Unable to convert system limits update for joint '", joint_name,
            "' because limit value (", target_joint_positions.at(i),
            ") exceeds new proposed limits [",
            joint_system_limit.min_position(), ", ",
            joint_system_limit.max_position(), "]."));
      }
    }
  }

  auto fill_limits =
      [&](const google::protobuf::Map<
              std::string, intrinsic_proto::JointLimitUpdate>& limits_map,
          intrinsic_proto::JointLimitsUpdate* limits_out) -> absl::Status {
#define FILL_FIELD(field_name)                                            \
  do {                                                                    \
    bool has_##field_name = false;                                        \
    for (const auto& name : ordered_joint_names) {                        \
      if (limits_map.at(name).has_##field_name()) {                       \
        has_##field_name = true;                                          \
        break;                                                            \
      }                                                                   \
    }                                                                     \
    if (has_##field_name) {                                               \
      auto* field_ptr = limits_out->mutable_##field_name();               \
      for (const auto& name : ordered_joint_names) {                      \
        const auto& lim = limits_map.at(name);                            \
        if (!lim.has_##field_name()) {                                    \
          return absl::InvalidArgumentError(absl::StrCat(                 \
              "Inconsistent ", #field_name, " in UpdateJointsRequest.")); \
        }                                                                 \
        field_ptr->add_values(lim.field_name());                          \
      }                                                                   \
    }                                                                     \
  } while (0)
    FILL_FIELD(min_position);
    FILL_FIELD(max_position);
    FILL_FIELD(max_velocity);
    FILL_FIELD(max_acceleration);
    FILL_FIELD(max_jerk);
    FILL_FIELD(max_effort);

#undef FILL_FIELD

    return absl::OkStatus();
  };

  if (!update_joints.joint_application_limits().empty()) {
    INTR_RETURN_IF_ERROR(fill_limits(update_joints.joint_application_limits(),
                                     req->mutable_joint_application_limits()));
  }

  if (!update_joints.joint_system_limits().empty()) {
    INTR_RETURN_IF_ERROR(fill_limits(update_joints.joint_system_limits(),
                                     req->mutable_joint_system_limits()));
  } else {
    // !HACK! Set the system limits to be what they already were or infinite,
    // that way we won't enforce limits when setting application limits.
    auto* sys_limits = req->mutable_joint_system_limits();

#define FILL_FIELD(field_name, max_lim_val)                                 \
  do {                                                                      \
    auto* field_ptr = sys_limits->mutable_##field_name();                   \
    for (int i = 0; i < ordered_joint_names.size(); ++i) {                  \
      const auto& lim =                                                     \
          target_object.kinematic_object_component().joint_system_limits(); \
      double lim_val = max_lim_val;                                         \
      if (lim.has_##field_name()) {                                         \
        lim_val = lim.field_name().values().at(i);                          \
      }                                                                     \
      field_ptr->add_values(lim_val);                                       \
    }                                                                       \
  } while (0)

    FILL_FIELD(min_position, -std::numeric_limits<double>::infinity());
    FILL_FIELD(max_position, std::numeric_limits<double>::infinity());
    FILL_FIELD(max_velocity, std::numeric_limits<double>::infinity());
    FILL_FIELD(max_acceleration, std::numeric_limits<double>::infinity());
    FILL_FIELD(max_jerk, std::numeric_limits<double>::infinity());
    FILL_FIELD(max_effort, std::numeric_limits<double>::infinity());

#undef FILL_FIELD
  }

  return absl::OkStatus();
}

bool IsFrame(const intrinsic_proto::world::Object& object,
             absl::string_view name) {
  return absl::c_any_of(
      object.frames(),
      [&](const intrinsic_proto::world::Frame& f) { return f.name() == name; });
}

const intrinsic_proto::world::Entity* FindEntityByName(
    const intrinsic_proto::world::Object& object, absl::string_view name) {
  for (const auto& [_, entity] : object.entities()) {
    if (entity.name() == name) {
      return &entity;
    }
  }
  return nullptr;
}

bool IsLinkOrJoint(const intrinsic_proto::world::Object& object,
                   absl::string_view name) {
  const auto* entity = FindEntityByName(object, name);
  if (entity == nullptr) {
    return false;
  }

  const bool is_link =
      entity->has_geometry_component() && entity->has_physics_component();
  const bool is_joint = entity->has_kinematics_component();
  return is_link || is_joint;
}

absl::Status AddFramePoseUpdate(
    absl::string_view world_id,
    const intrinsic_proto::world::Object& target_object,
    const EntityPoseUpdate& pose_update, ObjectWorldUpdates& world_updates) {
  auto* world_update = world_updates.add_updates();
  auto* req = world_update->mutable_update_transform();
  req->set_world_id(std::string(world_id));

  const auto* entity =
      FindEntityByName(target_object, pose_update.entity_name());
  if (entity == nullptr) {
    return absl::NotFoundError(
        absl::StrCat("Entity not found: ", pose_update.entity_name()));
  }

  const intrinsic_proto::world::Entity* parent_entity = nullptr;
  if (!entity->parent_id().empty()) {
    auto it = target_object.entities().find(entity->parent_id());
    if (it != target_object.entities().end()) {
      parent_entity = &it->second;
    }
  }

  if (parent_entity == nullptr) {
    return absl::NotFoundError(absl::StrCat("Parent entity not found for: ",
                                            pose_update.entity_name()));
  }

  req->mutable_node_b()->mutable_by_name()->mutable_frame()->set_object_name(
      target_object.name());
  req->mutable_node_b()->mutable_by_name()->mutable_frame()->set_frame_name(
      pose_update.entity_name());

  req->mutable_node_to_update()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_object_name(target_object.name());
  req->mutable_node_to_update()
      ->mutable_by_name()
      ->mutable_frame()
      ->set_frame_name(pose_update.entity_name());

  *req->mutable_a_t_b() = pose_update.parent_t_this();

  if (IsFrame(target_object, parent_entity->name())) {
    req->mutable_node_a()->mutable_by_name()->mutable_frame()->set_object_name(
        target_object.name());
    req->mutable_node_a()->mutable_by_name()->mutable_frame()->set_frame_name(
        parent_entity->name());
  } else {
    req->mutable_node_a()->mutable_by_name()->mutable_object()->set_object_name(
        target_object.name());
    req->mutable_node_a_filter()->add_entity_names(parent_entity->name());
  }

  return absl::OkStatus();
}

absl::Status ProcessKinematicEntityPoseUpdates(
    absl::string_view world_id,
    const intrinsic_proto::world::Object& target_object,
    const std::vector<EntityPoseUpdate>& pose_updates,
    ObjectWorldUpdates& world_updates) {
  if (pose_updates.empty()) return absl::OkStatus();

  auto* world_update = world_updates.add_updates();
  auto* properties_req =
      world_update->mutable_update_kinematic_object_properties();
  properties_req->set_world_id(std::string(world_id));
  *properties_req->mutable_object() = ObjectRefByName(target_object.name());

  auto* kinematics_update = properties_req->mutable_robot_kinematics_update();
  auto* entity_updates = kinematics_update->mutable_robot_entity_updates();

  for (const auto& pose_update : pose_updates) {
    auto* entity = entity_updates->add_entities();
    entity->set_entity_name(pose_update.entity_name());
    *entity->mutable_parent_t_entity() = pose_update.parent_t_this();
  }

  return absl::OkStatus();
}

absl::Status ProcessGeometryUpdate(
    absl::string_view world_id,
    const intrinsic_proto::world::Object& target_object,
    const GeometryUpdate& geometry_update, ObjectWorldUpdates& world_updates) {
  const auto* entity =
      FindEntityByName(target_object, geometry_update.entity_name());
  if (entity == nullptr) {
    return absl::NotFoundError(
        absl::StrCat("Entity not found: ", geometry_update.entity_name()));
  }

  if (!entity->has_geometry_component()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Entity ", geometry_update.entity_name(),
                     " does not have a geometry component."));
  }

  intrinsic_proto::world::GeometryComponent updated_geometry_component =
      entity->geometry_component();

  INTR_RETURN_IF_ERROR(::intrinsic::scene_object::ApplyGeometryUpdate(
      geometry_update, updated_geometry_component));

  auto* world_update = world_updates.add_updates();
  auto* req = world_update->mutable_update_entity_properties();
  req->set_world_id(std::string(world_id));

  auto* entity_ref = req->mutable_entity();
  entity_ref->mutable_reference()->mutable_by_name()->set_object_name(
      std::string(target_object.name()));
  entity_ref->mutable_entity_filter()->add_entity_names(
      std::string(geometry_update.entity_name()));

  *req->mutable_geometry_component() = updated_geometry_component;

  return absl::OkStatus();
}
absl::Status ProcessUpdateSimulationProperties(
    absl::string_view world_id,
    const intrinsic_proto::world::Object& target_object,
    const UpdateSimulationProperties& sim_update,
    ObjectWorldUpdates& world_updates) {
  if (!sim_update.has_is_static() && !sim_update.has_is_disabled()) {
    // Neither property is set -- do nothing.
    return absl::OkStatus();
  }

  auto* world_update = world_updates.add_updates();
  auto* req = world_update->mutable_update_object_properties();
  req->set_world_id(std::string(world_id));
  *req->mutable_object() = ObjectRefByName(target_object.name());

  if (target_object.has_object_component() &&
      target_object.object_component().has_simulation_component()) {
    *req->mutable_simulation_component() =
        target_object.object_component().simulation_component();
  }

  if (sim_update.has_is_static()) {
    req->mutable_simulation_component()->set_is_static(sim_update.is_static());
  }
  if (sim_update.has_is_disabled()) {
    req->mutable_simulation_component()->set_disabled(sim_update.is_disabled());
  }
  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<ObjectWorldUpdates> ConvertSceneObjectUpdatesToWorldUpdates(
    absl::string_view world_id,
    const intrinsic_proto::world::Object& target_object,
    const SceneObjectInstanceUpdates& scene_updates) {
  ObjectWorldUpdates world_updates;
  std::vector<EntityPoseUpdate> accumulated_pose_updates;
  UpdateSimulationProperties accumulated_sim_updates;
  bool has_sim_updates = false;

  for (const auto& scene_update : scene_updates.updates()) {
    switch (scene_update.update_case()) {
      case SceneObjectInstanceUpdate::kSetNamedConfigurations:
        INTR_RETURN_IF_ERROR(ProcessSetNamedConfigurations(
            world_id, target_object, scene_update.set_named_configurations(),
            world_updates));
        break;
      case SceneObjectInstanceUpdate::kCartesianLimits:
        INTR_RETURN_IF_ERROR(ProcessCartesianLimits(
            world_id, target_object, scene_update.cartesian_limits(),
            world_updates));
        break;
      case SceneObjectInstanceUpdate::kUpdateJoints:
        INTR_RETURN_IF_ERROR(ProcessUpdateJoints(world_id, target_object,
                                                 scene_update.update_joints(),
                                                 world_updates));
        break;
      case SceneObjectInstanceUpdate::kEntityPose: {
        const auto& pose_update = scene_update.entity_pose();
        std::string root_entity_name;
        if (!target_object.root_entity_id().empty()) {
          auto it =
              target_object.entities().find(target_object.root_entity_id());
          if (it != target_object.entities().end()) {
            root_entity_name = it->second.name();
          }
        }

        if (pose_update.entity_name() == root_entity_name) {
          LOG(WARNING) << "Ignoring root entity pose update for "
                       << pose_update.entity_name() << " during conversion.";
        } else if (IsFrame(target_object, pose_update.entity_name())) {
          INTR_RETURN_IF_ERROR(AddFramePoseUpdate(world_id, target_object,
                                                  pose_update, world_updates));
        } else if (IsLinkOrJoint(target_object, pose_update.entity_name())) {
          if (target_object.type() !=
              intrinsic_proto::world::KINEMATIC_OBJECT) {
            return absl::InvalidArgumentError(absl::StrCat(
                "Unsupported entity pose update for link or joint '",
                pose_update.entity_name(),
                "' on non-kinematic object during conversion."));
          }
          accumulated_pose_updates.push_back(pose_update);
        } else {
          return absl::UnimplementedError(absl::StrCat(
              "Unsupported entity pose update for unknown entity type '",
              pose_update.entity_name(), "' during conversion."));
        }
        break;
      }
      case SceneObjectInstanceUpdate::kUpdateGeometry:
        INTR_RETURN_IF_ERROR(ProcessGeometryUpdate(
            world_id, target_object, scene_update.update_geometry(),
            world_updates));
        break;
      case SceneObjectInstanceUpdate::kUpdateSimulationProperties: {
        const auto& sim_update = scene_update.update_simulation_properties();
        if (sim_update.has_is_static()) {
          accumulated_sim_updates.set_is_static(sim_update.is_static());
          has_sim_updates = true;
        }
        if (sim_update.has_is_disabled()) {
          accumulated_sim_updates.set_is_disabled(sim_update.is_disabled());
          has_sim_updates = true;
        }
        break;
      }
      default:
        return absl::UnimplementedError(absl::StrCat(
            "Unknown instance update type", scene_update.update_case()));
    }
  }

  INTR_RETURN_IF_ERROR(ProcessKinematicEntityPoseUpdates(
      world_id, target_object, accumulated_pose_updates, world_updates));

  if (has_sim_updates) {
    INTR_RETURN_IF_ERROR(ProcessUpdateSimulationProperties(
        world_id, target_object, accumulated_sim_updates, world_updates));
  }

  return world_updates;
}

}  // namespace intrinsic::world
