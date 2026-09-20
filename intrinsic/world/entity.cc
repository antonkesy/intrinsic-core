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

#include "intrinsic/world/entity.h"

#include <memory>
#include <set>
#include <string>
#include <utility>

#include "absl/log/check.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "google/protobuf/util/message_differencer.h"
#include "intrinsic/geometry/storage/dummy_storage.h"
#include "intrinsic/geometry/storage/geometry_deserializer.h"
#include "intrinsic/geometry/storage/geometry_serializer.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/component/attachment_component.h"
#include "intrinsic/world/component/collections_component.h"
#include "intrinsic/world/component/collections_member_component.h"
#include "intrinsic/world/component/collision_component.h"
#include "intrinsic/world/component/geometry_component.h"
#include "intrinsic/world/component/kinematics_component.h"
#include "intrinsic/world/component/physics_component.h"
#include "intrinsic/world/component/ppr_component.h"
#include "intrinsic/world/component/projector_component.h"
#include "intrinsic/world/component/robot_component.h"
#include "intrinsic/world/component/sensor_component.h"
#include "intrinsic/world/component/simulation_component.h"
#include "intrinsic/world/component/user_data_component.h"
#include "intrinsic/world/labels.h"
#include "intrinsic/world/proto/attachment_component.pb.h"
#include "intrinsic/world/proto/collections_component.pb.h"
#include "intrinsic/world/proto/collections_member_component.pb.h"
#include "intrinsic/world/proto/collision_component.pb.h"
#include "intrinsic/world/proto/geometry_component.pb.h"
#include "intrinsic/world/proto/kinematics_component.pb.h"
#include "intrinsic/world/proto/physics_component.pb.h"
#include "intrinsic/world/proto/ppr_component.pb.h"
#include "intrinsic/world/proto/robot_component.pb.h"
#include "intrinsic/world/proto/sensor_component.pb.h"
#include "intrinsic/world/proto/user_data_component.pb.h"
#include "intrinsic/world/proto/world_entity.pb.h"
#include "intrinsic/world/world_entity_acl_spec.h"

namespace intrinsic {

WorldEntity::WorldEntity(std::shared_ptr<EntityData> data,
                         const WorldEntityACLSpec& acls)
    : data_(std::move(data)), acls_(acls) {
  CHECK(acls.CanReadEntityData());
}

absl::Status WorldEntity::ValidateWriteAccess() const {
  if (!acls_.CanWriteEntityData()) {
    return absl::PermissionDeniedError(
        "Write access is disabled for this entity");
  }

  return absl::OkStatus();
}

absl::Status WorldEntity::SetLocalName(absl::string_view local_name) {
  INTR_RETURN_IF_ERROR(ValidateWriteAccess());
  data_->local_name = local_name;
  return absl::OkStatus();
}

std::string WorldEntity::GetLocalName() const { return data_->local_name; }

std::string WorldEntity::GetAlias() const { return data_->alias; }

absl::Status WorldEntity::SetAlias(absl::string_view alias) {
  INTR_RETURN_IF_ERROR(ValidateWriteAccess());
  data_->alias = std::string(alias);
  return absl::OkStatus();
}

const std::set<LabelId>& WorldEntity::GetLabels() const {
  return data_->labels;
}

absl::Status WorldEntity::AddLabels(const std::set<LabelId>& labels) {
  INTR_RETURN_IF_ERROR(ValidateWriteAccess());
  data_->labels.insert(labels.begin(), labels.end());
  return absl::OkStatus();
}

absl::Status WorldEntity::RemoveLabels(const std::set<LabelId>& labels) {
  INTR_RETURN_IF_ERROR(ValidateWriteAccess());
  for (const auto& label : labels) {
    data_->labels.erase(label);
  }
  return absl::OkStatus();
}

bool WorldEntity::HasLabel(const LabelId& label) const {
  return data_->labels.count(label) != 0;
}

absl::StatusOr<std::unique_ptr<WorldEntity>> WorldEntity::FromProto(
    const intrinsic_proto::world::internal::WorldEntity& proto,
    const GeometryDeserializer& geolib) {
  auto entity = WorldEntity::Create();
  INTR_RETURN_IF_ERROR(entity->UpdateFromProto(proto, geolib));
  return entity;
}

absl::StatusOr<std::unique_ptr<WorldEntity>> WorldEntity::FromProto(
    const intrinsic_proto::world::internal::WorldEntity& proto) {
  auto entity = WorldEntity::Create();
  INTR_RETURN_IF_ERROR(entity->UpdateFromProto(proto));
  return entity;
}

absl::StatusOr<intrinsic_proto::world::internal::WorldEntity>
WorldEntity::ToProto(GeometrySerializer* geolib) const {
  intrinsic_proto::world::internal::WorldEntity proto;
  proto.set_local_name(data_->local_name);
  proto.set_name(data_->local_name);
  proto.set_alias(data_->alias);

  for (const auto& label : data_->labels) {
    proto.add_labels()->set_name(label.value());
  }

  if (data_->attachment_component != nullptr) {
    INTR_ASSIGN_OR_RETURN(*proto.mutable_attachment_component(),
                          data_->attachment_component->ToProto());
  }

  if (data_->collections_component != nullptr) {
    INTR_ASSIGN_OR_RETURN(*proto.mutable_collections_component(),
                          data_->collections_component->ToProto());
  }

  if (data_->collections_member_component != nullptr) {
    INTR_ASSIGN_OR_RETURN(*proto.mutable_collections_member_component(),
                          data_->collections_member_component->ToProto());
  }

  if (data_->collision_component != nullptr) {
    INTR_ASSIGN_OR_RETURN(*proto.mutable_collision_component(),
                          data_->collision_component->ToProto());
  }

  if (data_->geometry_component != nullptr) {
    if (geolib != nullptr) {
      INTR_ASSIGN_OR_RETURN(*proto.mutable_geometry_component(),
                            data_->geometry_component->ToProto(geolib));
    } else {
      INTR_ASSIGN_OR_RETURN(*proto.mutable_geometry_component(),
                            data_->geometry_component->ToProto());
    }
  }

  if (data_->kinematics_component != nullptr) {
    INTR_ASSIGN_OR_RETURN(*proto.mutable_kinematics_component(),
                          data_->kinematics_component->ToProto());
  }

  if (data_->physics_component != nullptr) {
    INTR_ASSIGN_OR_RETURN(*proto.mutable_physics_component(),
                          data_->physics_component->ToProto());
  }

  if (data_->ppr_component != nullptr) {
    INTR_ASSIGN_OR_RETURN(*proto.mutable_ppr_component(),
                          data_->ppr_component->ToProto());
  }

  if (data_->projector_component != nullptr) {
    INTR_ASSIGN_OR_RETURN(*proto.mutable_projector_component(),
                          data_->projector_component->ToProto());
  }

  if (data_->robot_component != nullptr) {
    INTR_ASSIGN_OR_RETURN(*proto.mutable_robot_component(),
                          data_->robot_component->ToProto());
  }

  if (data_->user_data_component != nullptr) {
    INTR_ASSIGN_OR_RETURN(*proto.mutable_user_data_component(),
                          data_->user_data_component->ToProto());
  }

  if (data_->sensor_component != nullptr) {
    INTR_ASSIGN_OR_RETURN(*proto.mutable_sensor_component(),
                          data_->sensor_component->ToProto());
  }

  if (data_->simulation_component != nullptr) {
    INTR_ASSIGN_OR_RETURN(*proto.mutable_simulation_component(),
                          data_->simulation_component->ToProto());
  }

  return proto;
}

std::unique_ptr<WorldEntity> WorldEntity::Create() {
  return absl::WrapUnique(new WorldEntity(std::make_shared<EntityData>(),
                                          WorldEntityACLSpec::FullAccess()));
}

absl::StatusOr<std::unique_ptr<WorldEntity>> WorldEntity::WrapEntity(
    const WorldEntityACLSpec& spec) const {
  if (!spec.CanReadEntityData()) {
    return absl::InvalidArgumentError(
        "Cannot wrap an entity without read access");
  }

  INTR_RETURN_IF_ERROR(acls_.CanNarrowTo(spec));
  return absl::WrapUnique(new WorldEntity(data_, spec));
}

std::unique_ptr<WorldEntity> WorldEntity::Clone() const {
  auto result = WorldEntity::Create();
  result->data_->local_name = data_->local_name;
  result->data_->alias = data_->alias;
  result->data_->labels = data_->labels;

  if (data_->attachment_component != nullptr) {
    result->data_->attachment_component = data_->attachment_component->Clone();
  }

  if (data_->collections_component != nullptr) {
    result->data_->collections_component =
        data_->collections_component->Clone();
  }

  if (data_->collections_member_component != nullptr) {
    result->data_->collections_member_component =
        data_->collections_member_component->Clone();
  }

  if (data_->collision_component != nullptr) {
    result->data_->collision_component = data_->collision_component->Clone();
  }

  if (data_->geometry_component != nullptr) {
    result->data_->geometry_component = data_->geometry_component->Clone();
  }

  if (data_->kinematics_component != nullptr) {
    result->data_->kinematics_component = data_->kinematics_component->Clone();
  }

  if (data_->physics_component != nullptr) {
    result->data_->physics_component = data_->physics_component->Clone();
  }

  if (data_->robot_component != nullptr) {
    result->data_->robot_component = data_->robot_component->Clone();
  }

  if (data_->ppr_component != nullptr) {
    result->data_->ppr_component = data_->ppr_component->Clone();
  }

  if (data_->projector_component != nullptr) {
    result->data_->projector_component = data_->projector_component->Clone();
  }

  if (data_->user_data_component != nullptr) {
    result->data_->user_data_component = data_->user_data_component->Clone();
  }

  if (data_->sensor_component != nullptr) {
    result->data_->sensor_component = data_->sensor_component->Clone();
  }

  if (data_->simulation_component != nullptr) {
    result->data_->simulation_component = data_->simulation_component->Clone();
  }

  return result;
}

#define UPDATE_COMPONENT_HELPER(field_name, class_name, proto, component)   \
  if (proto.has_##field_name()) {                                           \
    if (component != nullptr) {                                             \
      INTR_RETURN_IF_ERROR(component->UpdateFromProto(proto.field_name())); \
    } else {                                                                \
      INTR_ASSIGN_OR_RETURN(component,                                      \
                            class_name::FromProto(proto.field_name()));     \
    }                                                                       \
  } else {                                                                  \
    component.reset();                                                      \
  }

absl::Status WorldEntity::UpdateFromProtoImpl(
    const intrinsic_proto::world::internal::WorldEntity& proto,
    const GeometryDeserializer* geolib) {
  auto write_access_status = ValidateWriteAccess();
  if (!write_access_status.ok()) {
    INTR_ASSIGN_OR_RETURN(auto current_proto, ToProto());
    std::string differences;
    google::protobuf::util::MessageDifferencer differencer;
    differencer.ReportDifferencesToString(&differences);
    differencer.set_float_comparison(
        google::protobuf::util::MessageDifferencer::APPROXIMATE);

    if (!differencer.Compare(current_proto, proto)) {
      INTR_RETURN_IF_ERROR(write_access_status)
          << "differences: " << differences;
    } else {
      // The protos are the same, no need to update anything.
      return absl::OkStatus();
    }
  }

  data_->local_name = proto.local_name();
  data_->alias = proto.alias();
  data_->labels.clear();

  // Fallback when we had a display name but not a local name.
  if (data_->local_name.empty()) {
    data_->local_name = proto.name();
  }

  for (const auto& label : proto.labels()) {
    data_->labels.emplace(label.name());
  }

  UPDATE_COMPONENT_HELPER(attachment_component, AttachmentComponent, proto,
                          data_->attachment_component);
  UPDATE_COMPONENT_HELPER(collections_component, CollectionsComponent, proto,
                          data_->collections_component);
  UPDATE_COMPONENT_HELPER(collections_member_component,
                          CollectionsMemberComponent, proto,
                          data_->collections_member_component);
  UPDATE_COMPONENT_HELPER(collision_component, CollisionComponent, proto,
                          data_->collision_component);

  // Geometry component has an extra param and thefore we copy the macro here
  if (proto.has_geometry_component()) {
    if (data_->geometry_component != nullptr) {
      if (geolib != nullptr) {
        INTR_RETURN_IF_ERROR(data_->geometry_component->UpdateFromProto(
            proto.geometry_component(), *geolib));
      } else {
        INTR_RETURN_IF_ERROR(data_->geometry_component->UpdateFromProto(
            proto.geometry_component()));
      }
    } else {
      if (geolib != nullptr) {
        INTR_ASSIGN_OR_RETURN(
            data_->geometry_component,
            GeometryComponent::FromProto(proto.geometry_component(), *geolib));
      } else {
        INTR_ASSIGN_OR_RETURN(
            data_->geometry_component,
            GeometryComponent::FromProto(proto.geometry_component()));
      }
    }
  } else {
    data_->geometry_component.reset();
  }

  UPDATE_COMPONENT_HELPER(kinematics_component, KinematicsComponent, proto,
                          data_->kinematics_component);
  UPDATE_COMPONENT_HELPER(physics_component, PhysicsComponent, proto,
                          data_->physics_component);
  UPDATE_COMPONENT_HELPER(robot_component, RobotComponent, proto,
                          data_->robot_component);
  UPDATE_COMPONENT_HELPER(ppr_component, PPRComponent, proto,
                          data_->ppr_component);
  UPDATE_COMPONENT_HELPER(projector_component, ProjectorComponent, proto,
                          data_->projector_component);
  UPDATE_COMPONENT_HELPER(user_data_component, UserDataComponent, proto,
                          data_->user_data_component);
  UPDATE_COMPONENT_HELPER(sensor_component, SensorComponent, proto,
                          data_->sensor_component);
  UPDATE_COMPONENT_HELPER(simulation_component, SimulationComponent, proto,
                          data_->simulation_component);

  return absl::OkStatus();
}

#undef UPDATE_COMPONENT_HELPER

}  // namespace intrinsic
