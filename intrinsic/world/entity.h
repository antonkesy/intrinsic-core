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

#ifndef INTRINSIC_WORLD_ENTITY_H_
#define INTRINSIC_WORLD_ENTITY_H_

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/geometry/storage/geometry_deserializer.h"
#include "intrinsic/geometry/storage/geometry_serializer.h"
#include "intrinsic/util/status/status_builder.h"
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
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/labels.h"
#include "intrinsic/world/proto/physics_component.pb.h"
#include "intrinsic/world/proto/sensor_component.pb.h"
#include "intrinsic/world/proto/world_entity.pb.h"
#include "intrinsic/world/world_entity_acl_spec.h"

namespace intrinsic {

class WorldEntity {
 public:
  // Copy and move are disabled because we have Clone/Create.
  WorldEntity(WorldEntity&& other) = default;
  WorldEntity(const WorldEntity& other) = delete;
  WorldEntity operator=(WorldEntity&& other) = delete;
  WorldEntity operator=(const WorldEntity& other) = delete;

  // Returns a new WorldEntity instance with no components.
  static std::unique_ptr<WorldEntity> Create();

  // Make a copy of the entity and all of the components within.
  std::unique_ptr<WorldEntity> Clone() const;

  // Makes a referenced version of the entity and all of the components within.
  // The returned instance will have the restrictions specified by the given
  // spec and maintain the reference
  absl::StatusOr<std::unique_ptr<WorldEntity>> WrapEntity(
      const WorldEntityACLSpec& spec) const;

  // Returns the local name associated with this entity. Can be used for
  // searching through parantage.
  std::string GetLocalName() const;

  // Sets the local name associated with this entity.
  absl::Status SetLocalName(absl::string_view local_name);

  // Returns the alias associated with this entity.
  std::string GetAlias() const;

  // Sets the alias associated with this entity. The alias must be unique within
  // the world that contains this entity. Calling this will not guarantee that
  // no other entity has the same alias.
  absl::Status SetAlias(absl::string_view alias);

  // Returns the set of labels associated with this entity.
  const std::set<LabelId>& GetLabels() const;

  // Adds the new labels to this entity. Duplicates are ignore as the labels are
  // treated like a set.
  absl::Status AddLabels(const std::set<LabelId>& labels);

  // Removes the labels from this entity. If the label is not part of the entity
  // we ignore it.
  absl::Status RemoveLabels(const std::set<LabelId>& labels);

  // Returns true if the entity contains the given label.
  bool HasLabel(const LabelId& label) const;

  // Validates this entity based on the given templates. Returns false if the
  // entity does not contain all of the required templated components.
  //
  // The set template pack ComponentTypes can be one of two alternatives:
  // - A pack of component types like AttachmentComponentType or
  //   CollisionComponentType.
  // - A single parameter of type TypedEntityId<T...> where T is a set of
  //   component types.
  template <typename... ComponentTypes>
  bool IsEntityValid() const;

  // Validates this entity based on the given templates. Returns an error if the
  // entity does not contain all of the required templated components.
  //
  // The set template pack ComponentTypes can be one of two alternatives:
  // - A pack of component types like AttachmentComponentType or
  //   CollisionComponentType.
  // - A single parameter of type TypedEntityId<T...> where T is a set of
  //   component types.
  // This facilitates being able to do things like:
  // INTR_RETURN_IF_ERROR(entity->ValidateEntity<AttachmentComponentType>());
  // INTR_RETURN_IF_ERROR(entity->ValidateEntity<PhysicalEntityId>());
  template <typename... ComponentTypes>
  absl::Status ValidateEntity() const;

  // Returns an Entity parsed from the given proto or an error.
  ABSL_DEPRECATED("Deserialize geometry separately from entities")
  static absl::StatusOr<std::unique_ptr<WorldEntity>> FromProto(
      const intrinsic_proto::world::internal::WorldEntity& proto,
      const GeometryDeserializer& geolib);

  // Returns an Entity parsed from the given proto or an error.
  static absl::StatusOr<std::unique_ptr<WorldEntity>> FromProto(
      const intrinsic_proto::world::internal::WorldEntity& proto);

  // Returns a proto representation of this entity or an error.
  ABSL_DEPRECATED("Serialize geometry separately from entities")
  absl::StatusOr<intrinsic_proto::world::internal::WorldEntity> ToProto(
      GeometrySerializer* geolib) const;

  // Returns a proto representation of this entity or an error.
  absl::StatusOr<intrinsic_proto::world::internal::WorldEntity> ToProto()
      const {
    return ToProto(nullptr);
  }

  // Returns true if the component exists for this entity.
  template <typename Component>
  bool HasComponent() const;

  // Returns the component if it exists or creates a new one and returns it if
  // it's not part of this entity.
  template <typename Component>
  absl::StatusOr<Component*> GetOrCreateComponent();

  // Set the entity's component. Destroys the previous component if it exists.
  template <typename Component>
  absl::Status SetComponent(std::unique_ptr<Component> component);

  // Returns the component if it exists or NotFoundError if it's not part of
  // this entity.
  template <typename Component>
  absl::StatusOr<Component*> GetComponent();

  // Returns the component if it exists or NotFoundError if it's not part of
  // this entity.
  template <typename Component>
  absl::StatusOr<const Component*> GetComponent() const;

  // Creates the component for this entity. Returns an error if the component
  // already exists.
  template <typename Component>
  absl::Status CreateComponent();

  // Creates the component for this entity using the given proto. Returns an
  // error if the component already exists.
  template <typename Component, typename ComponentProto>
  absl::Status CreateComponentFromProto(const ComponentProto& input_proto);

  // Creates the component for this entity. Returns an error if the component
  // already exists. In general, you should use CreateComponentsByType() instead
  // because it is more flexible. (This function's primary purpose is to bridge
  // the gap between Components and ComponentTypes.)
  template <typename ComponentType>
  absl::Status CreateComponentByType();

  // Attempts to add components to this entity based on the given templates.
  // Returns an error and leaves the entity unchanged if the entity already
  // contains any of the templated components.
  //
  // The set template pack ComponentTypes can be one of two alternatives:
  // - A pack of component types like AttachmentComponentType or
  //   CollisionComponentType.
  // - A single parameter of type TypedEntityId<T...> where T is a set of
  //   component types.
  // This facilitates being able to do things like:
  // INTR_RETURN_IF_ERROR(entity->CreateComponentsByType<AttachmentComponentType>());
  // INTR_RETURN_IF_ERROR(entity->CreateComponentsByType<PhysicalEntityId>());
  template <typename... ComponentTypes>
  absl::Status CreateComponentsByType();

  // Removes the component for this entity if it exists.
  template <typename Component>
  absl::Status RemoveComponent();

  // Updates the entity based on the given proto. This will do a full
  // override, if something is missing from this proto it will override any
  // existing values with the defaults.
  ABSL_DEPRECATED("Deserialize geometry separately from entities")
  absl::Status UpdateFromProto(
      const intrinsic_proto::world::internal::WorldEntity& proto,
      const GeometryDeserializer& geolib) {
    return UpdateFromProtoImpl(proto, &geolib);
  }

  // Updates the entity based on the given proto. This will do a full
  // override, if something is missing from this proto it will override any
  // existing values with the defaults.
  absl::Status UpdateFromProto(
      const intrinsic_proto::world::internal::WorldEntity& proto) {
    return UpdateFromProtoImpl(proto, nullptr);
  }

 private:
  absl::Status UpdateFromProtoImpl(
      const intrinsic_proto::world::internal::WorldEntity& proto,
      const GeometryDeserializer* geolib);

  // Returns ok when write access is enabled, permission error otherwise.
  absl::Status ValidateWriteAccess() const;

  // EntityData contains the actual internal state of the WorldEntity instance.
  struct EntityData {
    std::set<LabelId> labels;
    std::string local_name;
    std::string alias;

    std::unique_ptr<AttachmentComponent> attachment_component;
    std::unique_ptr<CollectionsComponent> collections_component;
    std::unique_ptr<CollectionsMemberComponent> collections_member_component;
    std::unique_ptr<CollisionComponent> collision_component;
    std::unique_ptr<GeometryComponent> geometry_component;
    std::unique_ptr<KinematicsComponent> kinematics_component;
    std::unique_ptr<PhysicsComponent> physics_component;
    std::unique_ptr<PPRComponent> ppr_component;
    std::unique_ptr<ProjectorComponent> projector_component;
    std::unique_ptr<RobotComponent> robot_component;
    std::unique_ptr<UserDataComponent> user_data_component;
    std::unique_ptr<SensorComponent> sensor_component;
    std::unique_ptr<SimulationComponent> simulation_component;
  };

  explicit WorldEntity(std::shared_ptr<EntityData> data,
                       const WorldEntityACLSpec& acls);

  std::shared_ptr<EntityData> data_;
  WorldEntityACLSpec acls_ = WorldEntityACLSpec::FullAccess();
};

namespace world_entity_details {

template <typename Component>
bool HasComponent(const WorldEntity& entity);

}  // namespace world_entity_details

#define INTRINSIC_SPECIALIZE_HAS_COMPONENT(Name, VarName)           \
  template <>                                                       \
  inline bool WorldEntity::HasComponent<Name>() const {             \
    return data_->VarName != nullptr;                               \
  }                                                                 \
                                                                    \
  namespace world_entity_details {                                  \
  template <>                                                       \
  inline bool HasComponent<Name##Type>(const WorldEntity& entity) { \
    return entity.HasComponent<Name>();                             \
  }                                                                 \
  }

INTRINSIC_SPECIALIZE_HAS_COMPONENT(AttachmentComponent, attachment_component);
INTRINSIC_SPECIALIZE_HAS_COMPONENT(CollectionsComponent, collections_component);
INTRINSIC_SPECIALIZE_HAS_COMPONENT(CollectionsMemberComponent,
                                   collections_member_component);
INTRINSIC_SPECIALIZE_HAS_COMPONENT(CollisionComponent, collision_component);
INTRINSIC_SPECIALIZE_HAS_COMPONENT(GeometryComponent, geometry_component);
INTRINSIC_SPECIALIZE_HAS_COMPONENT(KinematicsComponent, kinematics_component);
INTRINSIC_SPECIALIZE_HAS_COMPONENT(PhysicsComponent, physics_component);
INTRINSIC_SPECIALIZE_HAS_COMPONENT(PPRComponent, ppr_component);
INTRINSIC_SPECIALIZE_HAS_COMPONENT(ProjectorComponent, projector_component);
INTRINSIC_SPECIALIZE_HAS_COMPONENT(RobotComponent, robot_component);
INTRINSIC_SPECIALIZE_HAS_COMPONENT(UserDataComponent, user_data_component);
INTRINSIC_SPECIALIZE_HAS_COMPONENT(SensorComponent, sensor_component);
INTRINSIC_SPECIALIZE_HAS_COMPONENT(SimulationComponent, simulation_component);

#undef INTRINSIC_SPECIALIZE_HAS_COMPONENT

namespace world_entity_details {

// in order to support templating `ValidateEntity` on both `ComponentTypes`
// and `TypedEntityId<ComponentTypes...>`, we need to use a struct with
// partial template specialization to resolve the potential nested typenames
template <typename... ComponentTypes>
struct ValidateEntityImpl {
  static bool Get(const WorldEntity& entity) {
    return (HasComponent<ComponentTypes>(entity) && ...);
  }
};

template <typename... ComponentTypes>
struct ValidateEntityImpl<TypedEntityId<ComponentTypes...>> {
  static bool Get(const WorldEntity& entity) {
    return (HasComponent<ComponentTypes>(entity) && ...);
  }
};

// in order to support templating `CreateComponentsByType` on both
// `ComponentTypes` and `TypedEntityId<ComponentTypes...>`, we need to use a
// struct with partial template specialization to resolve the potential nested
// typenames
template <typename... ComponentTypes>
struct CreateComponentsByTypeImpl {
  static absl::Status CreateComponentsByType(WorldEntity* entity) {
    bool has_any_component = (HasComponent<ComponentTypes>(*entity) || ...);
    if (has_any_component) {
      return absl::AlreadyExistsError(
          "Entity already has one or more of the specified components");
    }

    std::vector<absl::Status> statuses = {
        entity->CreateComponentByType<ComponentTypes>()...};
    for (const auto& status : statuses) {
      INTR_RETURN_IF_ERROR(status);
    }
    return absl::OkStatus();
  }
};

template <typename... ComponentTypes>
struct CreateComponentsByTypeImpl<TypedEntityId<ComponentTypes...>> {
  static absl::Status CreateComponentsByType(WorldEntity* entity) {
    bool has_any_component = (HasComponent<ComponentTypes>(*entity) || ...);
    if (has_any_component) {
      return absl::AlreadyExistsError(
          "Entity already has one or more of the specified components");
    }

    std::vector<absl::Status> statuses = {
        entity->CreateComponentByType<ComponentTypes>()...};
    for (const auto& status : statuses) {
      INTR_RETURN_IF_ERROR(status);
    }
    return absl::OkStatus();
  }
};

// Helper for Extracting the TypedEntityId<...> from a set of template
// params that can be either component pack or a single TypedEntityId.
template <typename... Types>
struct TypedEntity {
  using Result = TypedEntityId<Types...>;
};

template <typename... Types>
struct TypedEntity<TypedEntityId<Types...>> {
  using Result = TypedEntityId<Types...>;
};

template <typename... Types>
using TypedResult = typename TypedEntity<Types...>::Result;

}  // namespace world_entity_details

#define INTRINSIC_SPECIALIZE_COMPONENT_ACCESS(ComponentName, VarName)      \
  template <>                                                              \
  inline absl::StatusOr<ComponentName*>                                    \
  WorldEntity::GetComponent<ComponentName>() {                             \
    if (data_->VarName == nullptr) {                                       \
      return intrinsic::NotFoundErrorBuilder()                             \
             << "Could not find the " << #ComponentName << " for entity '" \
             << data_->local_name << "'";                                  \
    }                                                                      \
    return data_->VarName.get();                                           \
  }                                                                        \
                                                                           \
  template <>                                                              \
  inline absl::StatusOr<const ComponentName*>                              \
  WorldEntity::GetComponent<ComponentName>() const {                       \
    if (data_->VarName == nullptr) {                                       \
      return intrinsic::NotFoundErrorBuilder()                             \
             << "Could not find the " << #ComponentName << " for entity '" \
             << data_->local_name << "'";                                  \
    }                                                                      \
    return data_->VarName.get();                                           \
  }                                                                        \
                                                                           \
  template <>                                                              \
  inline absl::Status WorldEntity::CreateComponent<ComponentName>() {      \
    INTR_RETURN_IF_ERROR(ValidateWriteAccess());                           \
    if (data_->VarName != nullptr) {                                       \
      return absl::AlreadyExistsError(#ComponentName " already exists");   \
    }                                                                      \
    data_->VarName = ComponentName::Create();                              \
    return absl::OkStatus();                                               \
  }                                                                        \
                                                                           \
  template <>                                                              \
  inline absl::Status                                                      \
  WorldEntity::CreateComponentByType<ComponentName##Type>() {              \
    return CreateComponent<ComponentName>();                               \
  }                                                                        \
                                                                           \
  template <>                                                              \
  inline absl::StatusOr<ComponentName*>                                    \
  WorldEntity::GetOrCreateComponent<ComponentName>() {                     \
    if (data_->VarName == nullptr) {                                       \
      INTR_RETURN_IF_ERROR(CreateComponent<ComponentName>());              \
    }                                                                      \
    return data_->VarName.get();                                           \
  }                                                                        \
                                                                           \
  template <>                                                              \
  inline absl::Status WorldEntity::SetComponent<ComponentName>(            \
      std::unique_ptr<ComponentName> component) {                          \
    data_->VarName = std::move(component);                                 \
    return absl::OkStatus();                                               \
  }                                                                        \
                                                                           \
  template <>                                                              \
  inline absl::Status WorldEntity::RemoveComponent<ComponentName>() {      \
    INTR_RETURN_IF_ERROR(ValidateWriteAccess());                           \
    data_->VarName.reset();                                                \
    return absl::OkStatus();                                               \
  }

INTRINSIC_SPECIALIZE_COMPONENT_ACCESS(AttachmentComponent,
                                      attachment_component);
INTRINSIC_SPECIALIZE_COMPONENT_ACCESS(CollectionsComponent,
                                      collections_component);
INTRINSIC_SPECIALIZE_COMPONENT_ACCESS(CollectionsMemberComponent,
                                      collections_member_component);
INTRINSIC_SPECIALIZE_COMPONENT_ACCESS(CollisionComponent, collision_component);
INTRINSIC_SPECIALIZE_COMPONENT_ACCESS(GeometryComponent, geometry_component);
INTRINSIC_SPECIALIZE_COMPONENT_ACCESS(KinematicsComponent,
                                      kinematics_component);
INTRINSIC_SPECIALIZE_COMPONENT_ACCESS(PhysicsComponent, physics_component);
INTRINSIC_SPECIALIZE_COMPONENT_ACCESS(PPRComponent, ppr_component);
INTRINSIC_SPECIALIZE_COMPONENT_ACCESS(ProjectorComponent, projector_component);
INTRINSIC_SPECIALIZE_COMPONENT_ACCESS(RobotComponent, robot_component);
INTRINSIC_SPECIALIZE_COMPONENT_ACCESS(UserDataComponent, user_data_component);
INTRINSIC_SPECIALIZE_COMPONENT_ACCESS(SensorComponent, sensor_component);
INTRINSIC_SPECIALIZE_COMPONENT_ACCESS(SimulationComponent,
                                      simulation_component);

#undef INTRINSIC_SPECIALIZE_COMPONENT_ACCESS

#define INTRINSIC_SPECIALIZE_COMPONENT_FROM_PROTO(ComponentName, VarName) \
  template <>                                                             \
  inline absl::Status WorldEntity::CreateComponentFromProto<              \
      ComponentName, intrinsic_proto::world::ComponentName>(              \
      const intrinsic_proto::world::ComponentName& input_proto) {         \
    if (data_->VarName != nullptr) {                                      \
      return absl::AlreadyExistsError(#ComponentName " already exists");  \
    }                                                                     \
    INTR_ASSIGN_OR_RETURN(data_->VarName,                                 \
                          ComponentName::FromProto(input_proto));         \
    return absl::OkStatus();                                              \
  }

INTRINSIC_SPECIALIZE_COMPONENT_FROM_PROTO(SensorComponent, sensor_component);
INTRINSIC_SPECIALIZE_COMPONENT_FROM_PROTO(GeometryComponent,
                                          geometry_component);
INTRINSIC_SPECIALIZE_COMPONENT_FROM_PROTO(PhysicsComponent, physics_component);
INTRINSIC_SPECIALIZE_COMPONENT_FROM_PROTO(ProjectorComponent,
                                          projector_component);
INTRINSIC_SPECIALIZE_COMPONENT_FROM_PROTO(KinematicsComponent,
                                          kinematics_component);

#undef INTRINSIC_SPECIALIZE_COMPONENT_FROM_PROTO

template <typename... ComponentTypes>
bool WorldEntity::IsEntityValid() const {
  return world_entity_details::ValidateEntityImpl<ComponentTypes...>::Get(
      *this);
}

template <typename... ComponentTypes>
absl::Status WorldEntity::ValidateEntity() const {
  if (!IsEntityValid<ComponentTypes...>()) {
    return absl::InvalidArgumentError(
        "Entity does not have all of the required components");
  }

  return absl::OkStatus();
}

template <typename... ComponentTypes>
absl::Status WorldEntity::CreateComponentsByType() {
  return world_entity_details::CreateComponentsByTypeImpl<
      ComponentTypes...>::CreateComponentsByType(this);
}

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_ENTITY_H_
