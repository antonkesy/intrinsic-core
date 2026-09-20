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

#include "intrinsic/simulation/gazebo/plugins/grippers/link_articulation_util.h"

#include <optional>
#include <ostream>
#include <string>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "gloop/util/gtl/flat_map.h"
#include "gloop/util/gtl/flat_set.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Types.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/World.hh"
#include "intrinsic/simulation/gazebo/components/articulation_component.h"
#include "intrinsic/util/status/status_macros.h"
#include "sdf/Joint.hh"
#include "sdf/Types.hh"

namespace intrinsic {
namespace simulation {

using gz::sim::Entity;
using gz::sim::kNullEntity;
using gz::sim::components::ChildLinkName;
using gz::sim::components::ContactSensorData;
using gz::sim::components::JointType;
using gz::sim::components::ParentEntity;
using gz::sim::components::ParentLinkName;
using NameComponent = gz::sim::components::Name;
using LinkComponent = gz::sim::components::Link;

namespace {

absl::StatusOr<std::string> EntityName(
    Entity entity, const gz::sim::EntityComponentManager& ecm) {
  std::optional<std::string> name = ecm.ComponentData<NameComponent>(entity);
  if (!name.has_value()) {
    return InvalidArgumentErrorBuilder()
           << "Entity '" << entity << "' has no name.";
  }
  return *name;
}

absl::StatusOr<Entity> GetParentJointEntity(
    Entity link_entity, const std::string& link_name, Entity world_entity,
    const gz::sim::EntityComponentManager& ecm) {
  if (link_entity == kNullEntity) {
    return InvalidArgumentErrorBuilder()
           << "Link '" << link_name << "' cannot be found in ECM.";
  }
  Entity parent_model = ecm.ParentEntity(link_entity);
  if (parent_model == kNullEntity) {
    return InvalidArgumentErrorBuilder()
           << "Link '" << link_name << "' has no parent model.";
  }
  Entity parent_joint = ecm.EntityByComponents(gz::sim::components::Joint(),
                                               ChildLinkName(link_name),
                                               ParentEntity(parent_model));

  // It is possible that the parent joint is defined in an ancestor model. In
  // that case, ChildLinkName will be the relative scoped name of the link.
  // So we incrementally prepend the ancestor model names to search for the
  // parent joint, until we reach the world.
  absl::Cord scoped_link_name{link_name};
  while (parent_joint == kNullEntity) {
    INTR_ASSIGN_OR_RETURN(std::string parent_name,
                          EntityName(parent_model, ecm));
    if (parent_model == world_entity) {
      return kNullEntity;
    }
    // Prepend the parent model name to the scoped link name and continue to
    // search for the parent joint.
    scoped_link_name.Prepend(absl::StrCat(parent_name, sdf::kScopeDelimiter));
    parent_model = ecm.ParentEntity(parent_model);
    if (parent_model == kNullEntity) {
      return InvalidArgumentErrorBuilder()
             << "Model '" << parent_name << "' has no parent.";
    }
    parent_joint =
        ecm.EntityByComponents(gz::sim::components::Joint(),
                               ChildLinkName(std::string(scoped_link_name)),
                               ParentEntity(parent_model));
  }
  return parent_joint;
}

absl::StatusOr<bool> IsMovingJoint(Entity joint_entity,
                                   const gz::sim::EntityComponentManager& ecm) {
  INTR_ASSIGN_OR_RETURN(std::string joint_name, EntityName(joint_entity, ecm));
  std::optional<sdf::JointType> joint_type =
      ecm.ComponentData<JointType>(joint_entity);
  if (!joint_type.has_value()) {
    return InvalidArgumentErrorBuilder()
           << "Joint '" << joint_name << "' has no JointType component.";
  }
  if (joint_type == sdf::JointType::INVALID) {
    return InvalidArgumentErrorBuilder()
           << "Joint '" << joint_name << "' has Invalid type.";
  }
  static constexpr auto kMovingJointTypes =
      gtl::fixed_flat_set_of<sdf::JointType>(
          {sdf::JointType::REVOLUTE, sdf::JointType::REVOLUTE2,
           sdf::JointType::PRISMATIC, sdf::JointType::BALL,
           sdf::JointType::CONTINUOUS, sdf::JointType::GEARBOX,
           sdf::JointType::SCREW, sdf::JointType::UNIVERSAL});
  return kMovingJointTypes.contains(*joint_type);
}
}  // namespace

absl::StatusOr<ArticulationType> GetArticulationType(
    const gz::sim::Link& link, const gz::sim::EntityComponentManager& ecm) {
  // Articulation type is not expected to change at runtime, so if the component
  // is present, return it directly.
  if (std::optional<ArticulationType> articulation_type =
          ecm.ComponentData<Articulation>(link.Entity());
      articulation_type.has_value()) {
    return *articulation_type;
  }

  Entity world_entity = ecm.EntityByComponents(gz::sim::components::World());
  if (world_entity == kNullEntity) {
    return InvalidArgumentErrorBuilder() << "ECM does not have a World entity.";
  }

  std::optional<std::string> link_name = link.Name(ecm);
  if (!link_name.has_value()) {
    return absl::InvalidArgumentError("Link has no name.");
  }
  INTR_ASSIGN_OR_RETURN(
      Entity parent_joint,
      GetParentJointEntity(link.Entity(), *link_name, world_entity, ecm));

  // Walk up the kinematic chain until no parent joint is found for an ancestor
  // link or we reach the world.
  bool has_moving_ancestor_joint = false;
  while (parent_joint != kNullEntity) {
    // Check if the parent joint is moving and if it is, save a flag to indicate
    // that the link has a moving ancestor joint. The articulation type in this
    // case can either be kArticulated or kFloating.
    if (!has_moving_ancestor_joint) {
      INTR_ASSIGN_OR_RETURN(has_moving_ancestor_joint,
                            IsMovingJoint(parent_joint, ecm));
    }

    INTR_ASSIGN_OR_RETURN(std::string joint_name,
                          EntityName(parent_joint, ecm));
    std::optional<std::string> parent_link_name =
        ecm.ComponentData<ParentLinkName>(parent_joint);
    if (!parent_link_name.has_value()) {
      return InvalidArgumentErrorBuilder()
             << "Joint '" << joint_name << "' has no parent link name.";
    }
    // We assume that the parent link is either "world" or is a link in the same
    // model as the joint.
    if (parent_link_name == "world") {
      return has_moving_ancestor_joint ? ArticulationType::kArticulated
                                       : ArticulationType::kFixed;
    }
    if (parent_link_name->find(sdf::kScopeDelimiter) != std::string::npos) {
      return InvalidArgumentErrorBuilder()
             << "Joint '" << joint_name << "' has parent link name '"
             << *parent_link_name << "' with scope separator.";
    }
    Entity joint_parent_model = ecm.ParentEntity(parent_joint);
    if (joint_parent_model == kNullEntity) {
      return InvalidArgumentErrorBuilder()
             << "Joint '" << joint_name << "' has no parent model.";
    }
    Entity parent_link = ecm.EntityByComponents(
        NameComponent(*parent_link_name), LinkComponent(),
        ParentEntity(joint_parent_model));
    if (parent_link == kNullEntity) {
      return InvalidArgumentErrorBuilder()
             << "Link '" << *parent_link_name << "' cannot be found in ECM.";
    }

    // Continue to walk up the kinematic chain.
    INTR_ASSIGN_OR_RETURN(parent_joint,
                          GetParentJointEntity(parent_link, *parent_link_name,
                                               world_entity, ecm));
  }

  // Kinematic chain has no parent joint at the top, so the chain is floating.
  return ArticulationType::kFloating;
}

std::ostream& operator<<(std::ostream& _out,
                         ArticulationType articulation_type) {
  static constexpr auto kArticulationTypeToString =
      gtl::fixed_flat_map_of<ArticulationType, absl::string_view>(
          {{ArticulationType::kFixed, "Fixed"},
           {ArticulationType::kArticulated, "Articulated"},
           {ArticulationType::kFloating, "Floating"}});
  if (kArticulationTypeToString.contains(articulation_type)) {
    _out << kArticulationTypeToString.at(articulation_type);
  } else {
    _out << "Unknown";
  }
  return _out;
}

}  // namespace simulation
}  // namespace intrinsic
