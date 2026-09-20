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

#include "intrinsic/simulation/gazebo/plugins/joint_device_util.h"

#include <cstddef>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "gz/math/Inertial.hh"
#include "gz/math/Matrix3.hh"
#include "gz/math/Pose3.hh"
#include "gz/math/Vector3.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointForce.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/JointVelocityReset.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "intrinsic/simulation/gazebo/plugins/util.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "sdf/Joint.hh"
#include "sdf/Types.hh"

namespace intrinsic {
namespace simulation {

using ::gz::sim::EntityComponentManager;
using ::gz::sim::kNullEntity;
using ::gz::sim::Model;
using ::gz::sim::components::ChildLinkName;
using ::gz::sim::components::JointForce;
using ::gz::sim::components::JointForceCmd;
using ::gz::sim::components::JointPosition;
using ::gz::sim::components::JointPositionReset;
using ::gz::sim::components::JointVelocity;
using ::gz::sim::components::JointVelocityCmd;
using ::gz::sim::components::JointVelocityReset;
using ::gz::sim::components::Name;
using ::gz::sim::components::ParentLinkName;
using ::gz::sim::components::Pose;
using SdfJointType = ::sdf::JointType;

namespace {
template <class ComponentType>
void SetComponentDataSize(ComponentType& component, uint joint_dof_count,
                          EntityComponentManager& ecm) {
  component.Data().resize(joint_dof_count);
}

template <class ComponentType>
std::vector<double>& GetComponentVectorData(gz::sim::Entity joint_entity,
                                            EntityComponentManager& ecm) {
  DCHECK(ecm.EntityHasComponentType(joint_entity, ComponentType().TypeId()))
      << ComponentType().TypeId() << " component has not been initialized for"
      << " entity " << joint_entity;

  return ecm.Component<ComponentType>(joint_entity)->Data();
}

template <class ComponentType>
const std::vector<double>& GetComponentVectorData(
    gz::sim::Entity joint_entity, const EntityComponentManager& ecm) {
  return GetComponentVectorData<ComponentType>(joint_entity, ecm);
}

template <class ComponentType>
void CreateVectorDataComponent(gz::sim::Entity joint_entity,
                               uint joint_dof_count,
                               EntityComponentManager& ecm) {
  if (!ecm.EntityHasComponentType(joint_entity, ComponentType().TypeId())) {
    auto& component = *(ecm.CreateComponent(joint_entity, ComponentType()));
    SetComponentDataSize<ComponentType>(component, joint_dof_count, ecm);
  } else {
    DCHECK(GetComponentVectorData<ComponentType>(joint_entity, ecm).size() ==
           joint_dof_count)
        << ComponentType().TypeId() << " component present for entity "
        << joint_entity << " already but with different dof count.";
  }
}

struct LinkVisitInfo {
  gz::sim::Entity link_entity = kNullEntity;
  gz::sim::Entity parent_model_entity = kNullEntity;
  gz::sim::Entity parent_joint_entity = kNullEntity;
  gz::math::Pose3d base_t_link;
  gz::math::Pose3d base_t_parent_model;
};
typedef std::function<absl::Status(LinkVisitInfo link_info,
                                   const EntityComponentManager& ecm)>
    LinkVisitorFunction;

// Traverses the attachment sub-tree of the given joint entity. The visitor
// function is called for each descendent link.
absl::Status TraverseDescendentLinkTree(gz::sim::Entity joint_entity,
                                        absl::string_view scope_separator,
                                        const EntityComponentManager& ecm,
                                        const LinkVisitorFunction& visit_link) {
  std::deque<LinkVisitInfo> child_link_queue;
  {
    // Initialize descendent link queue for traversal.
    std::string child_link_name =
        ecm.Component<ChildLinkName>(joint_entity)->Data();
    Model model(ecm.ParentEntity(joint_entity));
    auto child_link_entity = model.LinkByName(ecm, child_link_name);
    if (child_link_entity == kNullEntity) {
      return intrinsic::UnimplementedErrorBuilder()
             << "Descendent link traversal is not yet supported for "
                "a joint whose child link is defined in a nested model.";
    }
    gz::math::Pose3d base_t_link =
        ecm.Component<gz::sim::components::Pose>(child_link_entity)->Data();
    child_link_queue.push_back({.link_entity = child_link_entity,
                                .parent_model_entity = model.Entity(),
                                .parent_joint_entity = joint_entity,
                                .base_t_link = base_t_link});
  }

  // Breadth-first traversal of the descendent link tree.
  while (!child_link_queue.empty()) {
    const LinkVisitInfo link_info = child_link_queue.front();
    const gz::sim::Entity link_entity = link_info.link_entity;
    child_link_queue.pop_front();

    INTR_RETURN_IF_ERROR(visit_link(link_info, ecm));

    const std::string link_name = ecm.Component<Name>(link_entity)->Data();
    const gz::sim::Entity link_model_entity = link_info.parent_model_entity;
    for (const gz::sim::Entity child_joint_entity :
         Model(link_model_entity).Joints(ecm)) {
      // Disregard joints that don't attach to the current link.
      const std::string test_parent_link_name =
          ecm.Component<ParentLinkName>(child_joint_entity)->Data();
      if (test_parent_link_name != link_name) {
        continue;
      }

      std::string child_link_name =
          ecm.Component<ChildLinkName>(child_joint_entity)->Data();
      // Child link might be under a nested model
      size_t index = child_link_name.find(scope_separator);
      Model child_link_model(link_model_entity);
      gz::math::Pose3d base_t_child_link_model = link_info.base_t_parent_model;
      while (index != std::string::npos) {
        const std::string child_model_name = child_link_name.substr(0, index);
        const gz::sim::Entity child_model_entity =
            child_link_model.ModelByName(ecm, child_model_name);
        if (child_model_entity == kNullEntity) {
          return intrinsic::InternalErrorBuilder()
                 << "Could not resolve nested model: " << child_model_name;
        }
        child_link_model = Model(child_model_entity);
        child_link_name =
            child_link_name.substr(index + scope_separator.size());
        index = child_link_name.find(scope_separator);
        base_t_child_link_model =
            base_t_child_link_model *
            ecm.Component<Pose>(child_model_entity)->Data();
      }
      const gz::sim::Entity child_link_entity =
          child_link_model.LinkByName(ecm, child_link_name);
      if (child_link_entity != kNullEntity) {
        gz::math::Pose3d child_link_model_t_child_link =
            ecm.Component<gz::sim::components::Pose>(child_link_entity)->Data();
        // We don't check for cycles as they are yet not supported in world.
        child_link_queue.push_back(
            {.link_entity = child_link_entity,
             .parent_model_entity = child_link_model.Entity(),
             .parent_joint_entity = child_joint_entity,
             .base_t_link =
                 base_t_child_link_model * child_link_model_t_child_link,
             .base_t_parent_model = base_t_child_link_model});
      } else {
        LOG(WARNING)
            << "Could not resolve child link: "
            << ecm.Component<ChildLinkName>(child_joint_entity)->Data();
      }
    }
  }
  return absl::OkStatus();
}
}  // namespace

template <>
void CreateECMComponent<JointPosition>(gz::sim::Entity joint_entity,
                                       uint joint_dof_count,
                                       EntityComponentManager& ecm) {
  return CreateVectorDataComponent<JointPosition>(joint_entity, joint_dof_count,
                                                  ecm);
}

template <>
void CreateECMComponent<JointVelocity>(gz::sim::Entity joint_entity,
                                       uint joint_dof_count,
                                       EntityComponentManager& ecm) {
  return CreateVectorDataComponent<JointVelocity>(joint_entity, joint_dof_count,
                                                  ecm);
}

template <>
void CreateECMComponent<JointForce>(gz::sim::Entity joint_entity,
                                    uint joint_dof_count,
                                    EntityComponentManager& ecm) {
  return CreateVectorDataComponent<JointForce>(joint_entity, joint_dof_count,
                                               ecm);
}

void PreparePositionAndVelocityResetECMComponents(gz::sim::Entity joint_entity,
                                                  uint joint_dof_count,
                                                  EntityComponentManager& ecm) {
  // Remove joint velocity command component
  ecm.RemoveComponent<JointVelocityCmd>(joint_entity);

  // Create joint torque command component. This is required to apply gravity
  // compensation in position and velocity reset mode
  CreateVectorDataComponent<JointForceCmd>(joint_entity, joint_dof_count, ecm);

  // Create position reset command component
  CreateVectorDataComponent<JointPositionReset>(joint_entity, joint_dof_count,
                                                ecm);

  // Create velocity reset command component
  CreateVectorDataComponent<JointVelocityReset>(joint_entity, joint_dof_count,
                                                ecm);
}

void PrepareVelocityCmdECMComponent(gz::sim::Entity joint_entity,
                                    uint joint_dof_count,
                                    EntityComponentManager& ecm) {
  // Remove position reset command component
  ecm.RemoveComponent<JointPositionReset>(joint_entity);

  // Remove velocity reset command component
  ecm.RemoveComponent<JointVelocityReset>(joint_entity);

  // Remove joint torque command component
  ecm.RemoveComponent<JointForceCmd>(joint_entity);

  // Create joint velocity command component if one doesn't exist
  CreateVectorDataComponent<JointVelocityCmd>(joint_entity, joint_dof_count,
                                              ecm);
}

void PrepareTorqueCmdECMComponent(gz::sim::Entity joint_entity,
                                  uint joint_dof_count,
                                  EntityComponentManager& ecm) {
  // Remove joint velocity command component
  ecm.RemoveComponent<JointVelocityCmd>(joint_entity);

  // Remove position reset command component
  ecm.RemoveComponent<JointPositionReset>(joint_entity);

  // Remove velocity reset command component
  ecm.RemoveComponent<JointVelocityReset>(joint_entity);

  // Create joint torque command component if one doesn't exist
  CreateVectorDataComponent<JointForceCmd>(joint_entity, joint_dof_count, ecm);
}

template <>
std::vector<double>& GetECMComponentData<JointPosition>(
    gz::sim::Entity joint_entity, EntityComponentManager& ecm) {
  return GetComponentVectorData<JointPosition>(joint_entity, ecm);
}

template <>
std::vector<double>& GetECMComponentData<JointPositionReset>(
    gz::sim::Entity joint_entity, EntityComponentManager& ecm) {
  return GetComponentVectorData<JointPositionReset>(joint_entity, ecm);
}

template <>
std::vector<double>& GetECMComponentData<JointVelocity>(
    gz::sim::Entity joint_entity, EntityComponentManager& ecm) {
  return GetComponentVectorData<JointVelocity>(joint_entity, ecm);
}

template <>
std::vector<double>& GetECMComponentData<JointVelocityReset>(
    gz::sim::Entity joint_entity, EntityComponentManager& ecm) {
  return GetComponentVectorData<JointVelocityReset>(joint_entity, ecm);
}

template <>
std::vector<double>& GetECMComponentData<JointVelocityCmd>(
    gz::sim::Entity joint_entity, EntityComponentManager& ecm) {
  return GetComponentVectorData<JointVelocityCmd>(joint_entity, ecm);
}

template <>
std::vector<double>& GetECMComponentData<JointForce>(
    gz::sim::Entity joint_entity, EntityComponentManager& ecm) {
  return GetComponentVectorData<JointForce>(joint_entity, ecm);
}

template <>
std::vector<double>& GetECMComponentData<JointForceCmd>(
    gz::sim::Entity joint_entity, EntityComponentManager& ecm) {
  return GetComponentVectorData<JointForceCmd>(joint_entity, ecm);
}

absl::StatusOr<gz::sim::Entity> GetJointEntityFromRelativeScopedName(
    absl::string_view relative_scoped_joint_name,
    absl::string_view scope_separator, gz::sim::Entity parent_entity,
    const EntityComponentManager& ecm) {
  std::vector<std::string> parts =
      absl::StrSplit(relative_scoped_joint_name, scope_separator);
  if (parts.size() > 1) {
    for (auto& part : parts) {
      auto child_model_entity = GetChildModelByName(part, parent_entity, ecm);
      if (!child_model_entity.ok()) {
        break;
      }
      parent_entity = *child_model_entity;
      relative_scoped_joint_name = relative_scoped_joint_name.substr(
          part.size() + scope_separator.size());
    }
  }

  Model model(parent_entity);
  if (!model.Valid(ecm)) {
    return intrinsic::InvalidArgumentErrorBuilder()
           << "Parent entity " << parent_entity << " is not a model";
  }

  auto joint_entity =
      model.JointByName(ecm, std::string(relative_scoped_joint_name));
  if (joint_entity == kNullEntity) {
    return intrinsic::NotFoundErrorBuilder()
           << "Unable to find joint named " << relative_scoped_joint_name;
  }

  return joint_entity;
}

absl::StatusOr<double> GetChildLinkTotalMass(
    gz::sim::Entity joint_entity, absl::string_view scope_separator,
    const EntityComponentManager& ecm) {
  double child_link_mass = 0.0;

  LinkVisitorFunction visit_link =
      [&](LinkVisitInfo link_info,
          const EntityComponentManager& ecm) -> absl::Status {
    if (link_info.parent_joint_entity != joint_entity) {
      SdfJointType joint_type = ecm.Component<gz::sim::components::JointType>(
                                       link_info.parent_joint_entity)
                                    ->Data();
      if (joint_type != SdfJointType::FIXED) {
        return intrinsic::InvalidArgumentErrorBuilder()
               << "Can't add link masses across a non-fixed joint of type "
               << static_cast<int>(joint_type);
      }
    }
    gz::math::Inertiald inertial =
        ecm.Component<gz::sim::components::Inertial>(link_info.link_entity)
            ->Data();
    double mass = inertial.MassMatrix().Mass();
    child_link_mass += mass;
    return absl::OkStatus();
  };

  INTR_RETURN_IF_ERROR(TraverseDescendentLinkTree(joint_entity, scope_separator,
                                                  ecm, visit_link));
  return child_link_mass;
}

absl::StatusOr<double> GetChildLinkTotalInertia(
    gz::sim::Entity joint_entity, absl::string_view scope_separator,
    const EntityComponentManager& ecm) {
  gz::math::Inertiald total_joint_t_child_link_inertia;
  gz::math::Vector3d axis;
  {
    // Extract joint axis from ECM.
    std::optional<sdf::JointAxis> sdf_axis =
        ecm.ComponentData<gz::sim::components::JointAxis>(joint_entity);
    if (!sdf_axis.has_value()) {
      return intrinsic::InvalidArgumentErrorBuilder()
             << "Joint axis not found for given joint entity.";
    }
    sdf::Errors errors = sdf_axis->ResolveXyz(axis);
    if (!errors.empty()) {
      return intrinsic::InternalErrorBuilder()
             << "Could not resolve joint axis. SDF errors: " << errors;
    }
  }
  const gz::math::Pose3d child_link_t_joint =
      ecm.Component<gz::sim::components::Pose>(joint_entity)->Data();

  // Base is the parent model frame of the joint. Base_t_joint is initialized in
  // the first child link visit.
  std::optional<gz::math::Pose3d> base_t_joint;

  LinkVisitorFunction visit_link =
      [&](LinkVisitInfo link_info,
          const EntityComponentManager& ecm) -> absl::Status {
    if (link_info.parent_joint_entity == joint_entity) {
      base_t_joint = link_info.base_t_link * child_link_t_joint;
    } else {
      SdfJointType joint_type = ecm.Component<gz::sim::components::JointType>(
                                       link_info.parent_joint_entity)
                                    ->Data();

      if (joint_type != SdfJointType::FIXED) {
        return intrinsic::InvalidArgumentErrorBuilder()
               << "Can't add link inertia across a non-fixed joint of type "
               << static_cast<int>(joint_type);
      }
    }
    CHECK(base_t_joint.has_value());
    gz::math::Pose3d joint_t_link =
        base_t_joint->Inverse() * link_info.base_t_link;
    gz::math::Inertiald link_inertial =
        ecm.Component<gz::sim::components::Inertial>(link_info.link_entity)
            ->Data();

    gz::math::Inertiald joint_t_link_inertial(
        link_inertial.MassMatrix(), joint_t_link * link_inertial.Pose());
    total_joint_t_child_link_inertia += joint_t_link_inertial;
    return absl::OkStatus();
  };

  INTR_RETURN_IF_ERROR(TraverseDescendentLinkTree(joint_entity, scope_separator,
                                                  ecm, visit_link));

  // Rotation of total_joint_t_child_link_inertia should be identity. So we can
  // simply compute the mass matrix component along the joint axis.
  double child_link_inertia =
      axis.Dot(total_joint_t_child_link_inertia.MassMatrix().Moi() * axis);
  // Add parallel axis theorem component.
  gz::math::Vector3d com = total_joint_t_child_link_inertia.Pose().Pos();
  double com_axis_distance_sqr = (com - (com.Dot(axis)) * axis).SquaredLength();
  child_link_inertia += com_axis_distance_sqr *
                        total_joint_t_child_link_inertia.MassMatrix().Mass();

  return child_link_inertia;
}

}  // namespace simulation
}  // namespace intrinsic
