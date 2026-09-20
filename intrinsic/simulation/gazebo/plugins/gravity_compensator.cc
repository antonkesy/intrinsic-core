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

#include "intrinsic/simulation/gazebo/plugins/gravity_compensator.h"

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "gz/math/Inertial.hh"
#include "gz/math/Pose3.hh"
#include "gz/math/Vector3.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/World.hh"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"
#include "sdf/Types.hh"

namespace intrinsic {
namespace simulation {

using ::gz::math::Pose3d;
using ::gz::math::Vector3d;
using ::gz::sim::Entity;
using ::gz::sim::kNullEntity;
using ::gz::sim::components::ChildLinkName;
using ::gz::sim::components::Inertial;
using ::gz::sim::components::Link;
using ::gz::sim::components::Model;
using ::gz::sim::components::Name;
using ::gz::sim::components::ParentLinkName;
using ::gz::sim::components::Pose;
using ::sdf::JointType;

absl::StatusOr<GravityCompensator::JointDescendantsInfo>
GravityCompensator::ComputeJointDescendantsInfo(
    const gz::sim::EntityComponentManager& ecm, gz::sim::Entity joint_entity,
    absl::string_view joint_name, absl::string_view scope_separator) {
  Entity model_entity = ecm.ParentEntity(joint_entity);
  if (model_entity == kNullEntity) {
    return intrinsic::InvalidArgumentErrorBuilder()
           << "Got null parent model entity for joint " << joint_name;
  }

  std::string child_name = ecm.Component<ChildLinkName>(joint_entity)->Data();
  std::vector<Entity> child_link_entities =
      ecm.ChildrenByComponents(model_entity, Link(), Name(child_name));

  if (child_link_entities.size() != 1 ||
      child_link_entities[0] == kNullEntity) {
    return intrinsic::UnimplementedErrorBuilder()
           << "Could not find a unique child link entity for joint "
           << joint_name
           << " within the same model. Note that GravityCompensator does not "
              "support joints whose child link is defined in a nested model.";
  }
  Entity child_link_entity = child_link_entities[0];

  absl::flat_hash_map<Entity, int> model_ind = {{model_entity, 0}};

  std::vector<ModelPoseInfo> model_poses;
  model_poses.push_back(
      {.entity = model_entity, .pose = ecm.Component<Pose>(model_entity)});

  absl::flat_hash_map<gz::sim::Entity, LinkInfo> child_links_info_map;
  child_links_info_map[child_link_entity] =
      LinkInfo{.entity = child_link_entity,
               .link_name = ecm.Component<Name>(child_link_entity)->Data(),
               .inertial = ecm.Component<Inertial>(child_link_entity),
               .parent_ind = 0};
  std::deque<Entity> child_link_queue;
  child_link_queue.push_back(child_link_entity);

  while (!child_link_queue.empty()) {
    const Entity link_entity = child_link_queue.front();
    child_link_queue.pop_front();

    // TODO(b/198173664): This won't compensate for weight of end-effectors
    // attached using a DetachableJoint

    const std::string link_name = child_links_info_map[link_entity].link_name;
    const std::vector<Entity> child_joint_entities = ecm.EntitiesByComponents(
        gz::sim::components::ParentLinkName(link_name));

    const Entity parent_model_entity =
        model_poses[child_links_info_map[link_entity].parent_ind].entity;

    for (const Entity joint : child_joint_entities) {
      if (ecm.ParentEntity(joint) != parent_model_entity) {
        // TODO(b/243292841) Add a flag that controls whether joints specified
        // outside of the link model are considered or not.
        continue;
      }

      std::string joint_child_link_name =
          ecm.Component<ChildLinkName>(joint)->Data();
      Entity child_link_parent_model = parent_model_entity;

      // Check if child link might be under a nested model.
      int index = joint_child_link_name.find(scope_separator);
      while (index != std::string::npos) {
        const std::string intermediate_model_name =
            joint_child_link_name.substr(0, index);
        const Entity child_model_entity =
            ecm.EntityByComponents(Model(), Name(intermediate_model_name));
        if (child_model_entity == kNullEntity) {
          return InvalidArgumentErrorBuilder()
                 << "Could not find model named " << intermediate_model_name
                 << " which is in the scoped child link name for joint "
                 << ecm.Component<Name>(joint)->Data();
        }
        if (!model_ind.contains(child_model_entity)) {
          model_poses.push_back(
              {.entity = child_model_entity,
               .pose = ecm.Component<Pose>(child_model_entity),
               .parent_ind = model_ind[child_link_parent_model]});
          model_ind[child_model_entity] = model_poses.size() - 1;
        }
        child_link_parent_model = child_model_entity;

        joint_child_link_name =
            joint_child_link_name.substr(index + scope_separator.size());
        index = joint_child_link_name.find(scope_separator);
      }

      gz::sim::Model model(child_link_parent_model);
      Entity child_link_entity = model.LinkByName(ecm, joint_child_link_name);
      if (child_link_entity != kNullEntity) {
        child_links_info_map[child_link_entity] =
            LinkInfo{.entity = child_link_entity,
                     .link_name = joint_child_link_name,
                     .inertial = ecm.Component<Inertial>(child_link_entity),
                     .parent_ind = model_ind[child_link_parent_model]};
        child_link_queue.push_back(child_link_entity);
      }
    }
  }

  // Flatten map to vector since we don't care about the mapping information
  // when computing torque.
  std::vector<LinkInfo> child_links_info_vec;
  for (auto const& [link, link_info] : child_links_info_map) {
    child_links_info_vec.push_back(link_info);
  }

  return JointDescendantsInfo{.parent_model_entity = model_entity,
                              .child_link_entity = child_link_entity,
                              .links = std::move(child_links_info_vec),
                              .model_poses = std::move(model_poses)};
}

absl::StatusOr<std::unique_ptr<GravityCompensator>> GravityCompensator::Create(
    gz::sim::Entity joint_entity, const gz::sim::EntityComponentManager* ecm,
    absl::string_view scope_separator) {
  if (joint_entity == kNullEntity) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Passed null joint entity";
  }

  if (ecm == nullptr) {
    return ::intrinsic::InvalidArgumentErrorBuilder() << "Passed null ecm";
  }

  const Name* joint_name_component = ecm->Component<Name>(joint_entity);
  if (joint_name_component == nullptr) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Joint entity does not have a name";
  }
  const std::string& joint_name = joint_name_component->Data();

  JointType joint_type =
      ecm->Component<::gz::sim::components::JointType>(joint_entity)->Data();

  if (joint_type != JointType::REVOLUTE && joint_type != JointType::PRISMATIC) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "GravityCompensator not supported for joint type "
           << static_cast<int>(joint_type);
  }

  sdf::JointAxis sdf_axis =
      ecm->Component<gz::sim::components::JointAxis>(joint_entity)->Data();
  Vector3d axis;
  sdf::Errors errors = sdf_axis.ResolveXyz(axis);
  if (!errors.empty()) {
    return intrinsic::InternalErrorBuilder()
           << "Could not resolve joint axis for joint " << joint_name
           << ". SDF errors: " << errors;
  }

  INTR_ASSIGN_OR_RETURN(JointDescendantsInfo joint_descendants_info,
                        ComputeJointDescendantsInfo(
                            *ecm, joint_entity, joint_name, scope_separator));

  return absl::WrapUnique(
      new GravityCompensator(joint_entity, joint_type, axis,
                             std::move(joint_descendants_info.model_poses),
                             std::move(joint_descendants_info.links),
                             joint_descendants_info.child_link_entity, ecm));
}

GravityCompensator::GravityCompensator(
    gz::sim::Entity joint_entity, sdf::JointType joint_type,
    const gz::math::Vector3d& joint_axis,
    std::vector<ModelPoseInfo> initial_model_poses,
    std::vector<LinkInfo> child_links_info, gz::sim::Entity child_link_entity,
    const gz::sim::EntityComponentManager* ecm)
    : joint_type_(joint_type),
      joint_axis_(joint_axis),
      child_t_joint_(ecm->Component<Pose>(joint_entity)->Data()),
      child_link_entity_(child_link_entity),
      ecm_(ecm),
      model_poses_(std::move(initial_model_poses)),
      child_links_info_(std::move(child_links_info)) {
  Entity world_entity = ecm_->EntityByComponents(gz::sim::components::World());
  g_world_ =
      ecm_->Component<gz::sim::components::Gravity>(world_entity)->Data();
}

void GravityCompensator::UpdateModelPoses() {
  for (ModelPoseInfo& model_pose_info : model_poses_) {
    if (model_pose_info.parent_ind < 0) {
      continue;
    }
    Pose3d root_t_parent =
        model_poses_[model_pose_info.parent_ind].root_t_model;
    model_pose_info.root_t_model = root_t_parent * model_pose_info.pose->Data();
  }
}

double GravityCompensator::ComputeTorque() {
  UpdateModelPoses();

  Pose3d world_t_child_link = gz::sim::worldPose(child_link_entity_, *ecm_);
  Pose3d joint_t_world = (world_t_child_link * child_t_joint_).Inverse();

  Vector3d g_joint = joint_t_world.Rot() * g_world_;

  const Pose3d model_t_child_link =
      ecm_->Component<Pose>(child_link_entity_)->Data();
  const Pose3d model_t_joint = model_t_child_link * child_t_joint_;

  double gravity_torque = 0.0;

  for (auto const& link_info : child_links_info_) {
    const gz::math::Inertiald inertial = link_info.inertial->Data();
    const double mass = inertial.MassMatrix().Mass();
    Pose3d model_t_parent = model_poses_[link_info.parent_ind].root_t_model;
    if (joint_type_ == JointType::REVOLUTE) {
      const Pose3d model_t_link =
          model_t_parent * ecm_->Component<Pose>(link_info.entity)->Data();
      const Pose3d joint_t_link = model_t_joint.Inverse() * model_t_link;
      const Vector3d com_in_joint = (joint_t_link * inertial.Pose()).Pos();
      gravity_torque -= mass * (com_in_joint.Cross(g_joint)).Dot(joint_axis_);
    } else if (joint_type_ == JointType::PRISMATIC) {
      gravity_torque -= mass * g_joint.Dot(joint_axis_);
    }
  }
  return gravity_torque;
}

}  // namespace simulation
}  // namespace intrinsic
