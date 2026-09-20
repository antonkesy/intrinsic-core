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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRAVITY_COMPENSATOR_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRAVITY_COMPENSATOR_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "gz/math/Pose3.hh"
#include "gz/math/Vector3.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Pose.hh"
#include "sdf/Joint.hh"

namespace intrinsic {
namespace simulation {

class GravityCompensator {
 public:
  // Does not own ecm, but saves a pointer to it. So ecm should outlast the
  // created instance.
  static absl::StatusOr<std::unique_ptr<GravityCompensator>> Create(
      gz::sim::Entity joint_entity, const gz::sim::EntityComponentManager* ecm,
      absl::string_view scope_separator);

  // Computes torque based on current pose of descendent links.
  double ComputeTorque();

 private:
  // Helper structs to store information on models and links to improve
  // performance.

  struct ModelPoseInfo {
    gz::sim::Entity entity;
    const gz::sim::components::Pose* pose;
    gz::math::Pose3d root_t_model;
    // Index of parent in a vector of ModelPoseInfo. Value of -1 indicates root.
    int parent_ind = -1;
  };

  struct LinkInfo {
    gz::sim::Entity entity;
    std::string link_name;
    const gz::sim::components::Inertial* inertial;
    // Index of parent model in a vector of ModelPoseInfo.
    int parent_ind;
  };

  GravityCompensator(gz::sim::Entity joint_entity, ::sdf::JointType joint_type,
                     const gz::math::Vector3d& joint_axis,
                     std::vector<ModelPoseInfo> initial_model_poses,
                     std::vector<LinkInfo> child_links_info,
                     gz::sim::Entity child_link_entity,
                     const gz::sim::EntityComponentManager* ecm);

  void UpdateModelPoses();

  const ::sdf::JointType joint_type_;
  const gz::math::Vector3d joint_axis_;
  const gz::math::Pose3d child_t_joint_;
  const gz::sim::Entity child_link_entity_;
  const gz::sim::EntityComponentManager* ecm_;
  gz::math::Vector3d g_world_;
  std::vector<ModelPoseInfo> model_poses_;
  std::vector<LinkInfo> child_links_info_;

  struct JointDescendantsInfo {
    // Parent model of the joint.
    gz::sim::Entity parent_model_entity;

    // Child link entity of the joint.
    gz::sim::Entity child_link_entity;

    // Descendant links of the joint. The order in the vector is not specified.
    std::vector<GravityCompensator::LinkInfo> links;

    // Models which contain descendant links of the joint. First element in
    // `model_poses` contains pose info for `parent_model_entity`.
    std::vector<ModelPoseInfo> model_poses;
  };

  static absl::StatusOr<JointDescendantsInfo> ComputeJointDescendantsInfo(
      const gz::sim::EntityComponentManager& ecm, gz::sim::Entity joint_entity,
      absl::string_view joint_name, absl::string_view scope_separator);
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRAVITY_COMPENSATOR_H_
