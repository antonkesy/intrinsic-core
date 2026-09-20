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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_JOINT_DEVICE_UTIL_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_JOINT_DEVICE_UTIL_H_

#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"

namespace intrinsic {
namespace simulation {

// Collection of helper functions to implement Joint control devices in Gazebo.

// Create a new Joint component, one of JointPosition, JointVelocity or
// JointForce. No-op if the component exists already.
template <class ComponentType>
void CreateECMComponent(::gz::sim::Entity joint_entity, uint joint_dof_count,
                        ::gz::sim::EntityComponentManager& ecm);

// The Physics System allows a fixed set of combinations of components for
// joint control.
// - When Position and Velocity Reset components are specified, VelocityCmd
// cannot be specified. So we remove the VelocityCmd component from the ECM.
// - When VelocityCmd is specified, PositionReset, VelocityReset and ForceCmd
// components are removed from the ECM.
// - When ForceCmd is specified, PositionReset, VelocityReset and VelocityCmd
// components are removed from the ECM.

void PreparePositionAndVelocityResetECMComponents(
    ::gz::sim::Entity joint_entity, uint joint_dof_count,
    ::gz::sim::EntityComponentManager& ecm);

void PrepareVelocityCmdECMComponent(::gz::sim::Entity joint_entity,
                                    uint joint_dof_count,
                                    ::gz::sim::EntityComponentManager& ecm);

void PrepareTorqueCmdECMComponent(::gz::sim::Entity joint_entity,
                                  uint joint_dof_count,
                                  ::gz::sim::EntityComponentManager& ecm);

// Mutable joint component data. ComponentType can be one of JointPosition,
// JointPositionReset, JointVelocity, JointVelocityReset, JointVelocityCmd,
// JointForce and JointForceCmd.
// NOTE: Does not check if the component is actually available, for
// performance! If you want to check if the component is available, use
// ecm.ComponentData<ComponentType>(joint_entity) instead.
template <class ComponentType>
std::vector<double>& GetECMComponentData(
    ::gz::sim::Entity joint_entity, ::gz::sim::EntityComponentManager& ecm);

// Preferred scope separator for joint names specified in the plugin SDF for
// Timeslicer Joint Devices in Gazebo
constexpr char kJointDeviceJointNameScopeSeparator[] = "::";

// Get Joint entity by parsing relative scoped name of the form
// "child_model::grandchild_model::joint_name". Just "joint_name" also works.
absl::StatusOr<::gz::sim::Entity> GetJointEntityFromRelativeScopedName(
    absl::string_view relative_scoped_joint_name,
    absl::string_view scope_separator, ::gz::sim::Entity parent_entity,
    const ::gz::sim::EntityComponentManager& ecm);

// Get the total child link mass across all fixed joints for a given joint.
// Note: It is assumed that the model does not contain any loops.
// Note: A well-formed ECM is expected, otherwise a bad memory access can occur.
absl::StatusOr<double> GetChildLinkTotalMass(
    ::gz::sim::Entity joint_entity, absl::string_view scope_separator,
    const ::gz::sim::EntityComponentManager& ecm);

// Gets the total child link inertia across all fixed joints for a given joint
// about the joint axis.
// Only supports joints with a single axis, an UnimplementedError is returned
// if the input joint is either fixed or has two axes.
// Note: It is assumed that the model does not contain any loops.
// Note: A well-formed ECM is expected, otherwise a bad memory access can occur.
absl::StatusOr<double> GetChildLinkTotalInertia(
    ::gz::sim::Entity joint_entity, absl::string_view scope_separator,
    const ::gz::sim::EntityComponentManager& ecm);

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_JOINT_DEVICE_UTIL_H_
