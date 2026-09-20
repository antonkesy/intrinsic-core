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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_INFER_HARDWARE_INTERFACES_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_INFER_HARDWARE_INTERFACES_H_

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Model.hh"
#include "intrinsic/icon/hal/proto/hardware_module_config.pb.h"
#include "intrinsic/icon/hardware_modules/sim_bus/sim_bus_hardware_module.pb.h"
#include "intrinsic/simulation/gazebo/asset_instances_client.h"
#include "intrinsic/simulation/gazebo/plugins/sim_hardware_module_config.pb.h"

namespace intrinsic::simulation {

namespace automatic_interface_names {
constexpr char kAutomaticJointGroupName[] = "auto_joint_group";
constexpr char kAutomaticJointPositionCommandInterfaceName[] =
    "joint_position_command";
constexpr char kAutomaticJointTorqueCommandInterfaceName[] =
    "joint_torque_command";
constexpr char kAutomaticJointPositionStateInterfaceName[] =
    "joint_position_state";
constexpr char kAutomaticJointCommandedPositionInterfaceName[] =
    "joint_commanded_position";
constexpr char kAutomaticJointVelocityStateInterfaceName[] =
    "joint_velocity_state";
constexpr char kAutomaticJointAccelerationStateInterfaceName[] =
    "joint_acceleration_state";
constexpr char kAutomaticJointTorqueStateInterfaceName[] = "joint_torque_state";
constexpr char kAutomaticPayloadCommandInterfaceName[] = "payload_command";
constexpr char kAutomaticPayloadStateInterfaceName[] = "payload_state";
constexpr char kAutomaticProcessWrenchCommandInterfaceName[] =
    "process_wrench_command";
constexpr char kAutomaticForceTorqueCommandInterfaceName[] =
    "force_torque_command";
constexpr char kAutomaticForceTorqueStatusInterfaceName[] =
    "force_torque_status";
constexpr char kAutomaticJointLimitsCommandInterfaceName[] =
    "joint_system_limits";
constexpr char kAutomaticRangefinderStatusInterfaceName[] =
    "rangefinder_status";
}  // namespace automatic_interface_names

// Automatically create a HardwareModuleConfig to tell the rest of
// HardwareModuleLauncher which interfaces to simulate.
//
// This implements the default behavior of HardwareModuleLauncher: To infer as
// much as possible when there is no explicit configuration. This should be
// the common case, with manual configuration only required in exotic cases.
//
// This function behaves differently based on which of the `oneof` values in
// `intrinsic_proto.sim.SimHardwareModuleConfig.hardware_interfaces` is set:
// * If the oneof is not set, or set to `from_self`, try to infer interfaces
//   from `hwm_resource_gazebo_model`itself.
// * If it is set to `geometry_asset_name`, then
//   * Find the Gazebo model corresponding to `geometry_asset_name`
//   * Infer interfaces from that model
// * If it is set to `from_instances`, then
//   * Iterate through all instances,
//   * Find their Gazebo models, and aggregate inferred interfaces.
//   * For each instance, apply mappings based on raw inferred names (before
//     prefixing).
//   * Finally, default-apply the instance prefix to any unmapped interface.
// * If it is set to `no_interfaces`, or `manual_hardware_interfaces`, do
//   nothing and return the existing `SimHardwareModuleConfig` from `sim_config`
//   unchanged.
//
// This function relies extensively on Intrinsic's custom `ResourceName`
// component. Since we cannot be sure that the system that adds that name is
// configured before HardwareModuleLauncher, we call this function in the first
// PreUpdate().
//
// There are some limitations:
// * This only supports at most one joint command/state interface per SDF
//   (because there's no convention for how to name additional ones).
// * Any DIO blocks must have globally unique names (as opposed to unique
//   within their gz::sim::Model). Again, this is for naming reasons. When using
//   `from_instances` you may use prefixing to disambiguate.
absl::StatusOr<::intrinsic_proto::sim::SimHardwareModuleConfig>
InferSimHardwareModuleConfig(
    absl::string_view hwm_resource_name, absl::string_view hwm_name,
    const ::gz::sim::Model& hwm_resource_gazebo_model,
    const ::intrinsic_proto::icon::SimBusModuleConfig& sim_config,
    ::gz::sim::EntityComponentManager& ecm);

// Tries to find a `control_frequency_hz` tag in the ModelSdf component for an
// ECM model.
// `sim_config` decides which model this function looks at:
// * If `control_frequency_asset_name` is set, find the model that corresponds
//   to that asset name, and use that.
// * Else, if `geometry_asset_name` is set, find the model that corresponds
//   to that asset name, and use that.
// * Else, if `from_instances` is set, find the tag across all instances, and
//   use the first, if all found values are identical.
// * Else, use `hwm_resource_gazebo_model`
//
// Returns the control *period*, even though the SDF tag holds the control
// *frequency*.
// Returns NotFoundError if there is no corresponding Gazebo model for either of
// the optional asset names (see above).
// Returns InvalidArgumentError if there is more than one corresponding Gazebo
// model for either of the optional asset names.
// Returns InvalidArgumentError if the control frequencies found across all
// instances are not unique.
// Returns NotFoundError if the ECM model entity does not have a `ModelSdf`
// component.
// Returns NotFoundError if the SDF for the given model does not have a
// `control_frequency_hz` tag.
// Returns InvalidArgumentError if the SDF has a `control_frequency_hz` tag, but
// that tag does not have a valid frequency value inside it.
absl::StatusOr<absl::Duration> FindControlPeriod(
    const ::intrinsic_proto::icon::HardwareModuleConfig& module_config,
    const ::gz::sim::Model& hwm_resource_gazebo_model,
    const ::intrinsic_proto::sim::SimHardwareModuleConfig& sim_config,
    const ::gz::sim::EntityComponentManager& ecm,
    const AssetInstancesClient& asset_instances_client);

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_INFER_HARDWARE_INTERFACES_H_
