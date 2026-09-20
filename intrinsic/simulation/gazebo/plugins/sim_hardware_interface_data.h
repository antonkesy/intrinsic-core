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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SIM_HARDWARE_INTERFACE_DATA_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SIM_HARDWARE_INTERFACE_DATA_H_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "intrinsic/icon/hardware_modules/sim_bus/sim_bus_hardware_module.pb.h"
#include "intrinsic/simulation/gazebo/plugins/gravity_compensator.h"
#include "intrinsic/simulation/gazebo/plugins/sim_hardware_module_config.pb.h"

namespace intrinsic::simulation::hardware_interface_data {

// This file contains data types and helper functions that allow conversion
// between Gazebo ECM data and Intrinsic hardware interfaces (more precisely,
// their flatbuffer message types).
//
// They're used by GazeboHwm / HardwareModuleLauncher.
//
// The structs and std::variant definition below map to the hardware interface
// configuration types in
// intrinsic/simulation/gazebo/plugins/sim_hardware_module_config.proto.
//
// Note that, while many of these structs map 1:1 to their proto counterparts,
// some hold additional data that we use at runtime. One example of this is the
// vector of `GravityCompensator`s for the joint command interfaces.
//
// By defining a single std::variant that encompasses all interface types, we
// gain two things:
// * We can store all interfaces in a single flat_hash_map keyed with the
//   interface name, which means we can guarantee that interface names are
//   unique.
// * When we deal with the interfaces in HardwareModuleLauncher and GazeboHwm,
//   we can use std::visit to dynamically dispatch the correct method (and get
//   compile errors if we forget to handle an interface type)

struct StrictJointPositionCommandData {
  std::string joint_group_name;
  // If this is unset (this can happen at the start of the simulation if there
  // is no initial pose information in the ECM), GazeboHwm populates it with the
  // *current* setpoint.
  std::optional<std::vector<double>> previous_setpoints;
  // This must have as many elements as there are joints in the group
  // `joint_group_name`. The elements may be nullptr to indicate that the
  // interface should not do gravity compensation.
  std::vector<std::unique_ptr<GravityCompensator>> gravity_compensators;
};
struct NonStrictJointPositionCommandData {
  std::string joint_group_name;
  // If this is unset (this can happen at the start of the simulation if there
  // is no initial pose information in the ECM), GazeboHwm populates it with the
  // *current* setpoint.
  std::optional<std::vector<double>> previous_setpoints;
  // This must have as many elements as there are joints in the group
  // `joint_group_name`. The elements may be nullptr to indicate that the
  // interface should not do gravity compensation.
  std::vector<std::unique_ptr<GravityCompensator>> gravity_compensators;
};
struct StrictJointTorqueCommandData {
  std::string joint_group_name;
  std::vector<std::unique_ptr<GravityCompensator>> gravity_compensators;
};
struct NonStrictJointTorqueCommandData {
  std::string joint_group_name;
  std::vector<std::unique_ptr<GravityCompensator>> gravity_compensators;
};
struct JointPositionStateData {
  std::string joint_group_name;
};
struct JointCommandedPositionData {
  std::string joint_group_name;
};
struct JointVelocityStateData {
  std::string joint_group_name;
};
struct JointAccelerationStateData {
  std::string joint_group_name;
};
struct JointTorqueStateData {
  std::string joint_group_name;
};
struct KinematicChainPayloadCommandData {
  std::string joint_group_name;
};
struct KinematicChainPayloadStateData {
  std::string joint_group_name;
};
struct KinematicChainProcessWrenchCommandData {
  std::string joint_group_name;
};
struct DigitalInputStatusData {
  gz::sim::Entity digital_input_entity;
  // Technically, we don't need to store this (we could always look up the
  // number of inputs from the ECM again), but keeping lookups in the ECM (and
  // access to it in general) to a minimum makes some things simpler.
  size_t num_inputs;
  // Optional aliases for the individual bits of the input block (LSB is at
  // index 0). Bits that aren't in this map get an auto-generated name based on
  // their bit index.
  absl::flat_hash_map<size_t, std::string> bit_index_to_alias;
};
struct DigitalOutputCommandData {
  gz::sim::Entity digital_output_entity;
  // Technically, we don't need to store this (we could always look up the
  // number of outputs from the ECM again), but keeping lookups in the ECM (and
  // access to it in general) to a minimum makes some things simpler.
  size_t num_outputs;
  // Optional aliases for the individual bits of the output block (LSB is at
  // index 0). Bits that aren't in this map get an auto-generated name based on
  // their bit index.
  absl::flat_hash_map<size_t, std::string> bit_index_to_alias;
};
struct ForceTorqueCommandData {
  gz::sim::Entity ft_sensor_entity;
};
struct ForceTorqueStatusData {
  gz::sim::Entity ft_sensor_entity;
};
struct AnalogInputStatusData {
  size_t num_inputs;
};
struct AnalogOutputCommandData {
  size_t num_outputs;
};
struct JointLimitsCommandData {
  std::string joint_group_name;
};
struct RangefinderStatusData {
  gz::sim::Entity rangefinder_entity;
};

using HardwareInterfaceData = std::variant<
    StrictJointPositionCommandData, NonStrictJointPositionCommandData,
    StrictJointTorqueCommandData, NonStrictJointTorqueCommandData,
    JointPositionStateData, JointCommandedPositionData, JointVelocityStateData,
    JointAccelerationStateData, JointTorqueStateData,
    KinematicChainPayloadCommandData, KinematicChainPayloadStateData,
    KinematicChainProcessWrenchCommandData, DigitalInputStatusData,
    DigitalOutputCommandData, ForceTorqueCommandData, ForceTorqueStatusData,
    AnalogInputStatusData, AnalogOutputCommandData, JointLimitsCommandData,
    RangefinderStatusData>;

// Populates a map from interface name to HardwareInterfaceData based on
// `sim_hwm_config` and the contents of `ecm`.
//
// Modifies `ecm` to add components like `JointPosition` and `WrenchMeasured`,
// whose presence indicates to Gazebo that it should save the corresponding
// values in the ECM.
absl::StatusOr<absl::flat_hash_map<std::string, HardwareInterfaceData>>
BuildHardwareInterfaceData(
    absl::string_view hwm_resource_name,
    const intrinsic_proto::sim::SimHardwareModuleConfig& sim_hwm_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    gz::sim::EntityComponentManager& ecm,
    absl::flat_hash_map<gz::sim::Entity, std::string>&
        rangefinder_topic_by_entity);

// `BuildHardwareInterfaceData()` invokes the methods below, deoending on the
// types of hardware interface configs in its `sim_hwm_config` parameter. They
// are exposed in this header for a few reasons:
// * We can manually invoke them in tests
// * Their docstrings are visible here
//

// Extracts the interface data for a strict JointPositionCommand interface with
// `interface_config` from `joint_groups_by_name` and `ecm`.
//
// Uses `hwm_name` and `interface_name` for nicer log messages.
//
// Returns InvalidArgument if `interface_config.strict()` is false.
// Returns NotFoundError if `interface_config` refers to a joint group that is
// not present in `joint_groups_by_name`.
// Returns FailedPreconditionError if any of the joints for the configured joint
// group does not have a JointPosition *or* JointPositionReset component with at
// least one element.
// Returns an error if it cannot build a intrinsic::sim::GravityCompensator for
// any of the joints in the configured joint group.
absl::StatusOr<StrictJointPositionCommandData>
GetStrictJointPositionCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointPositionCommandInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm);

// Extracts the interface data for a non-strict JointPositionCommand interface
// with `interface_config` from `joint_groups_by_name` and `ecm`.
//
// Uses `hwm_name` and `interface_name` for nicer log messages.
//
// Returns InvalidArgument if `interface_config.strict()` is false.
// Returns NotFoundError if `interface_config` refers to a joint group that is
// not present in `joint_groups_by_name`.
// Returns FailedPreconditionError if any of the joints for the configured joint
// group does not have a JointPosition *or* JointPositionReset component with at
// least one element.
// Returns an error if it cannot build a intrinsic::sim::GravityCompensator for
// any of the joints in the configured joint group.
absl::StatusOr<NonStrictJointPositionCommandData>
GetNonStrictJointPositionCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointPositionCommandInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm);

// Extracts the interface data for a strict JointTorqueCommand interface with
// `interface_config` from `joint_groups_by_name` and `ecm`.
//
// Uses `hwm_name` and `interface_name` for nicer log messages.
//
// Returns InvalidArgument if `interface_config.strict()` is false.
// Returns NotFoundError if `interface_config` refers to a joint group that is
// not present in `joint_groups_by_name`.
// Returns an error if it cannot build a intrinsic::sim::GravityCompensator for
// any of the joints in the configured joint group.
absl::StatusOr<StrictJointTorqueCommandData>
GetStrictJointTorqueCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointTorqueCommandInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm);

// Extracts the interface data for a non-strict JointTorqueCommand interface
// with `interface_config` from `joint_groups_by_name` and `ecm`.
//
// Uses `hwm_name` and `interface_name` for nicer log messages.
//
// Returns InvalidArgument if `interface_config.strict()` is false.
// Returns NotFoundError if `interface_config` refers to a joint group that is
// not present in `joint_groups_by_name`.
// Returns an error if it cannot build a intrinsic::sim::GravityCompensator for
// any of the joints in the configured joint group.
absl::StatusOr<NonStrictJointTorqueCommandData>
GetNonStrictJointTorqueCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointTorqueCommandInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm);

// Extracts the interface data for a JointPositionState interface with
// `interface_config` from `joint_groups_by_name` and `ecm`.
//
// Adds the `JointPosition` Gazebo component to all joint entities in the
// selected joint group. This lets Gazebo know that it should populate the
// component during the physics update step, so we can read the positions and
// report them to ICON.
//
// Uses `hwm_name` and `interface_name` for nicer log messages.
//
// Returns NotFoundError if `interface_config` refers to a joint group that is
// not present in `joint_groups_by_name`.
absl::StatusOr<JointPositionStateData> GetJointPositionStateDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointPositionStateInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    gz::sim::EntityComponentManager& ecm);

// Extracts the interface data for a JointCommandedPosition interface with
// `interface_config` from `joint_groups_by_name` and `ecm`.
//
// Uses `hwm_name` and `interface_name` for nicer log messages.
//
// Returns NotFoundError if `interface_config` refers to a joint group that is
// not present in `joint_groups_by_name`.
absl::StatusOr<JointCommandedPositionData> GetJointCommandedPositionDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointCommandedPositionInterface&
        interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    gz::sim::EntityComponentManager& ecm);

// Extracts the interface data for a JointVelocityState interface with
// `interface_config` from `joint_groups_by_name` and `ecm`.
//
// Adds the `JointVelocity` Gazebo component to all joint entities in the
// selected joint group. This lets Gazebo know that it should populate the
// component during the physics update step, so we can read the velocities and
// report them to ICON.
//
// Uses `hwm_name` and `interface_name` for nicer log messages.
//
// Returns NotFoundError if `interface_config` refers to a joint group that is
// not present in `joint_groups_by_name`.
absl::StatusOr<JointVelocityStateData> GetJointVelocityStateDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointVelocityStateInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    gz::sim::EntityComponentManager& ecm);

// Extracts the interface data for a JointAccelerationState interface with
// `interface_config` from `joint_groups_by_name` and `ecm`.
//
// Unlike the position, velocity and torque state interfaces, there is actually
// no Gazebo component to read acceleration from. So we cannot add one, and the
// JointAccelerationState interface is confined to essentially being a no-op.
//
// Uses `hwm_name` and `interface_name` for nicer log messages.
//
// Returns NotFoundError if `interface_config` refers to a joint group that is
// not present in `joint_groups_by_name`.
absl::StatusOr<JointAccelerationStateData> GetJointAccelerationStateDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointAccelerationStateInterface&
        interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm);

// Extracts the interface data for a JointTorqueState interface with
// `interface_config` from `joint_groups_by_name` and `ecm`.
//
// Adds the `JointTorque` Gazebo component to all joint entities in the
// selected joint group. This lets Gazebo know that it should populate the
// component during the physics update step, so we can read the torques and
// report them to ICON.
//
// Uses `hwm_name` and `interface_name` for nicer log messages.
//
// Returns NotFoundError if `interface_config` refers to a joint group that is
// not present in `joint_groups_by_name`.
absl::StatusOr<JointTorqueStateData> GetJointTorqueStateDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointTorqueStateInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    gz::sim::EntityComponentManager& ecm);

// Extracts the interface data for a KinematicChainPayloadCommand interface with
// `interface_config` from `joint_groups_by_name` and `ecm`.
//
// Uses `hwm_name` and `interface_name` for nicer log messages.
//
// Returns NotFoundError if `interface_config` refers to a joint group that is
// not present in `joint_groups_by_name`.
absl::StatusOr<KinematicChainPayloadCommandData>
GetKinematicChainPayloadCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::KinematicChainPayloadCommandInterface&
        interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm);

// Extracts the interface data for a KinematicChainPayloadState interface with
// `interface_config` from `joint_groups_by_name` and `ecm`.
//
// Uses `hwm_name` and `interface_name` for nicer log messages.
//
// Returns NotFoundError if `interface_config` refers to a joint group that is
// not present in `joint_groups_by_name`.
absl::StatusOr<KinematicChainPayloadStateData>
GetKinematicChainPayloadStateDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::KinematicChainPayloadStateInterface&
        interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm);

// Extracts the interface data for a KinematicChainProcessWrenchCommand
// interface with `interface_config` from `joint_groups_by_name` and `ecm`.
//
// Uses `hwm_name` and `interface_name` for nicer log messages.
//
// Returns NotFoundError if `interface_config` refers to a joint group that is
// not present in `joint_groups_by_name`.
absl::StatusOr<KinematicChainProcessWrenchCommandData>
GetKinematicChainProcessWrenchCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::KinematicChainProcessWrenchCommandInterface&
        interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm);

// Extracts the interface data for a DigitalInputStatus interface with
// `interface_config` from `ecm`.
//
// Uses `hwm_name` and `interface_name` for nicer log messages.
//
// `DigitalInputInterface` has two options for configuration:
// * Set only `name_of_object_with_digital_input`
// * Set both `name_of_object_with_digital_input` and `input_block_name`
//
// In either case, this function first looks up the Entity (or Entities) for
// `name_of_object_with_digital_input` (as indicated by the
// `intrinsic::simulation::ResourceName` component). It then goes through the
// children of those Entities, specifically those children that have a
// `intrinsic::simulation::DigitalInput` component.
//
// If `input_block_name` is set, the function searches for a child that has a
// `gz::sim::components::Name` component with that name.
//
// If `input_block_name` is *not* set, the function checks whether there is
// exactly one DigitalInput child entity. If so, it picks that entity, but if
// there is more than one such entity, it returns an error.
//
// Returns InvalidArgumentError if
// `interface_config.name_of_object_with_digital_input` is *unset*, but
// `interface_config.input_block_name` *is* set.
// Returns NotFoundError if there are no entities with the World name
// `interface_config.name_of_object_with_digital_input`.
// Returns NotFoundError if ìnterface_config.input_block_name` is set, but there
// is no input block with that name in any of the entities with the name
// `interface_config.name_of_object_with_digital_input`.
// Returns InternalError name if there are any DigitalInput entities without a
// name.
// Returns FailedPreconditionError if there are multiple input blocks named
// `interface_config.input_block_name`.
absl::StatusOr<DigitalInputStatusData> GetDigitalInputStatusDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::DigitalInputInterface& interface_config,
    const gz::sim::EntityComponentManager& ecm);

// Extracts the interface data for a DigitalOutputCommand interface with
// `interface_config` from `ecm`.
//
// Uses `hwm_name` and `interface_name` for nicer log messages.
//
// `DigitalOutputInterface` has two options for configuration:
// * Set only `name_of_object_with_digital_output`
// * Set both `name_of_object_with_digital_output` and `output_block_name`
//
// In either case, this function first looks up the Entity (or Entities) for
// `name_of_object_with_digital_output` (as indicated by the
// `intrinsic::simulation::ResourceName` component). It then goes through the
// children of those Entities, specifically those children that have a
// `intrinsic::simulation::DigitalOutput` component.
//
// If `output_block_name` is set, the function searches for a child that has a
// `gz::sim::components::Name` component with that name.
//
// If `output_block_name` is *not* set, the function checks whether there is
// exactly one DigitalOutput child entity. If so, it picks that entity, but if
// there is more than one such entity, it returns an error.
//
// Returns InvalidArgumentError if
// `interface_config.name_of_object_with_digital_output` is *unset*, but
// `interface_config.output_block_name` *is* set.
// Returns NotFoundError if there are no entities with the World name
// `interface_config.name_of_object_with_digital_output`.
// Returns NotFoundError if ìnterface_config.output_block_name` is set, but
// there is no output block with that name in any of the entities with the name
// `interface_config.name_of_object_with_digital_output`.
// Returns InternalError name if there are any Digitaloutput entities without a
// name.
// Returns FailedPreconditionError if there are multiple output blocks named
// `interface_config.output_block_name`.
absl::StatusOr<DigitalOutputCommandData> GetDigitalOutputCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::DigitalOutputInterface& interface_config,
    const gz::sim::EntityComponentManager& ecm);

// Extracts the interface data for a ForceTorqueCommand interface with
// `interface_config` from `ecm`.
//
// Returns InvalidArgumentError if
// `interface_config.name_of_object_with_force_torque_sensor` is unset.
// Returns NotFoundError if there are no entities with the World name
// `interface_config.name_of_object_with_force_torque_sensor`.
// If `interface_config.ft_sensor_name` is set, returns NotFoundError if none of
// the entities with the World name
// `interface_config.name_of_object_with_force_torque_sensor` have a sensor
// entity named `interface_config.ft_sensor_name`.
// If `interface_config.ft_sensor_name` is *not* set, returns NotFoundError if
// none of the entities with the World name have any force/torque sensor
// entities.
// Returns FailedPreconditionError if the entities with the World name and
// (if present) `interface_config.ft_sensor_name` have more than one
// force/torque sensor entity between them.
absl::StatusOr<ForceTorqueCommandData> GetForceTorqueCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::ForceTorqueCommandInterface& interface_config,
    const gz::sim::EntityComponentManager& ecm);

// Extracts the interface data for a ForceTorqueStatus interface with
// `interface_config` from `ecm`.
//
// Adds the `WrenchMeasured` Gazebo component to the sensor entity. This lets
// Gazebo know that it should populate the component during the physics update
// step, so we can read the F/T readings and report them to ICON.
//
// Returns InvalidArgumentError if
// `interface_config.name_of_object_with_force_torque_sensor` is unset.
// Returns NotFoundError if there are no entities with the World name
// `interface_config.name_of_object_with_force_torque_sensor`.
// If `interface_config.ft_sensor_name` is set, returns NotFoundError if none of
// the entities with the World name
// `interface_config.name_of_object_with_force_torque_sensor` have a sensor
// entity named `interface_config.ft_sensor_name`.
// If `interface_config.ft_sensor_name` is *not* set, returns NotFoundError if
// none of the entities with the World name have any force/torque sensor
// entities.
// Returns FailedPreconditionError if the entities with the World name and
// (if present) `interface_config.ft_sensor_name` have more than one
// force/torque sensor entity between them.
absl::StatusOr<ForceTorqueStatusData> GetForceTorqueStatusDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::ForceTorqueStatusInterface& interface_config,
    gz::sim::EntityComponentManager& ecm);

// Extracts the interface data for a JointLimitsCommand interface with
// `interface_config` from `joint_groups_by_name` and `ecm`.
//
// Uses `hwm_name` and `interface_name` for nicer log messages.
//
// Returns NotFoundError if `interface_config` refers to a joint group that is
// not present in `joint_groups_by_name`.
absl::StatusOr<JointLimitsCommandData> GetJointLimitsCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointLimitsCommandInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    gz::sim::EntityComponentManager& ecm);

// Extracts the interface data for a RangefinderStatus interface with
// `interface_config` from `ecm`.
//
// Returns InvalidArgumentError if
// `interface_config.name_of_object_with_rangefinder` is unset.
// Returns NotFoundError if there are no entities with the World name
// `interface_config.name_of_object_with_rangefinder`.
// If `interface_config.rangefinder_name` is set, returns NotFoundError if none
// of the entities with the World name
// `interface_config.name_of_object_with_rangefinder` have a sensor
// entity named `interface_config.rangefinder_name`.
// If `interface_config.rangefinder_name` is *not* set, returns NotFoundError if
// none of the entities with the World name have any rangefinder sensor
// entities.
// Returns FailedPreconditionError if the entities with the World name and
// (if present) `interface_config.rangefinder_name` have more than one
// rangefinder sensor entity between them.
// Returns NotFoundError if the rangefinder sensor entity has no GpuLidar
// component.
absl::StatusOr<RangefinderStatusData> GetRangefinderStatusDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::RangefinderStatusInterface& interface_config,
    const gz::sim::EntityComponentManager& ecm);

}  // namespace intrinsic::simulation::hardware_interface_data

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SIM_HARDWARE_INTERFACE_DATA_H_
