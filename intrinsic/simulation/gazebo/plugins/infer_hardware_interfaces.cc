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

#include "intrinsic/simulation/gazebo/plugins/infer_hardware_interfaces.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_map.h"
#include "absl/log/log.h"
#include "absl/random/distributions.h"
#include "absl/random/random.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "intrinsic/icon/hal/proto/hardware_module_config.pb.h"
#include "intrinsic/icon/hardware_modules/sim_bus/sim_bus_hardware_module.pb.h"
#include "intrinsic/icon/server/config/dio_config.pb.h"
#include "intrinsic/scene/sdf/custom_tags.h"
#include "intrinsic/simulation/gazebo/asset_instances_client.h"
#include "intrinsic/simulation/gazebo/components/digital_io_components.h"
#include "intrinsic/simulation/gazebo/components/ppr_component.h"
#include "intrinsic/simulation/gazebo/plugins/sim_hardware_module_config.pb.h"
#include "intrinsic/util/status/status_macros.h"
#include "sdf/Element.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Param.hh"
#include "sdf/Sensor.hh"
#include "sdf/Types.hh"

namespace intrinsic::simulation {
namespace {

// Returns a tailored advice string for the user when a ModelSdf component
// cannot be found, based on the configuration method used in `sim_config`.
std::string GetSdfNotFoundErrorAdvice(
    const ::intrinsic_proto::sim::SimHardwareModuleConfig& sim_config) {
  switch (sim_config.hardware_interfaces_case()) {
    case ::intrinsic_proto::sim::SimHardwareModuleConfig::kGeometryAssetName:
      return absl::StrCat(
          "Check that `geometry_asset_name` ('",
          sim_config.geometry_asset_name(),
          "') is a valid asset name and contains an SDF model.");
    case ::intrinsic_proto::sim::SimHardwareModuleConfig::kFromInstances:
      return "Check that all instances in `from_instances` are valid asset "
             "names and contain SDF models.";
    default:
      return "If this HWM relies on a separate geometry asset, specify it in "
             "`geometry_asset_name` or `from_instances`.";
  }
}

// Retrieves the ModelSdf component for the given `model` from the `ecm`.
// This function returns a NotFoundError with advice derived from `sim_config`
// if the component is missing for the hardware module `hwm_name`.
absl::StatusOr<const gz::sim::components::ModelSdf*> GetModelSdf(
    const ::gz::sim::Model& model, absl::string_view hwm_name,
    const ::gz::sim::EntityComponentManager& ecm,
    const ::intrinsic_proto::sim::SimHardwareModuleConfig& sim_config) {
  const auto* model_sdf_comp =
      ecm.Component<gz::sim::components::ModelSdf>(model.Entity());
  if (model_sdf_comp == nullptr) {
    return absl::NotFoundError(
        absl::StrCat("Failed to get ModelSdf component for model '",
                     model.Name(ecm), "' while trying to set up HWM '",
                     hwm_name, "'. ", GetSdfNotFoundErrorAdvice(sim_config)));
  }
  return model_sdf_comp;
}

// Adds a joint group and its corresponding joint-related hardware interfaces
// for the given `object_with_joints_name` to the `sim_hwm_config`. This
// function returns an AlreadyExistsError if a joint group with the automatic
// name already exists.
absl::Status PopulateInferredJointGroupAndInterfaces(
    absl::string_view object_with_joints_name,
    ::intrinsic_proto::sim::SimHardwareModuleConfig& sim_hwm_config) {
  // First, add a joint group to `sim_hwm_config`, using a well-known name.
  intrinsic_proto::sim::JointGroup joint_group;
  joint_group.set_name_of_object_with_joints_in_implicit_order(
      object_with_joints_name);
  if (auto it_and_inserted =
          sim_hwm_config.mutable_manual_hardware_interfaces()
              ->mutable_joint_groups()
              ->try_emplace(automatic_interface_names::kAutomaticJointGroupName,
                            joint_group);
      !it_and_inserted.second) {
    // If the JointGroup already exists, we've already added joint interfaces.
    // Bail out!
    return absl::AlreadyExistsError(absl::StrCat(
        "Found more than one model with joints for simulated hardware module '",
        object_with_joints_name,
        "'. Please manually configure this simulated hardware module."));
  }

  // Add the SimHardwareInterface protos for all joint-related hardware
  // interfaces.
  // This code can overwrite existing interfaces in theory, but in practice we
  // know that if there isn't a JointGroup with the well-known name already, we
  // haven't added interfaces yet, either.
  (*sim_hwm_config.mutable_manual_hardware_interfaces()
        ->mutable_name_to_interface())
      [automatic_interface_names::kAutomaticJointPositionCommandInterfaceName]
          .mutable_joint_position_command()
          ->set_joint_group_name(
              automatic_interface_names::kAutomaticJointGroupName);
  (*sim_hwm_config.mutable_manual_hardware_interfaces()
        ->mutable_name_to_interface())
      [automatic_interface_names::kAutomaticJointTorqueCommandInterfaceName]
          .mutable_joint_torque_command()
          ->set_joint_group_name(
              automatic_interface_names::kAutomaticJointGroupName);
  (*sim_hwm_config.mutable_manual_hardware_interfaces()
        ->mutable_name_to_interface())
      [automatic_interface_names::kAutomaticJointPositionStateInterfaceName]
          .mutable_joint_position_state()
          ->set_joint_group_name(
              automatic_interface_names::kAutomaticJointGroupName);
  (*sim_hwm_config.mutable_manual_hardware_interfaces()
        ->mutable_name_to_interface())
      [automatic_interface_names::kAutomaticJointCommandedPositionInterfaceName]
          .mutable_joint_commanded_position()
          ->set_joint_group_name(
              automatic_interface_names::kAutomaticJointGroupName);
  (*sim_hwm_config.mutable_manual_hardware_interfaces()
        ->mutable_name_to_interface())
      [automatic_interface_names::kAutomaticJointVelocityStateInterfaceName]
          .mutable_joint_velocity_state()
          ->set_joint_group_name(
              automatic_interface_names::kAutomaticJointGroupName);
  (*sim_hwm_config.mutable_manual_hardware_interfaces()
        ->mutable_name_to_interface())
      [automatic_interface_names::kAutomaticJointAccelerationStateInterfaceName]
          .mutable_joint_acceleration_state()
          ->set_joint_group_name(
              automatic_interface_names::kAutomaticJointGroupName);
  (*sim_hwm_config.mutable_manual_hardware_interfaces()
        ->mutable_name_to_interface())
      [automatic_interface_names::kAutomaticJointTorqueStateInterfaceName]
          .mutable_joint_torque_state()
          ->set_joint_group_name(
              automatic_interface_names::kAutomaticJointGroupName);
  (*sim_hwm_config.mutable_manual_hardware_interfaces()
        ->mutable_name_to_interface())
      [automatic_interface_names::kAutomaticPayloadCommandInterfaceName]
          .mutable_payload_command()
          ->set_joint_group_name(
              automatic_interface_names::kAutomaticJointGroupName);
  (*sim_hwm_config.mutable_manual_hardware_interfaces()
        ->mutable_name_to_interface())
      [automatic_interface_names::kAutomaticPayloadStateInterfaceName]
          .mutable_payload_state()
          ->set_joint_group_name(
              automatic_interface_names::kAutomaticJointGroupName);
  (*sim_hwm_config.mutable_manual_hardware_interfaces()
        ->mutable_name_to_interface())
      [automatic_interface_names::kAutomaticProcessWrenchCommandInterfaceName]
          .mutable_process_wrench_command()
          ->set_joint_group_name(
              automatic_interface_names::kAutomaticJointGroupName);
  (*sim_hwm_config.mutable_manual_hardware_interfaces()
        ->mutable_name_to_interface())
      [automatic_interface_names::kAutomaticJointLimitsCommandInterfaceName]
          .mutable_joint_limits_command()
          ->set_joint_group_name(
              automatic_interface_names::kAutomaticJointGroupName);
  return absl::OkStatus();
}

// Adds force/torque sensor hardware interfaces for the provided
// `object_with_ft_sensor_name` to the `sim_hwm_config`. This function returns
// an AlreadyExistsError if the force/torque interfaces have already been
// defined.
absl::Status PopulateInferredForceTorqueSensorInterfaces(
    absl::string_view object_with_ft_sensor_name,
    ::intrinsic_proto::sim::SimHardwareModuleConfig& sim_hwm_config) {
  if (sim_hwm_config.manual_hardware_interfaces().name_to_interface().contains(
          automatic_interface_names::
              kAutomaticForceTorqueCommandInterfaceName) ||
      sim_hwm_config.manual_hardware_interfaces().name_to_interface().contains(
          automatic_interface_names::
              kAutomaticForceTorqueStatusInterfaceName)) {
    return absl::AlreadyExistsError(absl::StrCat(
        "Found more than one model with a force/torque sensor for simulated "
        "hardware module '",
        object_with_ft_sensor_name,
        "'. Please manually configure this simulated hardware module."));
  }
  (*sim_hwm_config.mutable_manual_hardware_interfaces()
        ->mutable_name_to_interface())
      [automatic_interface_names::kAutomaticForceTorqueCommandInterfaceName]
          .mutable_force_torque_command()
          ->set_name_of_object_with_force_torque_sensor(
              object_with_ft_sensor_name);
  (*sim_hwm_config.mutable_manual_hardware_interfaces()
        ->mutable_name_to_interface())
      [automatic_interface_names::kAutomaticForceTorqueStatusInterfaceName]
          .mutable_force_torque_status()
          ->set_name_of_object_with_force_torque_sensor(
              object_with_ft_sensor_name);
  return absl::OkStatus();
}

// Adds rangefinder hardware interfaces for the given `object_with_rangefinder`
// to the `sim_hwm_config`. This function returns an AlreadyExistsError if the
// rangefinder interface already exists.
absl::Status PopulateInferredRangefinderInterfaces(
    absl::string_view object_with_rangefinder,
    ::intrinsic_proto::sim::SimHardwareModuleConfig& sim_hwm_config) {
  if (sim_hwm_config.manual_hardware_interfaces().name_to_interface().contains(
          automatic_interface_names::
              kAutomaticRangefinderStatusInterfaceName)) {
    return absl::AlreadyExistsError(absl::StrCat(
        "Found more than one model with a rangefinder for simulated "
        "hardware module '",
        object_with_rangefinder,
        "'. Please manually configure this simulated hardware module."));
  }
  (*sim_hwm_config.mutable_manual_hardware_interfaces()
        ->mutable_name_to_interface())
      [automatic_interface_names::kAutomaticRangefinderStatusInterfaceName]
          .mutable_rangefinder_status()
          ->set_name_of_object_with_rangefinder(object_with_rangefinder);
  return absl::OkStatus();
}

enum class DigitalIoInterfaceDirection {
  kStatus,
  kCommand,
};

// Determines the hardware interface name for this digital I/O block:
// * If `io_block_configs` has an entry for this input block, and that entry
//   prescribes a `{command,status}_interface_name_sim_only`. If that's the
//   case, we use that name.
// * Else, if `is_legacy` is true we append `backwards_compatible_name_suffix`
//   to the block name.
// * If `io_block_configs` *doesn't* have an entry for this input block, and
//   `is_legacy` is false, we just use the input block's name as the hardware
//   interface name.
std::string GetHardwareInterfaceNameForDioBlock(
    absl::Span<const ::intrinsic_proto::icon::DigitalIO* const>
        io_block_configs,
    absl::string_view io_block_name, bool is_legacy,
    DigitalIoInterfaceDirection io_block_direction,
    absl::string_view backwards_compatible_name_suffix) {
  auto matching_config = absl::c_find_if(
      io_block_configs,
      [&](const ::intrinsic_proto::icon::DigitalIO* const block) {
        return block->device_name() == io_block_name;
      });
  if (matching_config != io_block_configs.end()) {
    const ::intrinsic_proto::icon::DigitalIO& block_config = **matching_config;
    switch (io_block_direction) {
      case DigitalIoInterfaceDirection::kStatus:
        if (!block_config.status_interface_name_sim_only().empty()) {
          return block_config.status_interface_name_sim_only();
        }
        break;
      case DigitalIoInterfaceDirection::kCommand:
        if (!block_config.command_interface_name_sim_only().empty()) {
          return block_config.command_interface_name_sim_only();
        }
        break;
    }
  }
  if (is_legacy) {
    return absl::StrCat(io_block_name, backwards_compatible_name_suffix);
  }
  return std::string(io_block_name);
}

// Builds a map from bit index to bit alias for a digital I/O block.
absl::flat_hash_map<size_t, std::string> GetBitAliasMapForDioBlock(
    absl::Span<const ::intrinsic_proto::icon::DigitalIO* const>
        io_block_configs,
    absl::string_view io_block_name) {
  auto matching_config = absl::c_find_if(
      io_block_configs,
      [&](const ::intrinsic_proto::icon::DigitalIO* const block) {
        return block->device_name() == io_block_name;
      });
  absl::flat_hash_map<size_t, std::string> bit_index_to_alias;
  if (matching_config == io_block_configs.end()) {
    return bit_index_to_alias;
  }
  for (const auto& signal : (*matching_config)->signals()) {
    bit_index_to_alias[signal.bit()] = signal.name();
  }
  return bit_index_to_alias;
}

// Adds digital I/O hardware interfaces to `sim_hwm_config` for all digital I/O
// entities in `child_entities`.
absl::Status PopulateInferredDigitalIoInterfaces(
    absl::string_view object_with_digital_ios_name,
    gz::sim::Entity parent_entity,
    absl::Span<const gz::sim::Entity> child_entities,
    const ::intrinsic_proto::icon::DigitalIOs& dio_config,
    ::gz::sim::EntityComponentManager& ecm,
    ::intrinsic_proto::sim::SimHardwareModuleConfig& sim_hwm_config) {
  for (const gz::sim::Entity child : child_entities) {
    if (const intrinsic::simulation::DigitalInput* input_component =
            ecm.Component<intrinsic::simulation::DigitalInput>(child);
        input_component != nullptr) {
      std::optional<std::string> input_name =
          ecm.ComponentData<gz::sim::components::Name>(child);
      if (input_name == std::nullopt) {
        // The DigitalInputOutput plugin should have assigned a name to each
        // input/output!
        LOG(ERROR) << "Simulated hardware module '"
                   << object_with_digital_ios_name
                   << "' has a DigitalInput entity without a name. Skipping "
                      "this input and continuing. Please report this as a bug.";
        continue;
      }
      std::string interface_name = GetHardwareInterfaceNameForDioBlock(
          absl::MakeConstSpan(dio_config.inputs()), input_name.value(),
          input_component->Data().is_legacy,
          DigitalIoInterfaceDirection::kStatus, "_digital_input_status");
      if (sim_hwm_config.manual_hardware_interfaces()
              .name_to_interface()
              .contains(interface_name)) {
        return absl::AlreadyExistsError(absl::StrCat(
            "Found more than one digital input block with the interface name '",
            interface_name, "' for simulated hardware module '",
            object_with_digital_ios_name,
            "'. Please manually configure this simulated hardware module."));
      }
      ::intrinsic_proto::sim::DigitalInputInterface*
          input_status_interface_config =
              (*sim_hwm_config.mutable_manual_hardware_interfaces()
                    ->mutable_name_to_interface())[interface_name]
                  .mutable_digital_input();
      input_status_interface_config->set_name_of_object_with_digital_input(
          object_with_digital_ios_name);
      input_status_interface_config->set_input_block_name(input_name.value());
      for (const auto& [bit_index, alias] : GetBitAliasMapForDioBlock(
               absl::MakeConstSpan(dio_config.inputs()), input_name.value())) {
        input_status_interface_config->mutable_bit_number_to_alias()->emplace(
            bit_index, alias);
      }
    }
    if (const intrinsic::simulation::DigitalOutput* output_component =
            ecm.Component<DigitalOutput>(child);
        output_component != nullptr) {
      std::optional<std::string> output_name =
          ecm.ComponentData<gz::sim::components::Name>(child);
      if (output_name == std::nullopt) {
        // The DigitalInputOutput plugin should have assigned a name to each
        // input/output!
        LOG(ERROR)
            << "Simulated hardware module '" << object_with_digital_ios_name
            << "' has a DigitalOutput entity without a name. Skipping "
               "this output and continuing. Please report this as a bug.";
        continue;
      }
      // If this output is legacy, generate a status interface too.
      // Newer digital outputs shouldn't have this config, and shouldn't have
      // status interfaces.
      if (output_component->Data().is_legacy) {
        std::string interface_name = GetHardwareInterfaceNameForDioBlock(
            absl::MakeConstSpan(dio_config.outputs()), output_name.value(),
            output_component->Data().is_legacy,
            DigitalIoInterfaceDirection::kStatus, "_digital_output_status");
        if (sim_hwm_config.manual_hardware_interfaces()
                .name_to_interface()
                .contains(interface_name)) {
          return absl::AlreadyExistsError(absl::StrCat(
              "Found more than one digital output block with the interface "
              "name '",
              interface_name, "' for simulated hardware module '",
              object_with_digital_ios_name,
              "'. Please manually configure this simulated hardware module."));
        }
        // Add a sibling entity with a unique name, and give it a DigitalInput
        std::string sibling_entity_name = absl::StrCat(
            output_name.value(), "_autogenerated_input_",
            // NOLINTNEXTLINE(runtime/expensive_random_temporaries)<
            absl::Hex(absl::Uniform<uint64_t>(absl::BitGen())));
        gz::sim::Entity sibling_entity = ecm.CreateEntity();
        LOG(INFO) << "Added implicit input status for legacy DigitalOutput '"
                  << output_name.value() << "'. Unique entity name: '"
                  << sibling_entity_name << "', Entity ID: " << sibling_entity;
        if (!ecm.SetParentEntity(sibling_entity, parent_entity)) {
          ecm.RequestRemoveEntity(sibling_entity);
          return absl::InternalError(absl::StrCat(
              "Failed to set parent/child relation for auto-generated "
              "digital input entity for output block '",
              output_name.value(), "' in World object '",
              object_with_digital_ios_name, "'"));
        }

        {
          // Initialize the DigitalInput component with all-false bits.
          // Don't change the value of `is_legacy` if it exists already, and
          // don't set it otherwise (it defaults to `false`).
          auto* digital_input_component =
              ecm.ComponentDefault<DigitalInput>(sibling_entity);
          digital_input_component->Data().data =
              std::vector<bool>(output_component->Data().data.size(), false);
        }
        // Add ParentEntity and Name Components. These are important for
        // consumers that want to use the ECM, rather than the gz-transport
        // topic, to interact with the input.
        ecm.SetComponentData<gz::sim::components::ParentEntity>(sibling_entity,
                                                                parent_entity);
        ecm.SetComponentData<gz::sim::components::Name>(sibling_entity,
                                                        sibling_entity_name);
        ::intrinsic_proto::sim::DigitalInputInterface* input_interface_config =
            (*sim_hwm_config.mutable_manual_hardware_interfaces()
                  ->mutable_name_to_interface())[interface_name]
                .mutable_digital_input();
        input_interface_config->set_name_of_object_with_digital_input(
            object_with_digital_ios_name);
        input_interface_config->set_input_block_name(sibling_entity_name);
        for (const auto& [bit_index, alias] : GetBitAliasMapForDioBlock(
                 absl::MakeConstSpan(dio_config.outputs()),
                 output_name.value())) {
          input_interface_config->mutable_bit_number_to_alias()->emplace(
              bit_index, alias);
        }
      }

      // Otherwise, just have a command interface for the output block.
      std::string interface_name = GetHardwareInterfaceNameForDioBlock(
          absl::MakeConstSpan(dio_config.outputs()), output_name.value(),
          output_component->Data().is_legacy,
          DigitalIoInterfaceDirection::kCommand, "_digital_output_command");
      if (sim_hwm_config.manual_hardware_interfaces()
              .name_to_interface()
              .contains(interface_name)) {
        return absl::AlreadyExistsError(absl::StrCat(
            "Found more than one digital output block with the interface name "
            "'",
            interface_name, "' for simulated hardware module '",
            object_with_digital_ios_name,
            "'. Please manually configure this simulated hardware module."));
      }
      ::intrinsic_proto::sim::DigitalOutputInterface*
          output_command_interface_config =
              (*sim_hwm_config.mutable_manual_hardware_interfaces()
                    ->mutable_name_to_interface())[interface_name]
                  .mutable_digital_output();
      output_command_interface_config->set_name_of_object_with_digital_output(
          object_with_digital_ios_name);
      output_command_interface_config->set_output_block_name(
          output_name.value());
      for (const auto& [bit_index, alias] :
           GetBitAliasMapForDioBlock(absl::MakeConstSpan(dio_config.outputs()),
                                     output_name.value())) {
        output_command_interface_config->mutable_bit_number_to_alias()->emplace(
            bit_index, alias);
      }
    }
  }
  return absl::OkStatus();
}

// Adds analog I/O hardware interfaces to `sim_hwm_config`.
absl::Status PopulateInferredAnalogIoInterfaces(
    absl::string_view hwm_resource_name,
    absl::Span<const ::intrinsic_proto::icon::DeviceConfig* const>
        device_configs,
    ::intrinsic_proto::sim::SimHardwareModuleConfig& sim_hwm_config) {
  for (const ::intrinsic_proto::icon::DeviceConfig* const device_config :
       device_configs) {
    if (device_config->has_analog_input()) {
      auto aio_status_interface_name =
          absl::StrCat(device_config->name(), "_status");
      if (!device_config->analog_input().interface_name().empty()) {
        aio_status_interface_name =
            device_config->analog_input().interface_name();
      }
      if (sim_hwm_config.manual_hardware_interfaces()
              .name_to_interface()
              .contains(aio_status_interface_name)) {
        return absl::AlreadyExistsError(absl::StrCat(
            "Found more than one analog input block with the interface name '",
            aio_status_interface_name, "' for simulated hardware module '",
            hwm_resource_name,
            "'. Please manually configure this simulated hardware module."));
      }
      (*sim_hwm_config.mutable_manual_hardware_interfaces()
            ->mutable_name_to_interface())[aio_status_interface_name]
          .mutable_analog_input()
          ->set_num_inputs(device_config->analog_input().num_inputs());
    }
    if (device_config->has_analog_output()) {
      auto aio_command_interface_name =
          absl::StrCat(device_config->name(), "_command");
      if (!device_config->analog_output().interface_name().empty()) {
        aio_command_interface_name =
            device_config->analog_output().interface_name();
      }
      if (sim_hwm_config.manual_hardware_interfaces()
              .name_to_interface()
              .contains(aio_command_interface_name)) {
        return absl::AlreadyExistsError(absl::StrCat(
            "Found more than one analog output block with the interface name '",
            aio_command_interface_name, "' for simulated hardware module '",
            hwm_resource_name,
            "'. Please manually configure this simulated hardware module."));
      }
      (*sim_hwm_config.mutable_manual_hardware_interfaces()
            ->mutable_name_to_interface())[aio_command_interface_name]
          .mutable_analog_output()
          ->set_num_outputs(device_config->analog_output().num_outputs());
    }
  }

  return absl::OkStatus();
}

int CountRevoluteAndPrismaticJoints(const ::sdf::Model& model_sdf) {
  int count = 0;
  for (std::size_t j = 0; j < model_sdf.JointCount(); ++j) {
    const ::sdf::Joint* joint_sdf = model_sdf.JointByIndex(j);
    if (joint_sdf->Type() == ::sdf::JointType::REVOLUTE ||
        joint_sdf->Type() == ::sdf::JointType::PRISMATIC) {
      ++count;
    }
  }
  return count;
}

int CountForceTorqueSensors(const ::sdf::Model& model_sdf) {
  int count = 0;
  for (std::size_t j = 0; j < model_sdf.JointCount(); ++j) {
    const ::sdf::Joint* joint_sdf = model_sdf.JointByIndex(j);
    // Check for force-torque sensors, which can be attached to multiple joint
    // types, including fixed and revolute joints.
    for (std::size_t s = 0; s < joint_sdf->SensorCount(); ++s) {
      const ::sdf::Sensor* sensor_sdf = joint_sdf->SensorByIndex(s);
      if (sensor_sdf->Type() == ::sdf::SensorType::FORCE_TORQUE) {
        ++count;
      }
    }
  }
  return count;
}

int CountRangefinders(const ::sdf::Model& model_sdf) {
  int count = 0;
  for (std::size_t l = 0; l < model_sdf.LinkCount(); ++l) {
    const ::sdf::Link* link_sdf = model_sdf.LinkByIndex(l);
    // Check for rangefinder sensors, which can be attached to links.
    for (std::size_t s = 0; s < link_sdf->SensorCount(); ++s) {
      const ::sdf::Sensor* sensor_sdf = link_sdf->SensorByIndex(s);
      if (sensor_sdf->Type() == ::sdf::SensorType::GPU_LIDAR) {
        ++count;
      }
    }
  }
  return count;
}

// Populates a `SimHardwareModuleConfig` by iterating through Gazebo entities
// in the `ecm` matching `hwm_resource_name`, inspecting their SDF models, and
// inferring hardware interfaces like joint groups, force/torque and
// rangefinder sensors, and digital I/O interfaces using the provided
// `dio_config` and `additional_device_configs`.
absl::StatusOr<::intrinsic_proto::sim::SimHardwareModuleConfig>
InferSimHardwareModuleConfigImpl(
    absl::string_view hwm_resource_name,
    const ::intrinsic_proto::icon::DigitalIOs& dio_config,
    absl::Span<const ::intrinsic_proto::icon::DeviceConfig* const>
        additional_device_configs,
    ::gz::sim::EntityComponentManager& ecm) {
  ::intrinsic_proto::sim::SimHardwareModuleConfig sim_hwm_config;
  // First, find all Gazebo entities that are related to the resource we care
  // about.
  auto entities_with_resource_name =
      ecm.EntitiesByComponents(ResourceName{std::string(hwm_resource_name)});
  if (entities_with_resource_name.empty()) {
    return absl::NotFoundError(absl::StrCat(
        "There are no entities with the resource name ", hwm_resource_name));
  }
  for (const gz::sim::Entity& entity : entities_with_resource_name) {
    // Read from the ModelSdf component to ensure Joints are in the right
    // order.
    auto model_sdf = ecm.ComponentData<gz::sim::components::ModelSdf>(entity);
    auto model_entity_name =
        ecm.ComponentData<gz::sim::components::Name>(entity).value_or(
            "[UNKNOWN]");
    if (!model_sdf.has_value()) {
      LOG(INFO) << "Failed to get ModelSdf component for entity "
                << model_entity_name << ". Skipping...";
      continue;
    }
    // Handle any joints
    if (CountRevoluteAndPrismaticJoints(model_sdf.value()) > 0) {
      INTR_RETURN_IF_ERROR(PopulateInferredJointGroupAndInterfaces(
          hwm_resource_name, sim_hwm_config));
    }
    // Handle any F/T sensors
    {
      int num_ft_sensors = CountForceTorqueSensors(model_sdf.value());
      if (num_ft_sensors > 1) {
        return absl::FailedPreconditionError(absl::StrCat(
            "SDF for entity '", model_entity_name, "' (for HWM '",
            hwm_resource_name,
            "') has more than one Force/Torque sensor. Automatic "
            "configuration can only handle one Force/Torque sensor, "
            "please manually configure your hardware module's interfaces."));
      }
      if (num_ft_sensors == 1) {
        INTR_RETURN_IF_ERROR(PopulateInferredForceTorqueSensorInterfaces(
            hwm_resource_name, sim_hwm_config));
      }
    }
    // Handle any rangefinders
    {
      int num_rangefinders = CountRangefinders(model_sdf.value());
      if (num_rangefinders > 1) {
        return absl::FailedPreconditionError(absl::StrCat(
            "SDF for entity '", model_entity_name, "' (for HWM '",
            hwm_resource_name,
            "') has more than one rangefinder. Automatic "
            "configuration can only handle one rangefinder, "
            "please manually configure your hardware module's interfaces."));
      }
      if (num_rangefinders == 1) {
        INTR_RETURN_IF_ERROR(PopulateInferredRangefinderInterfaces(
            hwm_resource_name, sim_hwm_config));
      }
    }
    // Handle any DIOs
    INTR_RETURN_IF_ERROR(PopulateInferredDigitalIoInterfaces(
        hwm_resource_name, entity, ecm.ChildrenByComponents(entity), dio_config,
        ecm, sim_hwm_config));
    // Finally, copy any extra devices from the legacy SimBusHardwareModule
    // config (well, Analog I/Os, at least)
    INTR_RETURN_IF_ERROR(PopulateInferredAnalogIoInterfaces(
        hwm_resource_name, additional_device_configs, sim_hwm_config));
  }
  return sim_hwm_config;
}

// Finds and returns Gazebo entities in the `ecm` that have model data
// corresponding to the specified `world_object_name`.
std::vector<gz::sim::Entity> FindEntitiesWithModelDataForWorldName(
    const gz::sim::EntityComponentManager& ecm,
    absl::string_view world_object_name) {
  std::vector<gz::sim::Entity> entities;
  ecm.Each<ResourceName, gz::sim::components::ModelSdf>(
      [&](const gz::sim::Entity& entity, const ResourceName* name_component,
          const gz::sim::components::ModelSdf* /*unused*/) -> bool {
        if (name_component->Data() == world_object_name) {
          entities.push_back(entity);
        }
        return true;
      });
  return entities;
}

// Finds the single Gazebo entity in the `ecm` that contains both an Intrinsic
// ResourceName component matching `world_object_name` and a `ModelSdf`
// component, returning the entity ID or an error if zero or multiple entities
// match.
absl::StatusOr<gz::sim::Entity> FindSingleEntityWithModelDataForWorldName(
    const gz::sim::EntityComponentManager& ecm,
    absl::string_view world_object_name) {
  auto entities_with_object_name =
      FindEntitiesWithModelDataForWorldName(ecm, world_object_name);
  if (entities_with_object_name.empty()) {
    return absl::NotFoundError(absl::StrCat(
        "Gazebo ECM has no entities with Intrinsic World object name '",
        world_object_name, "'"));
  }
  if (entities_with_object_name.size() > 1) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Gazebo ECM has multiple entities with Intrinsic World object "
        "name '",
        world_object_name, "'"));
  }
  return entities_with_object_name.front();
}

// Finds the control period by recursively searching the given SDF `element`
// and its ancestors for a `control_frequency_hz` tag, using `hwm_name` and
// `model_name` for error reporting.
absl::StatusOr<absl::Duration> FindControlPeriod(absl::string_view hwm_name,
                                                 absl::string_view model_name,
                                                 ::sdf::ElementPtr element) {
  for (::sdf::ElementPtr current_element = element; current_element != nullptr;
       current_element = current_element->GetParent()) {
    ::sdf::ElementConstPtr control_frequency_element =
        current_element->FindElement(
            std::string{::intrinsic::sdf::kControlFrequencyHz});
    if (control_frequency_element == nullptr) {
      continue;
    }
    ::sdf::Errors errors;
    double control_frequency_hz =
        control_frequency_element->Get<double>(errors);
    if (!errors.empty()) {
      return absl::InvalidArgumentError(
          absl::StrCat("SDF model '", model_name, "' has <",
                       ::intrinsic::sdf::kControlFrequencyHz,
                       "> tag, but does not have a numeric value within!\n",
                       "Full tag: ", control_frequency_element->ToString("")));
    }
    if (control_frequency_hz <= 0) {
      return absl::InvalidArgumentError(absl::StrCat(
          "SDF model '", model_name,
          "' has invalid control frequency: ", control_frequency_hz,
          ". Please set the control frequency to a positive value."));
    }
    return absl::Seconds(1.0 / control_frequency_hz);
  }
  return absl::NotFoundError(absl::StrCat(
      "Could not find control period in either the configuration proto for the "
      "hardware module '",
      hwm_name, "' or in the world model '", model_name,
      "'. Please ensure your top-level hardware module configuration has "
      "either `control_frequency_hz` or `control_period_ns` set."));
}

// Applies the specified `prefix` to the `joint_group_name` of the given
// `interface` if the prefix is non-empty.
void ApplyPrefixToSimHardwareInterface(
    ::intrinsic_proto::sim::SimHardwareInterface& interface,
    absl::string_view prefix) {
  if (prefix.empty()) return;

  switch (interface.interface_case()) {
    case ::intrinsic_proto::sim::SimHardwareInterface::kJointPositionCommand:
      interface.mutable_joint_position_command()->set_joint_group_name(
          absl::StrCat(prefix,
                       interface.joint_position_command().joint_group_name()));
      break;
    case ::intrinsic_proto::sim::SimHardwareInterface::kJointPositionState:
      interface.mutable_joint_position_state()->set_joint_group_name(
          absl::StrCat(prefix,
                       interface.joint_position_state().joint_group_name()));
      break;
    case ::intrinsic_proto::sim::SimHardwareInterface::kJointCommandedPosition:
      interface.mutable_joint_commanded_position()->set_joint_group_name(
          absl::StrCat(
              prefix, interface.joint_commanded_position().joint_group_name()));
      break;
    case ::intrinsic_proto::sim::SimHardwareInterface::kJointVelocityState:
      interface.mutable_joint_velocity_state()->set_joint_group_name(
          absl::StrCat(prefix,
                       interface.joint_velocity_state().joint_group_name()));
      break;
    case ::intrinsic_proto::sim::SimHardwareInterface::kJointAccelerationState:
      interface.mutable_joint_acceleration_state()->set_joint_group_name(
          absl::StrCat(
              prefix, interface.joint_acceleration_state().joint_group_name()));
      break;
    case ::intrinsic_proto::sim::SimHardwareInterface::kJointTorqueCommand:
      interface.mutable_joint_torque_command()->set_joint_group_name(
          absl::StrCat(prefix,
                       interface.joint_torque_command().joint_group_name()));
      break;
    case ::intrinsic_proto::sim::SimHardwareInterface::kJointTorqueState:
      interface.mutable_joint_torque_state()->set_joint_group_name(absl::StrCat(
          prefix, interface.joint_torque_state().joint_group_name()));
      break;
    case ::intrinsic_proto::sim::SimHardwareInterface::kPayloadCommand:
      interface.mutable_payload_command()->set_joint_group_name(
          absl::StrCat(prefix, interface.payload_command().joint_group_name()));
      break;
    case ::intrinsic_proto::sim::SimHardwareInterface::kPayloadState:
      interface.mutable_payload_state()->set_joint_group_name(
          absl::StrCat(prefix, interface.payload_state().joint_group_name()));
      break;
    case ::intrinsic_proto::sim::SimHardwareInterface::kProcessWrenchCommand:
      interface.mutable_process_wrench_command()->set_joint_group_name(
          absl::StrCat(prefix,
                       interface.process_wrench_command().joint_group_name()));
      break;
    case ::intrinsic_proto::sim::SimHardwareInterface::kJointLimitsCommand:
      interface.mutable_joint_limits_command()->set_joint_group_name(
          absl::StrCat(prefix,
                       interface.joint_limits_command().joint_group_name()));
      break;
    default:
      // Other interfaces (DIO, sensors) don't reference joint groups.
      break;
  }
}

// Infers simulation hardware interfaces for a specific Gazebo `model`.
absl::StatusOr<::intrinsic_proto::sim::SimHardwareModuleConfig>
InferSimHardwareModuleConfigForModel(
    absl::string_view asset_name, absl::string_view hwm_name,
    const ::gz::sim::Model& model,
    const ::intrinsic_proto::icon::SimBusModuleConfig& sim_config,
    ::gz::sim::EntityComponentManager& ecm, bool include_additional_devices) {
  INTR_RETURN_IF_ERROR(
      GetModelSdf(model, hwm_name, ecm, sim_config.new_sim_api_config())
          .status());
  return InferSimHardwareModuleConfigImpl(
      asset_name, sim_config.dio_config(),
      include_additional_devices
          ? absl::MakeConstSpan(sim_config.additional_devices())
          : absl::Span<const ::intrinsic_proto::icon::DeviceConfig* const>{},
      ecm);
}

// Infers simulation hardware interfaces for a model identified by its
// `asset_name`.
absl::StatusOr<::intrinsic_proto::sim::SimHardwareModuleConfig>
InferSimHardwareModuleConfigForAssetName(
    absl::string_view asset_name, absl::string_view hwm_name,
    const ::intrinsic_proto::icon::SimBusModuleConfig& sim_config,
    ::gz::sim::EntityComponentManager& ecm, bool include_additional_devices) {
  INTR_ASSIGN_OR_RETURN(
      auto matching_entity,
      FindSingleEntityWithModelDataForWorldName(ecm, asset_name),
      _.SetAppend() << " while looking for asset '" << asset_name << "'");
  ::gz::sim::Model model(matching_entity);
  return InferSimHardwareModuleConfigForModel(
      asset_name, hwm_name, model, sim_config, ecm, include_additional_devices);
}
}  // namespace

absl::StatusOr<::intrinsic_proto::sim::SimHardwareModuleConfig>
InferSimHardwareModuleConfig(
    absl::string_view hwm_resource_name, absl::string_view hwm_name,
    const ::gz::sim::Model& hwm_resource_gazebo_model,
    const ::intrinsic_proto::icon::SimBusModuleConfig& sim_config,
    ::gz::sim::EntityComponentManager& ecm) {
  const ::intrinsic_proto::sim::SimHardwareModuleConfig& new_sim_config =
      sim_config.new_sim_api_config();
  // Infer the manual_hardware_interfaces map, if necessary
  switch (new_sim_config.hardware_interfaces_case()) {
    case ::intrinsic_proto::sim::SimHardwareModuleConfig::
        HardwareInterfacesCase::HARDWARE_INTERFACES_NOT_SET:
    case ::intrinsic_proto::sim::SimHardwareModuleConfig::
        HardwareInterfacesCase::kFromSelf: {
      absl::StatusOr<::intrinsic_proto::sim::SimHardwareModuleConfig>
          updated_hwm_config = InferSimHardwareModuleConfigForModel(
              hwm_resource_name, hwm_name, hwm_resource_gazebo_model,
              sim_config, ecm, /*include_additional_devices=*/true);
      if (!updated_hwm_config.ok() &&
          absl::IsNotFound(updated_hwm_config.status())) {
        return absl::NotFoundError(
            absl::StrCat(updated_hwm_config.status().message(),
                         GetSdfNotFoundErrorAdvice(new_sim_config)));
      }
      INTR_RETURN_IF_ERROR(updated_hwm_config.status());

      LOG(INFO) << "[" << hwm_name << "]: Inferred "
                << updated_hwm_config->manual_hardware_interfaces()
                       .name_to_interface()
                       .size()
                << " hardware interfaces:";
      LOG(INFO) << *updated_hwm_config;
      return *updated_hwm_config;
    }
    case ::intrinsic_proto::sim::SimHardwareModuleConfig::
        HardwareInterfacesCase::kGeometryAssetName: {
      absl::StatusOr<::intrinsic_proto::sim::SimHardwareModuleConfig>
          updated_hwm_config = InferSimHardwareModuleConfigForAssetName(
              new_sim_config.geometry_asset_name(), hwm_name, sim_config, ecm,
              /*include_additional_devices=*/true);
      if (!updated_hwm_config.ok() &&
          absl::IsNotFound(updated_hwm_config.status())) {
        return absl::NotFoundError(
            absl::StrCat(updated_hwm_config.status().message(),
                         GetSdfNotFoundErrorAdvice(new_sim_config)));
      }
      INTR_RETURN_IF_ERROR(updated_hwm_config.status());

      LOG(INFO) << "[" << hwm_name << "]: Inferred "
                << updated_hwm_config->manual_hardware_interfaces()
                       .name_to_interface()
                       .size()
                << " hardware interfaces:";
      LOG(INFO) << *updated_hwm_config;
      return *updated_hwm_config;
    }
    case ::intrinsic_proto::sim::SimHardwareModuleConfig::
        HardwareInterfacesCase::kFromInstances: {
      ::intrinsic_proto::sim::SimHardwareModuleConfig combined_config;
      const auto& from_instances = new_sim_config.from_instances();

      for (const auto& instance : from_instances.instances()) {
        const std::string& instance_name = instance.instance_name();
        // We pass include_additional_devices=false here because analog IOs
        // are defined globally in the HWM config, not per instance. We will
        // populate them once at the end to avoid duplication.
        absl::StatusOr<::intrinsic_proto::sim::SimHardwareModuleConfig>
            instance_config = InferSimHardwareModuleConfigForAssetName(
                instance_name, hwm_name, sim_config, ecm,
                /*include_additional_devices=*/false);
        if (!instance_config.ok() &&
            absl::IsNotFound(instance_config.status())) {
          return absl::NotFoundError(
              absl::StrCat(instance_config.status().message(),
                           GetSdfNotFoundErrorAdvice(new_sim_config)));
        }
        INTR_RETURN_IF_ERROR(instance_config.status());

        std::string prefix = absl::StrCat(instance_name, "_");
        if (instance.has_omit_instance_name_prefix()) {
          prefix = "";
        } else if (instance.has_custom_prefix()) {
          if (instance.custom_prefix().empty()) {
            return absl::InvalidArgumentError(absl::StrCat(
                "Instance '", instance_name,
                "' has an empty `custom_prefix`. Use "
                "`omit_instance_name_prefix: true` if you want no prefix."));
          }
          prefix = absl::StrCat(instance.custom_prefix(), "_");
        }

        for (const auto& [jg_name, jg] :
             (*instance_config).manual_hardware_interfaces().joint_groups()) {
          auto& combined_joint_groups =
              *combined_config.mutable_manual_hardware_interfaces()
                   ->mutable_joint_groups();
          std::string final_jg_name = absl::StrCat(prefix, jg_name);
          if (combined_joint_groups.contains(final_jg_name)) {
            return absl::AlreadyExistsError(absl::StrCat(
                "Duplicate joint group name '", final_jg_name,
                "' generated while processing instance '", instance_name, "'"));
          }
          combined_joint_groups[final_jg_name] = jg;
        }

        absl::flat_hash_map<std::string, std::string> name_mapping;
        for (const auto& mapping : instance.interface_mappings()) {
          const auto from_interface_name = mapping.from_interface_name();
          if (name_mapping.contains(from_interface_name)) {
            return absl::AlreadyExistsError(absl::StrCat(
                "Duplicate `from_interface_name` '", from_interface_name,
                "' while processing instance '", instance_name,
                "'. You cannot map the same interface multiple times."));
          }

          name_mapping[mapping.from_interface_name()] =
              mapping.to_interface_name();
        }

        for (auto [inferred_name, interface] : (*instance_config)
                                                   .manual_hardware_interfaces()
                                                   .name_to_interface()) {
          ApplyPrefixToSimHardwareInterface(interface, prefix);

          std::string final_name;
          if (auto it = name_mapping.find(inferred_name);
              it != name_mapping.end()) {
            final_name = it->second;
            LOG(INFO) << "Mapped interface '" << inferred_name << "' to '"
                      << final_name << "' for instance '" << instance_name
                      << "'";
          } else {
            final_name = absl::StrCat(prefix, inferred_name);
          }

          auto& name_to_interface =
              *combined_config.mutable_manual_hardware_interfaces()
                   ->mutable_name_to_interface();
          if (name_to_interface.contains(final_name)) {
            return absl::AlreadyExistsError(absl::StrCat(
                "Duplicate hardware interface name '", final_name,
                "' generated while processing instance '", instance_name, "'"));
          }
          name_to_interface[final_name] = interface;
        }
      }

      // Populate analog IOs once for the combined config.
      INTR_RETURN_IF_ERROR(PopulateInferredAnalogIoInterfaces(
          hwm_name, absl::MakeConstSpan(sim_config.additional_devices()),
          combined_config));

      LOG(INFO) << "[" << hwm_name << "]: Inferred "
                << combined_config.manual_hardware_interfaces()
                       .name_to_interface()
                       .size()
                << " hardware interfaces:";
      LOG(INFO) << combined_config;

      return combined_config;
    }
    case ::intrinsic_proto::sim::SimHardwareModuleConfig::
        HardwareInterfacesCase::kNoInterfaces:
      // Nothing to do here, fall through to the kManualHardwareInterfaces case
      // (which also does nothing)
    case ::intrinsic_proto::sim::SimHardwareModuleConfig::
        HardwareInterfacesCase::kManualHardwareInterfaces: {
      // Nothing to do here. The configuration
      // already has manual interfaces.
      return new_sim_config;
    }
  }
  return absl::InvalidArgumentError(
      absl::StrCat("Configuration for HWM '", hwm_name,
                   "' has invalid oneof value for 'hardware_interfaces"));
}

absl::StatusOr<absl::Duration> FindControlPeriod(
    const ::intrinsic_proto::icon::HardwareModuleConfig& module_config,
    const ::gz::sim::Model& hwm_resource_gazebo_model,
    const ::intrinsic_proto::sim::SimHardwareModuleConfig& sim_config,
    const ::gz::sim::EntityComponentManager& ecm,
    const AssetInstancesClient& asset_instances_client) {
  std::optional<absl::StatusOr<absl::Duration>> control_period = std::nullopt;

  auto get_control_period =
      [](const ::intrinsic_proto::icon::HardwareModuleConfig& module_config)
      -> std::optional<absl::StatusOr<absl::Duration>> {
    switch (module_config.control_rate_case()) {
      case intrinsic_proto::icon::HardwareModuleConfig::kControlFrequencyHz: {
        if (module_config.control_frequency_hz() <= 0) {
          return absl::InvalidArgumentError(
              absl::StrCat("Control frequency must be positive, but is ",
                           module_config.control_frequency_hz()));
        }
        return absl::Seconds(1.0 / module_config.control_frequency_hz());
      }
      case intrinsic_proto::icon::HardwareModuleConfig::kControlPeriodNs: {
        if (module_config.control_period_ns() <= 0) {
          return absl::InvalidArgumentError(
              absl::StrCat("Control period must be positive, but is ",
                           module_config.control_period_ns()));
        }
        return absl::Nanoseconds(module_config.control_period_ns());
      }
      case intrinsic_proto::icon::HardwareModuleConfig::CONTROL_RATE_NOT_SET:
      default:
        // TODO(b/389912730): If and when we deprecate vendor specific
        // control frequency parameters and the corresponding WorldUpdate,
        // change this to return an error if the control_rate oneof is
        // unset.
        return std::nullopt;
    }
  };

  if (sim_config.control_frequency_asset_name().empty()) {
    // No control_frequency_asset_name, try to extract control period from
    // this HWM's `module_config` directly.
    control_period = get_control_period(module_config);
  } else {
    // Retrieve module config for an asset named `control_frequency_asset_name`,
    // then try to get control frequency from that.
    INTR_ASSIGN_OR_RETURN(
        intrinsic_proto::assets::v1::AssetInstance asset_instance,
        asset_instances_client.GetAssetInstance(
            sim_config.control_frequency_asset_name()));
    INTR_ASSIGN_OR_RETURN(
        intrinsic_proto::icon::HardwareModuleConfig hwm_config,
        AssetInstancesClient::ExtractServiceConfig<
            intrinsic_proto::icon::HardwareModuleConfig>(asset_instance));
    control_period = get_control_period(hwm_config);
  }

  // If we've found a control period (or encountered an error retrieving it),
  // then return that.
  if (control_period.has_value()) {
    return control_period.value();
  }

  // If not, then we move on to trying to get the control frequency from the
  // SDFormat model.
  // TODO(b/389912730): If and when we deprecate vendor specific
  // control frequency parameters and the corresponding WorldUpdate,
  // return an error here.

  // Precedence for reading from SDF:
  // 1. control_frequency_asset_name
  // 2. geometry_asset_name
  // 3. from_instances (aggregated and consistent)
  // 4. Primary HWM model (fallback)

  // If a specific asset name is provided for frequency or geometry, use it as
  // the single source of truth for the SDF lookup, preferring the dedicated
  // control frequency asset if both are set.
  if (!sim_config.control_frequency_asset_name().empty() ||
      sim_config.has_geometry_asset_name()) {
    std::string asset_name = sim_config.control_frequency_asset_name();
    std::string advice =
        ". If there is a separate geometry asset for this hardware "
        "module, put that asset's name in the hardware module "
        "configuration's "
        "`simulation_module_config.new_sim_api_config.control_frequency_"
        "asset_name` field";

    if (asset_name.empty()) {
      asset_name = sim_config.geometry_asset_name();
      advice =
          ". If there is a separate geometry asset for this hardware "
          "module, put that asset's name in the hardware module "
          "configuration's "
          "`simulation_module_config.new_sim_api_config.geometry_asset_"
          "name` field";
    }

    INTR_ASSIGN_OR_RETURN(
        auto matching_entity,
        FindSingleEntityWithModelDataForWorldName(ecm, asset_name),
        _.SetAppend() << advice);
    ::gz::sim::Model model_with_control_frequency(matching_entity);
    INTR_ASSIGN_OR_RETURN(const auto* model_sdf_comp,
                          GetModelSdf(model_with_control_frequency,
                                      module_config.name(), ecm, sim_config));
    return FindControlPeriod(
        /*hwm_name=*/module_config.name(),
        /*model_name=*/model_with_control_frequency.Name(ecm),
        /*element=*/model_sdf_comp->Data().Element());
  }

  if (sim_config.has_from_instances()) {
    absl::flat_hash_map<absl::Duration, std::vector<std::string>> found_periods;
    for (const auto& instance : sim_config.from_instances().instances()) {
      INTR_ASSIGN_OR_RETURN(
          auto matching_entity,
          FindSingleEntityWithModelDataForWorldName(ecm,
                                                    instance.instance_name()),
          _.SetAppend() << " (while checking instances for control frequency)");
      ::gz::sim::Model instance_model(matching_entity);
      INTR_ASSIGN_OR_RETURN(
          const auto* model_sdf_comp,
          GetModelSdf(instance_model, module_config.name(), ecm, sim_config));
      auto status_or_period = FindControlPeriod(
          /*hwm_name=*/module_config.name(),
          /*model_name=*/instance_model.Name(ecm),
          /*element=*/model_sdf_comp->Data().Element());
      if (status_or_period.ok()) {
        found_periods[status_or_period.value()].push_back(
            instance.instance_name());
      } else if (!absl::IsNotFound(status_or_period.status())) {
        return status_or_period.status();
      }
    }

    if (found_periods.size() > 1) {
      std::string error_msg =
          "Inconsistent control frequencies found in instances:\n";
      for (const auto& [period, instances] : found_periods) {
        absl::StrAppend(&error_msg, "  - ", 1.0 / absl::ToDoubleSeconds(period),
                        "Hz in instances: ", absl::StrJoin(instances, ", "),
                        "\n");
      }
      return absl::InvalidArgumentError(error_msg);
    } else if (found_periods.size() == 1) {
      return found_periods.begin()->first;
    }
  }

  // Fallback: Get control period from the HWM's primary model by inspecting the
  // SDF.
  INTR_ASSIGN_OR_RETURN(const auto* model_sdf_comp,
                        GetModelSdf(hwm_resource_gazebo_model,
                                    module_config.name(), ecm, sim_config));
  return FindControlPeriod(
      /*hwm_name=*/module_config.name(),
      /*model_name=*/hwm_resource_gazebo_model.Name(ecm),
      /*element=*/model_sdf_comp->Data().Element());
}

}  // namespace intrinsic::simulation
