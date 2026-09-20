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

#include "intrinsic/simulation/gazebo/plugins/sim_hardware_interface_data.h"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Joint.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/components/GpuLidar.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointForce.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityReset.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/WrenchMeasured.hh"
#include "intrinsic/simulation/gazebo/components/digital_io_components.h"
#include "intrinsic/simulation/gazebo/components/ft_sensor_tare_components.h"
#include "intrinsic/simulation/gazebo/components/ppr_component.h"
#include "intrinsic/simulation/gazebo/plugins/gravity_compensator.h"
#include "intrinsic/simulation/gazebo/plugins/sim_hardware_module_config.pb.h"
#include "intrinsic/util/status/status_macros.h"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Sensor.hh"

namespace intrinsic::simulation::hardware_interface_data {
namespace {

// Tries to find a joint group `joint_group_name` in `joint_groups_by_name`, and
// ensures that all of the entities in that group
// * Exist in `ecm`
// * Have the gz::sim::components::Joint Component
//
// Uses `hwm_name`, `interface_type_name` and `interface_name` to build helpful
// error messages on failure.
absl::StatusOr<std::vector<gz::sim::Entity>> LookupJointGroup(
    absl::string_view hwm_name, absl::string_view interface_type_name,
    absl::string_view interface_name, absl::string_view joint_group_name,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm) {
  // Look up joint group, error if missing
  auto joint_group = joint_groups_by_name.find(joint_group_name);
  if (joint_group == joint_groups_by_name.end()) {
    return absl::NotFoundError(absl::StrCat(
        interface_type_name, " interface '", interface_name,
        "' for simulated HWM '", hwm_name, "' refers to joint group '",
        joint_group_name,
        "', but that group does not exist. Available joint groups: [",
        absl::StrJoin(joint_groups_by_name, ", ",
                      [](std::string* str, const auto& name_and_joints) {
                        absl::StrAppend(str, name_and_joints.first);
                      }),
        "]"));
  }
  // Check that every joint in the group actually exists in `ecm`
  for (const auto& joint_entity : joint_group->second) {
    if (!ecm.HasEntity(joint_entity)) {
      return absl::NotFoundError(absl::StrCat(
          "Joint group '", joint_group_name, "' contains an entity (with ID ",
          joint_entity, ") that does not exist in the Gazebo ECM"));
    }
    if (!ecm.EntityHasComponentType(joint_entity,
                                    gz::sim::components::Joint::typeId)) {
      return absl::FailedPreconditionError(absl::StrCat(
          "Joint group '", joint_group_name, "' contains an entity (with ID ",
          joint_entity, ") that is not a joint"));
    }
  }
  return joint_group->second;
}

// Finds all entities that have *both* an Intrinsic ResourceName Component with
// `world_object_name` as its data, and a gz::sim::components::ModelSdf
// component.
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

// Extracts the information needed for the (Non)StrictJointPositionCommandData
// for `interface_name` of `hwm_name` from `ecm`, based on `interface_config`.
//
// This implementation is shared between
// `GetStrictJointPositionCommandDataFromEcm()` and
// `GetNonStrictJointPositionCommandDataFromEcm()` (see their documentation for
// more details).
absl::StatusOr<std::variant<StrictJointPositionCommandData,
                            NonStrictJointPositionCommandData>>
GetStrictOrNonStrictJointPositionCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointPositionCommandInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm) {
  // Look up joint group, error if missing
  INTR_ASSIGN_OR_RETURN(
      auto joint_group,
      LookupJointGroup(hwm_name,
                       interface_config.GetMetadata().descriptor->name(),
                       interface_name, interface_config.joint_group_name(),
                       joint_groups_by_name, ecm));

  std::optional<std::vector<double>> previous_position_setpoints;
  std::vector<std::unique_ptr<GravityCompensator>> gravity_compensators;
  // For each joint in the group
  // 1. Initialize previous_setpoint with the current position
  // 2. Try to build a GravityCompensator
  for (size_t i = 0; i < joint_group.size(); ++i) {
    const gz::sim::Entity& joint_entity = joint_group.at(i);
    // During Physics::Update, the current position of each joint is written
    // to the JointPosition component, but before the first Physics::Update,
    // the component's value depends on how it is initialized. We initialize the
    // JointPosition components to zero in GetJointPositionStateDataFromEcm(),
    // but that may happen after this function runs, depending on the order of
    // interfaces. If there is a SetModelState plugin in the Gazebo world, that
    // writes the initial joint positions to the JointPositionReset component,
    // which the Physics System copies to the JointPosition component during the
    // first Physics::Update.
    //
    // If the JointPositionReset component is present and not empty,
    // use its first vector value as the initial joint position setpoint.
    // Otherwise use the value of the JointPosition component if it is a
    // vector of size 1, and otherwise initialize fall back to zero.
    std::optional<double> current_position = std::nullopt;
    {
      std::optional<std::vector<double>> joint_position_reset_data =
          ecm.ComponentData<gz::sim::components::JointPositionReset>(
              joint_entity);
      if (joint_position_reset_data.has_value() &&
          !joint_position_reset_data->empty()) {
        current_position = joint_position_reset_data->front();
      }
    }
    if (current_position == std::nullopt) {
      std::optional<std::vector<double>> joint_position =
          ecm.ComponentData<gz::sim::components::JointPosition>(joint_entity);
      if (joint_position.has_value() && !joint_position->empty()) {
        current_position = joint_position->front();
      }
    }
    if (current_position.has_value()) {
      if (previous_position_setpoints == std::nullopt) {
        previous_position_setpoints = std::vector<double>{};
      }
      previous_position_setpoints->push_back(current_position.value());
    }
    INTR_ASSIGN_OR_RETURN(gravity_compensators.emplace_back(),
                          GravityCompensator::Create(joint_entity, &ecm,
                                                     /*scope_separator=*/"::"),
                          _ << "building GravityCompensator for Joint entity "
                            << joint_entity << " (at index " << i
                            << ") in group '"
                            << interface_config.joint_group_name()
                            << "' for simulated HWM '" << hwm_name << "'");
  }

  if (previous_position_setpoints.has_value() &&
      previous_position_setpoints->size() != joint_group.size()) {
    return absl::FailedPreconditionError(absl::StrCat(
        "JointPositionCommand interface '", interface_name,
        "' for simulated HWM '", hwm_name, "' (for joint group '",
        interface_config.joint_group_name(),
        "') has *some* joints with initial positions. It's very unlikely that "
        "this is correct, please check your configuration"));
  }

  // Add the correct interface (strict or non-strict)
  if (interface_config.strict()) {
    return StrictJointPositionCommandData{
        .joint_group_name = interface_config.joint_group_name(),
        .previous_setpoints = std::move(previous_position_setpoints),
        .gravity_compensators = std::move(gravity_compensators),
    };
  } else {
    return NonStrictJointPositionCommandData{
        .joint_group_name = interface_config.joint_group_name(),
        .previous_setpoints = std::move(previous_position_setpoints),
        .gravity_compensators = std::move(gravity_compensators),
    };
  }
}

// Extracts the information needed for the (Non)StrictJointTorqueCommandData for
// `interface_name` of `hwm_name` from `ecm`, based on `interface_config`.
//
// This implementation is shared between
// `GetStrictJointTorqueCommandDataFromEcm()` and
// `GetNonStrictJointTorqueCommandDataFromEcm()` (see their documentation for
// more details).
absl::StatusOr<
    std::variant<StrictJointTorqueCommandData, NonStrictJointTorqueCommandData>>
GetStrictOrNonStrictJointTorqueCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointTorqueCommandInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm) {
  // Look up joint group, error if missing
  INTR_ASSIGN_OR_RETURN(
      auto joint_group,
      LookupJointGroup(hwm_name,
                       interface_config.GetMetadata().descriptor->name(),
                       interface_name, interface_config.joint_group_name(),
                       joint_groups_by_name, ecm));

  std::vector<std::unique_ptr<GravityCompensator>> gravity_compensators;
  gravity_compensators.reserve(joint_group.size());
  // Try to build a GravityCompensator for each joint in the group
  for (size_t i = 0; i < joint_group.size(); ++i) {
    const gz::sim::Entity& joint_entity = joint_group.at(i);
    INTR_ASSIGN_OR_RETURN(gravity_compensators.emplace_back(),
                          GravityCompensator::Create(joint_entity, &ecm,
                                                     /*scope_separator=*/"::"),
                          _ << "building GravityCompensator for Joint entity "
                            << joint_entity << " (at index " << i
                            << ") in group '"
                            << interface_config.joint_group_name()
                            << "' for simulated HWM '" << hwm_name << "'");
  }

  // Add the correct interface (strict or non-strict)
  if (interface_config.strict()) {
    return StrictJointTorqueCommandData{
        .joint_group_name = interface_config.joint_group_name(),
        .gravity_compensators = std::move(gravity_compensators),
    };
  } else {
    return NonStrictJointTorqueCommandData{
        .joint_group_name = interface_config.joint_group_name(),
        .gravity_compensators = std::move(gravity_compensators),
    };
  }
}

// Finds a force torque sensor under one of the Gazebo models associated with
// the Intrinsic World object name `world_object_name`.
//
// If `sensor_name` is set, this searches for a sensor with that name.
// If `sensor_name` is unset, this checks that there is only a single
// force/torque sensor across all models associated with `world_object_name`,
// and if so, returns that.
//
// Returns InvalidArgumentError if `world_object_name` is empty.
// Returns NotFoundError if there are no Intrinsic World objects with
// `world_object_name` (more precisely, no Gazebo Entities with
// `intrinsic::simulation::components::ResourceName(world_object_name)`).
// Returns FailedPreconditionError if there is more than one sensor that matches
// `world_object_name` and (if present) `sensor_name`.
// Returns NotFoundError if there is no sensor that matches `world_object_name`
// and (if present) `sensor_name`.
absl::StatusOr<gz::sim::Entity> FindForceTorqueSensorEntity(
    absl::string_view hwm_name, absl::string_view interface_name,
    absl::string_view world_object_name, absl::string_view sensor_name,
    const gz::sim::EntityComponentManager& ecm) {
  if (world_object_name.empty()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Interface '", interface_name,
                     "' of simulated hardware module '", hwm_name,
                     "' does not specify a world_object_name. Please check "
                     "your configuration"));
  }
  std::vector<gz::sim::Entity> ft_sensor_parent_entities =
      FindEntitiesWithModelDataForWorldName(ecm, world_object_name);
  if (ft_sensor_parent_entities.empty()) {
    return absl::NotFoundError(absl::StrCat(
        "Interface '", interface_name, "' of simulated hardware module '",
        hwm_name, "' refers to a world object named '", world_object_name,
        "', but there are no simulated entities that are associated with that "
        "name and have SDF model information"));
  }

  std::optional<gz::sim::Entity> ft_sensor_entity = std::nullopt;
  // Try all entities we've found (skipping those that don't have the ModelSdf
  // component)
  for (const gz::sim::Entity& object_entity : ft_sensor_parent_entities) {
    auto model_sdf_comp =
        ecm.Component<gz::sim::components::ModelSdf>(object_entity);
    if (!model_sdf_comp) {
      continue;
    }
    const ::sdf::Model& model_sdf = model_sdf_comp->Data();
    gz::sim::Model model_gz(object_entity);
    for (size_t j = 0; j < model_sdf.JointCount(); ++j) {
      const ::sdf::Joint* joint_sdf = model_sdf.JointByIndex(j);

      gz::sim::Joint joint(model_gz.JointByName(ecm, joint_sdf->Name()));
      // Check for force-torque sensors, which can be attached to multiple joint
      // types, including fixed and revolute joints.
      for (size_t s = 0; s < joint_sdf->SensorCount(); ++s) {
        const ::sdf::Sensor* sensor_sdf = joint_sdf->SensorByIndex(s);
        if (sensor_sdf->Type() != ::sdf::SensorType::FORCE_TORQUE) {
          continue;
        }
        gz::sim::Entity sensor_entity =
            joint.SensorByName(ecm, sensor_sdf->Name());

        if (sensor_name.empty()) {
          // No ft_sensor_name, so we want there to be exactly one ft sensor
          // entity.
          if (ft_sensor_entity.has_value()) {
            return absl::FailedPreconditionError(absl::StrCat(
                "Entity ", object_entity, " for interface '", interface_name,
                "' of simulated HWM '", hwm_name,
                "' has multiple force/torque sensors, but the configuration "
                "does not tell us which one to use. Please set ft_sensor_name "
                "in your configuration"));
          }
          ft_sensor_entity = sensor_entity;
        } else if (sensor_sdf->Name() == sensor_name) {
          // We have a sensor name, so the error message is slightly different
          // if we find more than one matching sensor.
          if (ft_sensor_entity.has_value()) {
            return absl::FailedPreconditionError(absl::StrCat(
                "The Intrinsic World object '", world_object_name,
                "' for interface '", interface_name, "' of simulated HWM '",
                hwm_name, "' has multiple force/torque sensors named '",
                sensor_name,
                "'. Please check both your configuration and the SceneObject "
                "definition for '",
                world_object_name, "'"));
          }
          ft_sensor_entity = sensor_entity;
        }
      }
    }
  }
  if (ft_sensor_entity == std::nullopt) {
    return absl::NotFoundError(absl::StrCat(
        "Could not find the simulated force/torque sensor entity for interface "
        "'",
        interface_name, "' of simulated HWM '", hwm_name, "'"));
  }
  return *ft_sensor_entity;
}

// Finds a rangefinder / Lidar sensor under one of the Gazebo models associated
// with the Intrinsic World object name `world_object_name`.
//
// If `sensor_name` is set, this searches for a sensor with that name.
// If `sensor_name` is unset, this checks that there is only a single
// Lidar sensor across all models associated with `world_object_name`,
// and if so, returns that.
//
// Returns InvalidArgumentError if `world_object_name` is empty.
// Returns NotFoundError if there are no Intrinsic World objects with
// `world_object_name` (more precisely, no Gazebo Entities with
// `intrinsic::simulation::components::ResourceName(world_object_name)`).
// Returns FailedPreconditionError if there is more than one sensor that matches
// `world_object_name` and (if present) `sensor_name`.
// Returns NotFoundError if there is no sensor that matches `world_object_name`
// and (if present) `sensor_name`.
// Returns NotFoundError if the rangefinder sensor entity has no GpuLidar
// component.
absl::StatusOr<gz::sim::Entity> FindRangefinderSensorEntity(
    absl::string_view hwm_name, absl::string_view interface_name,
    absl::string_view world_object_name, absl::string_view sensor_name,
    const gz::sim::EntityComponentManager& ecm) {
  if (world_object_name.empty()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Interface '", interface_name,
                     "' of simulated hardware module '", hwm_name,
                     "' does not specify a world_object_name. Please check "
                     "your configuration"));
  }
  std::vector<gz::sim::Entity> rangefinder_parent_entities =
      FindEntitiesWithModelDataForWorldName(ecm, world_object_name);
  if (rangefinder_parent_entities.empty()) {
    return absl::NotFoundError(absl::StrCat(
        "Interface '", interface_name, "' of simulated hardware module '",
        hwm_name, "' refers to a world object named '", world_object_name,
        "', but there are no simulated entities that are associated with that "
        "name and have SDF model information"));
  }

  std::optional<gz::sim::Entity> rangefinder_entity = std::nullopt;
  // Try all entities we've found (skipping those that don't have the ModelSdf
  // component)
  for (const gz::sim::Entity& object_entity : rangefinder_parent_entities) {
    auto model_sdf_comp =
        ecm.Component<gz::sim::components::ModelSdf>(object_entity);
    if (!model_sdf_comp) {
      continue;
    }
    const ::sdf::Model& model_sdf = model_sdf_comp->Data();
    gz::sim::Model model_gz(object_entity);
    for (size_t l = 0; l < model_sdf.LinkCount(); ++l) {
      const ::sdf::Link* link_sdf = model_sdf.LinkByIndex(l);

      gz::sim::Link link(model_gz.LinkByName(ecm, link_sdf->Name()));
      // Assume that rangefinders are represented by GPU_LIDAR sensors, which
      // can only be attached to links.
      for (size_t s = 0; s < link_sdf->SensorCount(); ++s) {
        const ::sdf::Sensor* sensor_sdf = link_sdf->SensorByIndex(s);
        if (sensor_sdf->Type() != ::sdf::SensorType::GPU_LIDAR) {
          continue;
        }
        gz::sim::Entity sensor_entity =
            link.SensorByName(ecm, sensor_sdf->Name());

        if (sensor_name.empty()) {
          // No sensor name, so we want there to be exactly one rangefinder
          // sensor entity.
          if (rangefinder_entity.has_value()) {
            return absl::FailedPreconditionError(absl::StrCat(
                "Entity ", object_entity, " for interface '", interface_name,
                "' of simulated HWM '", hwm_name,
                "' has multiple rangefinder sensors, but the configuration "
                "does not tell us which one to use. Please set "
                "rangefinder_name in your configuration"));
          }
          rangefinder_entity = sensor_entity;
        } else if (sensor_sdf->Name() == sensor_name) {
          // We have a sensor name, so the error message is slightly different
          // if we find more than one matching sensor.
          if (rangefinder_entity.has_value()) {
            return absl::FailedPreconditionError(absl::StrCat(
                "The Intrinsic World object '", world_object_name,
                "' for interface '", interface_name, "' of simulated HWM '",
                hwm_name, "' has multiple rangefinder sensors named '",
                sensor_name,
                "'. Please check both your configuration and the SceneObject "
                "definition for '",
                world_object_name, "'"));
          }
          rangefinder_entity = sensor_entity;
        }
      }
    }
  }
  if (rangefinder_entity == std::nullopt) {
    return absl::NotFoundError(absl::StrCat(
        "Could not find the simulated rangefinder sensor entity for interface "
        "'",
        interface_name, "' of simulated HWM '", hwm_name, "'"));
  }
  std::optional<sdf::Sensor> gpu_lidar_component_data =
      ecm.ComponentData<gz::sim::components::GpuLidar>(*rangefinder_entity);
  if (!gpu_lidar_component_data.has_value()) {
    return absl::NotFoundError(
        absl::StrCat("ECM entity ", *rangefinder_entity,
                     " does not have GpuLidar component"));
  }
  return *rangefinder_entity;
}

}  // namespace

absl::StatusOr<absl::flat_hash_map<std::string, HardwareInterfaceData>>
BuildHardwareInterfaceData(
    absl::string_view hwm_resource_name,
    const intrinsic_proto::sim::SimHardwareModuleConfig& sim_hwm_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    gz::sim::EntityComponentManager& ecm,
    absl::flat_hash_map<gz::sim::Entity, std::string>&
        rangefinder_topic_by_entity) {
  absl::flat_hash_map<std::string, HardwareInterfaceData>
      interface_name_to_interface_data;
  for (const auto& [interface_name, interface_config] :
       sim_hwm_config.manual_hardware_interfaces().name_to_interface()) {
    switch (interface_config.interface_case()) {
      case intrinsic_proto::sim::SimHardwareInterface::kJointPositionCommand: {
        const ::intrinsic_proto::sim::JointPositionCommandInterface& interface =
            interface_config.joint_position_command();
        if (interface.strict()) {
          INTR_ASSIGN_OR_RETURN(
              interface_name_to_interface_data[interface_name],
              GetStrictJointPositionCommandDataFromEcm(
                  hwm_resource_name, interface_name, interface,
                  joint_groups_by_name, ecm));
        } else {
          INTR_ASSIGN_OR_RETURN(
              interface_name_to_interface_data[interface_name],

              GetNonStrictJointPositionCommandDataFromEcm(
                  hwm_resource_name, interface_name, interface,
                  joint_groups_by_name, ecm));
        }
        break;
      }
      case intrinsic_proto::sim::SimHardwareInterface::
          kJointCommandedPosition: {
        INTR_ASSIGN_OR_RETURN(interface_name_to_interface_data[interface_name],
                              GetJointCommandedPositionDataFromEcm(
                                  hwm_resource_name, interface_name,
                                  interface_config.joint_commanded_position(),
                                  joint_groups_by_name, ecm));
        break;
      }
      case intrinsic_proto::sim::SimHardwareInterface::kJointPositionState: {
        INTR_ASSIGN_OR_RETURN(interface_name_to_interface_data[interface_name],
                              GetJointPositionStateDataFromEcm(
                                  hwm_resource_name, interface_name,
                                  interface_config.joint_position_state(),
                                  joint_groups_by_name, ecm));
        break;
      }
      case intrinsic_proto::sim::SimHardwareInterface::kJointVelocityState: {
        INTR_ASSIGN_OR_RETURN(interface_name_to_interface_data[interface_name],
                              GetJointVelocityStateDataFromEcm(
                                  hwm_resource_name, interface_name,
                                  interface_config.joint_velocity_state(),
                                  joint_groups_by_name, ecm));
        break;
      }
      case intrinsic_proto::sim::SimHardwareInterface::
          kJointAccelerationState: {
        INTR_ASSIGN_OR_RETURN(interface_name_to_interface_data[interface_name],
                              GetJointAccelerationStateDataFromEcm(
                                  hwm_resource_name, interface_name,
                                  interface_config.joint_acceleration_state(),
                                  joint_groups_by_name, ecm));
        break;
      }
      case intrinsic_proto::sim::SimHardwareInterface::kJointTorqueCommand: {
        const ::intrinsic_proto::sim::JointTorqueCommandInterface& interface =
            interface_config.joint_torque_command();
        if (interface.strict()) {
          INTR_ASSIGN_OR_RETURN(
              interface_name_to_interface_data[interface_name],
              GetStrictJointTorqueCommandDataFromEcm(
                  hwm_resource_name, interface_name, interface,
                  joint_groups_by_name, ecm));
        } else {
          INTR_ASSIGN_OR_RETURN(
              interface_name_to_interface_data[interface_name],

              GetNonStrictJointTorqueCommandDataFromEcm(
                  hwm_resource_name, interface_name, interface,
                  joint_groups_by_name, ecm));
        }
        break;
      }
      case intrinsic_proto::sim::SimHardwareInterface::kJointTorqueState: {
        INTR_ASSIGN_OR_RETURN(interface_name_to_interface_data[interface_name],
                              GetJointTorqueStateDataFromEcm(
                                  hwm_resource_name, interface_name,
                                  interface_config.joint_torque_state(),
                                  joint_groups_by_name, ecm));
        break;
      }
      case intrinsic_proto::sim::SimHardwareInterface::kPayloadCommand: {
        INTR_ASSIGN_OR_RETURN(
            interface_name_to_interface_data[interface_name],
            GetKinematicChainPayloadCommandDataFromEcm(
                hwm_resource_name, interface_name,
                interface_config.payload_command(), joint_groups_by_name, ecm));
        break;
      }
      case intrinsic_proto::sim::SimHardwareInterface::kPayloadState: {
        INTR_ASSIGN_OR_RETURN(
            interface_name_to_interface_data[interface_name],
            GetKinematicChainPayloadStateDataFromEcm(
                hwm_resource_name, interface_name,
                interface_config.payload_state(), joint_groups_by_name, ecm));
        break;
      }
      case intrinsic_proto::sim::SimHardwareInterface::kProcessWrenchCommand: {
        INTR_ASSIGN_OR_RETURN(interface_name_to_interface_data[interface_name],

                              GetKinematicChainProcessWrenchCommandDataFromEcm(
                                  hwm_resource_name, interface_name,
                                  interface_config.process_wrench_command(),
                                  joint_groups_by_name, ecm));
        break;
      }
      case intrinsic_proto::sim::SimHardwareInterface::kDigitalInput: {
        INTR_ASSIGN_OR_RETURN(interface_name_to_interface_data[interface_name],
                              GetDigitalInputStatusDataFromEcm(
                                  hwm_resource_name, interface_name,
                                  interface_config.digital_input(), ecm));
        break;
      }
      case intrinsic_proto::sim::SimHardwareInterface::kDigitalOutput: {
        INTR_ASSIGN_OR_RETURN(interface_name_to_interface_data[interface_name],
                              GetDigitalOutputCommandDataFromEcm(
                                  hwm_resource_name, interface_name,
                                  interface_config.digital_output(), ecm));
        break;
      }
      case intrinsic_proto::sim::SimHardwareInterface::InterfaceCase::
          kAnalogInput: {
        interface_name_to_interface_data[interface_name] =
            AnalogInputStatusData{
                .num_inputs = interface_config.analog_input().num_inputs()};
        break;
      }
      case intrinsic_proto::sim::SimHardwareInterface::kAnalogOutput: {
        interface_name_to_interface_data[interface_name] =
            AnalogOutputCommandData{
                .num_outputs = interface_config.analog_output().num_outputs()};
        break;
      }
      case intrinsic_proto::sim::SimHardwareInterface::kForceTorqueCommand: {
        INTR_ASSIGN_OR_RETURN(
            interface_name_to_interface_data[interface_name],
            GetForceTorqueCommandDataFromEcm(
                hwm_resource_name, interface_name,
                interface_config.force_torque_command(), ecm));
        break;
      }
      case intrinsic_proto::sim::SimHardwareInterface::kForceTorqueStatus: {
        INTR_ASSIGN_OR_RETURN(interface_name_to_interface_data[interface_name],
                              GetForceTorqueStatusDataFromEcm(
                                  hwm_resource_name, interface_name,
                                  interface_config.force_torque_status(), ecm));
        break;
      }
      case intrinsic_proto::sim::SimHardwareInterface::kJointLimitsCommand: {
        INTR_ASSIGN_OR_RETURN(interface_name_to_interface_data[interface_name],
                              GetJointLimitsCommandDataFromEcm(
                                  hwm_resource_name, interface_name,
                                  interface_config.joint_limits_command(),
                                  joint_groups_by_name, ecm));
        break;
      }
      case intrinsic_proto::sim::SimHardwareInterface::kRangefinderStatus: {
        INTR_ASSIGN_OR_RETURN(interface_name_to_interface_data[interface_name],
                              GetRangefinderStatusDataFromEcm(
                                  hwm_resource_name, interface_name,
                                  interface_config.rangefinder_status(), ecm));
        // Get rangefinder topic from ECM.
        auto rangefinder_status_data = std::get<RangefinderStatusData>(
            interface_name_to_interface_data[interface_name]);
        const auto& rangefinder_entity =
            rangefinder_status_data.rangefinder_entity;
        std::optional<sdf::Sensor> gpu_lidar_component_data =
            ecm.ComponentData<gz::sim::components::GpuLidar>(
                rangefinder_entity);
        rangefinder_topic_by_entity[rangefinder_entity] =
            gpu_lidar_component_data->Topic();
        break;
      }
      default:
        break;
    }
  }

  return interface_name_to_interface_data;
}

absl::StatusOr<StrictJointPositionCommandData>
GetStrictJointPositionCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointPositionCommandInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm) {
  if (!interface_config.strict()) {
    return absl::InvalidArgumentError(
        absl::StrCat("The ", interface_config.GetMetadata().descriptor->name(),
                     " hardware interface '", interface_name,
                     "' in simulated HWM '", hwm_name,
                     "' is configured to be non-strict, but something called "
                     "GetStrictJointPositionCommandDataFromEcm() on it"));
  }
  INTR_ASSIGN_OR_RETURN(auto interface_data,
                        GetStrictOrNonStrictJointPositionCommandDataFromEcm(
                            hwm_name, interface_name, interface_config,
                            joint_groups_by_name, ecm));
  if (!std::holds_alternative<StrictJointPositionCommandData>(interface_data)) {
    return absl::InternalError(
        absl::StrCat("Got NonStrictJointPositionCommandData for interface '",
                     interface_name, "' of simulated HWM '", hwm_name,
                     "', but expected StrictJointPositionCommandData. Please "
                     "report this as a bug"));
  }
  return std::move(std::get<StrictJointPositionCommandData>(interface_data));
}

absl::StatusOr<NonStrictJointPositionCommandData>
GetNonStrictJointPositionCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointPositionCommandInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm) {
  if (interface_config.strict()) {
    return absl::InvalidArgumentError(
        absl::StrCat("The ", interface_config.GetMetadata().descriptor->name(),
                     " hardware interface '", interface_name,
                     "' in simulated HWM '", hwm_name,
                     "' is configured to be strict, but something called "
                     "GetNonStrictJointPositionCommandDataFromEcm() on it"));
  }
  INTR_ASSIGN_OR_RETURN(auto interface_data,
                        GetStrictOrNonStrictJointPositionCommandDataFromEcm(
                            hwm_name, interface_name, interface_config,
                            joint_groups_by_name, ecm));
  if (!std::holds_alternative<NonStrictJointPositionCommandData>(
          interface_data)) {
    return absl::InternalError(
        absl::StrCat("Got StrictJointPositionCommandData for interface '",
                     interface_name, "' of simulated HWM '", hwm_name,
                     "', but expected NonStrictJointPositionCommandData. "
                     "Please report this as a bug"));
  }
  return std::move(std::get<NonStrictJointPositionCommandData>(interface_data));
}

absl::StatusOr<StrictJointTorqueCommandData>
GetStrictJointTorqueCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointTorqueCommandInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm) {
  if (!interface_config.strict()) {
    return absl::InvalidArgumentError(
        absl::StrCat("The ", interface_config.GetMetadata().descriptor->name(),
                     " hardware interface '", interface_name,
                     "' in simulated HWM '", hwm_name,
                     "' is configured to be non-strict, but something called "
                     "GetStrictJointPositionCommandDataFromEcm() on it"));
  }
  INTR_ASSIGN_OR_RETURN(auto interface_data,
                        GetStrictOrNonStrictJointTorqueCommandDataFromEcm(
                            hwm_name, interface_name, interface_config,
                            joint_groups_by_name, ecm));
  if (!std::holds_alternative<StrictJointTorqueCommandData>(interface_data)) {
    return absl::InternalError(
        absl::StrCat("Got NonStrictJointTorqueCommandData for interface '",
                     interface_name, "' of simulated HWM '", hwm_name,
                     "', but expected StrictJointTorqueCommandData. Please "
                     "report this as a bug"));
  }
  return std::move(std::get<StrictJointTorqueCommandData>(interface_data));
}

absl::StatusOr<NonStrictJointTorqueCommandData>
GetNonStrictJointTorqueCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointTorqueCommandInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm) {
  if (interface_config.strict()) {
    return absl::InvalidArgumentError(
        absl::StrCat("The ", interface_config.GetMetadata().descriptor->name(),
                     " hardware interface '", interface_name,
                     "' in simulated HWM '", hwm_name,
                     "' is configured to be non-strict, but something called "
                     "GetNonStrictJointTorqueCommandDataFromEcm() on it"));
  }
  INTR_ASSIGN_OR_RETURN(auto interface_data,
                        GetStrictOrNonStrictJointTorqueCommandDataFromEcm(
                            hwm_name, interface_name, interface_config,
                            joint_groups_by_name, ecm));
  if (!std::holds_alternative<NonStrictJointTorqueCommandData>(
          interface_data)) {
    return absl::InternalError(
        absl::StrCat("Got StrictJointTorqueCommandData for interface '",
                     interface_name, "' of simulated HWM '", hwm_name,
                     "', but expected NonStrictJointTorqueCommandData. "
                     "Please report this as a bug"));
  }
  return std::move(std::get<NonStrictJointTorqueCommandData>(interface_data));
}

absl::StatusOr<JointPositionStateData> GetJointPositionStateDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointPositionStateInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    gz::sim::EntityComponentManager& ecm) {
  // Look up joint group, error if missing
  INTR_ASSIGN_OR_RETURN(
      auto joint_entities,
      LookupJointGroup(hwm_name,
                       interface_config.GetMetadata().descriptor->name(),
                       interface_name, interface_config.joint_group_name(),
                       joint_groups_by_name, ecm));
  // Add an empty JointPosition component to each joint entity, so that Gazebo
  // writes positions into the ECM in the future
  for (const gz::sim::Entity& joint_entity : joint_entities) {
    // If there is a "pending" JointPositionReset component, seed the
    // JointPosition component with its data.
    if (auto position_reset =
            ecm.ComponentData<gz::sim::components::JointPositionReset>(
                joint_entity);
        position_reset.has_value()) {
      ecm.CreateComponent(joint_entity,
                          gz::sim::components::JointPosition(*position_reset));
    } else {
      ecm.CreateComponent(joint_entity, gz::sim::components::JointPosition());
    }
  }
  return JointPositionStateData{
      .joint_group_name = interface_config.joint_group_name(),
  };
}

absl::StatusOr<JointCommandedPositionData> GetJointCommandedPositionDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointCommandedPositionInterface&
        interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    gz::sim::EntityComponentManager& ecm) {
  // Look up joint group, error if missing
  INTR_ASSIGN_OR_RETURN(
      auto joint_entities,
      LookupJointGroup(hwm_name,
                       interface_config.GetMetadata().descriptor->name(),
                       interface_name, interface_config.joint_group_name(),
                       joint_groups_by_name, ecm));

  return JointCommandedPositionData{
      .joint_group_name = interface_config.joint_group_name(),
  };
}

absl::StatusOr<JointVelocityStateData> GetJointVelocityStateDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointVelocityStateInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    gz::sim::EntityComponentManager& ecm) {
  // Look up joint group, error if missing
  INTR_ASSIGN_OR_RETURN(
      auto joint_entities,
      LookupJointGroup(hwm_name,
                       interface_config.GetMetadata().descriptor->name(),
                       interface_name, interface_config.joint_group_name(),
                       joint_groups_by_name, ecm));
  // Add a JointVelocity component to each joint entity, so that Gazebo writes
  // velocities into the ECM.
  for (const gz::sim::Entity& joint_entity : joint_entities) {
    // If there is a "pending" JointVelocityReset component, seed the
    // JointVelocity component with its data.
    if (auto velocity_reset =
            ecm.ComponentData<gz::sim::components::JointVelocityReset>(
                joint_entity);
        velocity_reset.has_value()) {
      ecm.CreateComponent(joint_entity,
                          gz::sim::components::JointVelocity(*velocity_reset));
    } else {
      ecm.CreateComponent(joint_entity, gz::sim::components::JointVelocity());
    }
  }
  return JointVelocityStateData{
      .joint_group_name = interface_config.joint_group_name(),
  };
}

absl::StatusOr<JointAccelerationStateData> GetJointAccelerationStateDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointAccelerationStateInterface&
        interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm) {
  // Look up joint group, error if missing
  INTR_RETURN_IF_ERROR(
      LookupJointGroup(hwm_name,
                       interface_config.GetMetadata().descriptor->name(),
                       interface_name, interface_config.joint_group_name(),
                       joint_groups_by_name, ecm)
          .status());
  return JointAccelerationStateData{
      .joint_group_name = interface_config.joint_group_name(),
  };
}

absl::StatusOr<JointTorqueStateData> GetJointTorqueStateDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointTorqueStateInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    gz::sim::EntityComponentManager& ecm) {
  // Look up joint group, error if missing
  INTR_ASSIGN_OR_RETURN(
      auto joint_entities,
      LookupJointGroup(hwm_name,
                       interface_config.GetMetadata().descriptor->name(),
                       interface_name, interface_config.joint_group_name(),
                       joint_groups_by_name, ecm));
  // Add a JointForce component to each joint entity and initialize it to zero.
  // Gazebo writes updated forces into the ECM after each step, but zero is a
  // safe guess at the start of the simulation.
  for (const gz::sim::Entity& joint_entity : joint_entities) {
    ecm.CreateComponent(joint_entity, gz::sim::components::JointForce({0.0}));
  }
  return JointTorqueStateData{
      .joint_group_name = interface_config.joint_group_name(),
  };
}

absl::StatusOr<KinematicChainPayloadCommandData>
GetKinematicChainPayloadCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::KinematicChainPayloadCommandInterface&
        interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm) {
  // Look up joint group, error if missing
  INTR_RETURN_IF_ERROR(
      LookupJointGroup(hwm_name,
                       interface_config.GetMetadata().descriptor->name(),
                       interface_name, interface_config.joint_group_name(),
                       joint_groups_by_name, ecm)
          .status());
  return KinematicChainPayloadCommandData{
      .joint_group_name = interface_config.joint_group_name(),
  };
}

absl::StatusOr<KinematicChainPayloadStateData>
GetKinematicChainPayloadStateDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::KinematicChainPayloadStateInterface&
        interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm) {
  // Look up joint group, error if missing
  INTR_RETURN_IF_ERROR(
      LookupJointGroup(hwm_name,
                       interface_config.GetMetadata().descriptor->name(),
                       interface_name, interface_config.joint_group_name(),
                       joint_groups_by_name, ecm)
          .status());
  return KinematicChainPayloadStateData{
      .joint_group_name = interface_config.joint_group_name(),
  };
}

absl::StatusOr<KinematicChainProcessWrenchCommandData>
GetKinematicChainProcessWrenchCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::KinematicChainProcessWrenchCommandInterface&
        interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const gz::sim::EntityComponentManager& ecm) {
  // Look up joint group, error if missing
  INTR_RETURN_IF_ERROR(
      LookupJointGroup(hwm_name,
                       interface_config.GetMetadata().descriptor->name(),
                       interface_name, interface_config.joint_group_name(),
                       joint_groups_by_name, ecm)
          .status());
  return KinematicChainProcessWrenchCommandData{
      .joint_group_name = interface_config.joint_group_name(),
  };
}

absl::StatusOr<DigitalInputStatusData> GetDigitalInputStatusDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::DigitalInputInterface& interface_config,
    const gz::sim::EntityComponentManager& ecm) {
  if (interface_config.name_of_object_with_digital_input().empty()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Interface '", interface_name, "' of simulated hardware module '",
        hwm_name,
        "' does not set required parameter name_of_object_with_digital_input"));
  }
  std::vector<gz::sim::Entity> digital_input_parent_entities =
      ecm.EntitiesByComponents(intrinsic::simulation::ResourceName{
          interface_config.name_of_object_with_digital_input()});
  if (digital_input_parent_entities.empty()) {
    return absl::NotFoundError(absl::StrCat(
        "Interface '", interface_name, "' of simulated hardware module '",
        hwm_name, "' refers to a world object named '",
        interface_config.name_of_object_with_digital_input(),
        "', but there are no simulated entities associated with that name"));
  }

  // Find the actual digital input component we care about. This can be a child
  // of any of the entities associated with
  // interface_config.name_of_object_with_digital_input().
  //
  // The logic is slightly tricky:
  // * If `interface` does not have an `input_block_name`, then we want to
  //   ensure there's only one input block (so we need to look at all of them)
  // * If `interface` *does* have an `input_block_name`, we just want to find
  //   that and exit this loop early.
  // * Either way, we want to return an error if we don't find a matching input
  //   block entity.
  //
  // `input_block_entity` helps us to do that:
  // * If it is already set when we're trying to set it, that the parent entity
  //   has more than one block and the interface doesn't have
  //   `input_block_name`, so we should error out.
  // * If it is unset after finishing the for loop, we haven't found a matching
  //   input block entity and should also error out.
  std::string input_parent_name = "N/A";
  std::optional<gz::sim::Entity> input_block_entity = std::nullopt;
  size_t num_inputs = 0;
  for (const gz::sim::Entity& digital_input_parent_entity :
       digital_input_parent_entities) {
    std::vector<gz::sim::Entity> children =
        ecm.ChildrenByComponents(digital_input_parent_entity);
    std::string parent_name = ecm.ComponentData<gz::sim::components::Name>(
                                     digital_input_parent_entity)
                                  .value_or("N/A");
    for (const gz::sim::Entity child : children) {
      const intrinsic::simulation::DigitalInput* input_component =
          ecm.Component<intrinsic::simulation::DigitalInput>(child);
      if (input_component == nullptr) {
        continue;
      }
      std::optional<std::string> input_name =
          ecm.ComponentData<gz::sim::components::Name>(child);
      if (input_name == std::nullopt) {
        // The DigitalInputOutput plugin should have assigned a name to each
        // input/output!
        return absl::InternalError(absl::StrCat(
            "Entity ", digital_input_parent_entity, " for interface '",
            interface_name, "' of simulated HWM '", hwm_name,
            "' has a DigitalInput entity without a name"));
      }
      if (interface_config.input_block_name().empty()) {
        // No input_block_name, so we want there to be exactly one digital
        // input entity
        if (input_block_entity.has_value()) {
          return absl::FailedPreconditionError(absl::StrCat(
              "The Intrinsic World object '",
              interface_config.name_of_object_with_digital_input(),
              "' for interface '", interface_name, "' of simulated HWM '",
              hwm_name, "' has multiple DigitalInput entities: One under '",
              input_parent_name, "' with entity ID ",
              input_block_entity.value(), " and one under '", parent_name,
              "' with entity ID ", child,
              ". The configuration does not tell us which one to use. Please "
              "set input_block_name in your configuration"));
        }
        input_block_entity = child;
        input_parent_name = parent_name;
        num_inputs = input_component->Data().data.size();
      } else if (input_name.value() == interface_config.input_block_name()) {
        // We have an input block name, so the error message is slightly
        // different.
        if (input_block_entity.has_value()) {
          return absl::FailedPreconditionError(absl::StrCat(
              "The Intrinsic World object '",
              interface_config.name_of_object_with_digital_input(),
              "' for interface '", interface_name, "' of simulated HWM '",
              hwm_name, "' has multiple DigitalInput entities named '",
              interface_config.input_block_name(), "'. One under '",
              input_parent_name, "' with entity ID ",
              input_block_entity.value(), " and one under '", parent_name,
              "' with entity ID ", child,
              ". Please check both your configuration and the SceneObject "
              "definition for '",
              interface_config.name_of_object_with_digital_input(), "'"));
        }
        input_block_entity = child;
        input_parent_name = parent_name;
        num_inputs = input_component->Data().data.size();
      }
    }
  }
  if (input_block_entity == std::nullopt) {
    return absl::NotFoundError(absl::StrCat(
        "Could not find the simulated DigitalInput entity for interface '",
        interface_name, "' of simulated HWM '", hwm_name, "'"));
  }
  // Copy bit aliases
  absl::flat_hash_map<size_t, std::string> bit_number_to_alias;
  for (const auto& [bit_number, alias] :
       interface_config.bit_number_to_alias()) {
    bit_number_to_alias[bit_number] = alias;
  }
  return DigitalInputStatusData{
      .digital_input_entity = *input_block_entity,
      .num_inputs = num_inputs,
      .bit_index_to_alias = std::move(bit_number_to_alias),
  };
}

absl::StatusOr<DigitalOutputCommandData> GetDigitalOutputCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::DigitalOutputInterface& interface_config,
    const gz::sim::EntityComponentManager& ecm) {
  if (interface_config.name_of_object_with_digital_output().empty()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Interface '", interface_name,
                     "' of simulated hardware module '", hwm_name,
                     "' does not set required parameter "
                     "name_of_object_with_digital_output"));
  }
  std::vector<gz::sim::Entity> digital_output_parent_entities =
      ecm.EntitiesByComponents(intrinsic::simulation::ResourceName{
          interface_config.name_of_object_with_digital_output()});
  if (digital_output_parent_entities.empty()) {
    return absl::NotFoundError(absl::StrCat(
        "Interface '", interface_name, "' of simulated hardware module '",
        hwm_name, "' refers to a world object named '",
        interface_config.name_of_object_with_digital_output(),
        "', but there are no simulated entities associated with that "
        "name"));
  }

  // Find the actual digital output component we care about. This can be a
  // child of any of the entities associated with
  // interface_config.name_of_object_with_digital_output().
  //
  // The logic is slightly tricky:
  // * If `interface` does not have an `output_block_name`, then we want to
  //   ensure there's only one output block (so we need to look at all of them)
  // * If `interface` *does* have an `output_block_name`, we just want to find
  //   that and exit this loop early.
  // * Either way, we want to return an error if we don't find a matching output
  //   block entity.
  //
  // `output_block_entity` helps us to do that:
  // * If it is already set when we're trying to set it, that the parent entity
  //   has more than one block and the interface doesn't have
  //   `output_block_name`, so we should error out.
  // * If it is unset after finishing the for loop, we haven't found a matching
  //   output block entity and should also error out.
  std::string output_parent_name = "N/A";
  std::optional<gz::sim::Entity> output_block_entity = std::nullopt;
  size_t num_outputs = 0;
  for (const gz::sim::Entity& digital_output_parent_entity :
       digital_output_parent_entities) {
    std::vector<gz::sim::Entity> children =
        ecm.ChildrenByComponents(digital_output_parent_entity);
    std::string parent_name = ecm.ComponentData<gz::sim::components::Name>(
                                     digital_output_parent_entity)
                                  .value_or("N/A");
    for (const gz::sim::Entity child : children) {
      const intrinsic::simulation::DigitalOutput* output_component =
          ecm.Component<intrinsic::simulation::DigitalOutput>(child);
      if (output_component == nullptr) {
        continue;
      }
      std::optional<std::string> output_name =
          ecm.ComponentData<gz::sim::components::Name>(child);
      if (output_name == std::nullopt) {
        // The DigitalInputOutput plugin should have assigned a name to each
        // input/output!
        return absl::InternalError(absl::StrCat(
            "Entity ", digital_output_parent_entity, " for interface '",
            interface_name, "' of simulated HWM '", hwm_name,
            "' has a DigitalOutput entity without a name"));
      }
      if (interface_config.output_block_name().empty()) {
        // No output_block_name, so we want there to be exactly one digital
        // output entity
        if (output_block_entity.has_value()) {
          return absl::FailedPreconditionError(absl::StrCat(
              "The Intrinsic World object '",
              interface_config.name_of_object_with_digital_output(),
              "' for interface '", interface_name, "' of simulated HWM '",
              hwm_name, "' has multiple DigitalOutput entities: One under '",
              output_parent_name, "' with entity ID ",
              output_block_entity.value(), " and one under '", parent_name,
              "' with entity ID ", child,
              ". The configuration does not tell us which one to use. Please "
              "set output_block_name in your configuration"));
        }
        output_block_entity = child;
        num_outputs = output_component->Data().data.size();
      } else if (output_name.value() == interface_config.output_block_name()) {
        // We have an output block name, so the error message is slightly
        // different.
        if (output_block_entity.has_value()) {
          return absl::FailedPreconditionError(absl::StrCat(
              "The Intrinsic World object '",
              interface_config.name_of_object_with_digital_output(),
              "' for interface '", interface_name, "' of simulated HWM '",
              hwm_name, "' has multiple DigitalOutput entities named '",
              interface_config.output_block_name(), "'. One under '",
              output_parent_name, "' with entity ID ",
              output_block_entity.value(), " and one under '", parent_name,
              "' with entity ID ", child,
              ". Please check both your configuration and the SceneObject "
              "definition for '",
              interface_config.name_of_object_with_digital_output(), "'"));
        }
        output_block_entity = child;
        num_outputs = output_component->Data().data.size();
      }
    }
  }
  if (output_block_entity == std::nullopt) {
    return absl::NotFoundError(absl::StrCat(
        "Could not find the simulated DigitalOutput entity for interface "
        "'",
        interface_name, "' of simulated HWM '", hwm_name, "'"));
  }
  // Copy bit aliases
  absl::flat_hash_map<size_t, std::string> bit_number_to_alias;
  for (const auto& [bit_number, alias] :
       interface_config.bit_number_to_alias()) {
    bit_number_to_alias[bit_number] = alias;
  }
  return DigitalOutputCommandData{
      .digital_output_entity = *output_block_entity,
      .num_outputs = num_outputs,
      .bit_index_to_alias = std::move(bit_number_to_alias),
  };
}

absl::StatusOr<ForceTorqueCommandData> GetForceTorqueCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::ForceTorqueCommandInterface& interface_config,
    const gz::sim::EntityComponentManager& ecm) {
  INTR_ASSIGN_OR_RETURN(
      gz::sim::Entity ft_sensor_entity,
      FindForceTorqueSensorEntity(
          hwm_name, interface_name,
          interface_config.name_of_object_with_force_torque_sensor(),
          interface_config.ft_sensor_name(), ecm));
  return ForceTorqueCommandData{
      .ft_sensor_entity = ft_sensor_entity,
  };
}

absl::StatusOr<ForceTorqueStatusData> GetForceTorqueStatusDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::ForceTorqueStatusInterface& interface_config,
    gz::sim::EntityComponentManager& ecm) {
  INTR_ASSIGN_OR_RETURN(
      gz::sim::Entity ft_sensor_entity,
      FindForceTorqueSensorEntity(
          hwm_name, interface_name,
          interface_config.name_of_object_with_force_torque_sensor(),
          interface_config.ft_sensor_name(), ecm));
  // Add WrenchMeasured component to ft_sensor_entity, so that Gazebo writes
  // sensor readings into the ECM.
  ecm.CreateComponent(ft_sensor_entity, gz::sim::components::WrenchMeasured());
  // Add ForceTorqueTaring component to ft_sensor_entity with default values.
  ecm.CreateComponent(ft_sensor_entity,
                      intrinsic::simulation::ForceTorqueTaring());
  return ForceTorqueStatusData{
      .ft_sensor_entity = ft_sensor_entity,
  };
}

absl::StatusOr<JointLimitsCommandData> GetJointLimitsCommandDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::JointLimitsCommandInterface& interface_config,
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    gz::sim::EntityComponentManager& ecm) {
  // Look up joint group, error if missing
  INTR_ASSIGN_OR_RETURN(
      auto joint_entities,
      LookupJointGroup(hwm_name,
                       interface_config.GetMetadata().descriptor->name(),
                       interface_name, interface_config.joint_group_name(),
                       joint_groups_by_name, ecm));
  return JointLimitsCommandData{
      .joint_group_name = interface_config.joint_group_name(),
  };
}

absl::StatusOr<RangefinderStatusData> GetRangefinderStatusDataFromEcm(
    absl::string_view hwm_name, absl::string_view interface_name,
    const intrinsic_proto::sim::RangefinderStatusInterface& interface_config,
    const gz::sim::EntityComponentManager& ecm) {
  INTR_ASSIGN_OR_RETURN(gz::sim::Entity rangefinder_entity,
                        FindRangefinderSensorEntity(
                            hwm_name, interface_name,
                            interface_config.name_of_object_with_rangefinder(),
                            interface_config.rangefinder_name(), ecm));
  return RangefinderStatusData{
      .rangefinder_entity = rangefinder_entity,
  };
}

}  // namespace intrinsic::simulation::hardware_interface_data
