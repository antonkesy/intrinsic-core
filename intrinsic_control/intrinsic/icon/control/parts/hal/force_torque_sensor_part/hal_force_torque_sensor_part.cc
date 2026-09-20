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

#include "intrinsic/icon/control/parts/hal/force_torque_sensor_part/hal_force_torque_sensor_part.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/context.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/feature_interfaces/force_torque_sensor.h"
#include "intrinsic/icon/control/parts/feature_interfaces/standalone_force_torque_sensor.h"
#include "intrinsic/icon/control/parts/hal/force_torque_sensor_part/force_sensor_controller_migration_utils.h"
#include "intrinsic/icon/control/parts/hal/force_torque_sensor_part/hal_force_torque_sensor_part_config.pb.h"
#include "intrinsic/icon/control/parts/hal/v1/hal_part_config.pb.h"
#include "intrinsic/icon/control/parts/hal_realtime_part_base.h"
#include "intrinsic/icon/control/parts/payload_property.h"
#include "intrinsic/icon/control/parts/realtime_part_factory_common.h"
#include "intrinsic/icon/control/parts/realtime_robot_payload.h"
#include "intrinsic/icon/control/services/world_service.h"
#include "intrinsic/icon/hal/default_hardware_interfaces.h"  // IWYU pragma: keep
#include "intrinsic/icon/hal/hardware_module_manager.h"
#include "intrinsic/icon/hal/interfaces/force_torque.fbs.h"
#include "intrinsic/icon/hal/interfaces/hardware_module_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/objects/world_object.h"
#include "intrinsic/world/robot_payload/robot_payload.h"

namespace intrinsic::icon {

PostSensorPayload FromRobotPayload(
    const std::optional<RealtimeRobotPayload>& payload) {
  if (!payload.has_value()) {
    return PostSensorPayload{};
  }
  return PostSensorPayload{.ft_t_cog = payload->tip_t_cog().translation(),
                           .mass_kg = payload->mass()};
}

RealtimeStatusOr<RealtimeRobotPayload> FromPostSensorPayload(
    const PostSensorPayload& payload) {
  return RealtimeRobotPayload::Create(
      payload.mass_kg,
      Pose3d(eigenmath::Quaterniond::Identity(), payload.ft_t_cog),
      eigenmath::Matrix3d::Identity());
}

HalForceTorqueSensorPart::HalForceTorqueSensorPart(
    absl::string_view name, HardwareModuleManager* const manager)
    : HalRealtimePartBase(manager), name_(name) {}

// static
absl::StatusOr<PartPtrAndGenericConfig> HalForceTorqueSensorPart::FromProto(
    PartFactoryContext context,
    const intrinsic_proto::icon::HalForceTorqueSensorPartConfig& config) {
  auto part = std::make_unique<HalForceTorqueSensorPart>(
      context.part_name, context.context.GetHardwareModuleManager());

  // We have an initial branching here. If joint_position_state is not provided,
  // we assume we have a freestanding force torque sensor (not attached to an
  // arm).
  if (!config.has_joint_position_state()) {
    if (!config.has_force_torque_state() ||
        !config.has_force_torque_command()) {
      return absl::InvalidArgumentError(
          "Force torque sensors that are not attached to an arm (i.e., do not "
          "provide joint_position_state), must provide a direct force torque "
          "sensor access through force_torque_state and force_torque_command");
    }

    INTR_ASSIGN_OR_RETURN(
        auto force_torque_status_handle,
        context.context
            .GetHardwareInterfaceHandle<intrinsic_fbs::ForceTorqueStatus>(
                config.force_torque_state().module_name(),
                config.force_torque_state().interface_name()));
    INTR_RETURN_IF_ERROR(
        part->AddHardwareModule(config.force_torque_state().module_name()));
    part->hwm_state_proxy_ =
        part->hardware_module_manager_->GetHardwareModuleProxy(
            config.force_torque_state().module_name());

    INTR_ASSIGN_OR_RETURN(auto force_torque_command_handle,
                          context.context.GetMutableHardwareInterfaceHandle<
                              intrinsic_fbs::ForceTorqueCommand>(
                              config.force_torque_command().module_name(),
                              config.force_torque_command().interface_name()));
    INTR_RETURN_IF_ERROR(
        part->AddHardwareModule(config.force_torque_command().module_name()));

    INTR_ASSIGN_OR_RETURN(std::unique_ptr<StandaloneForceTorqueSensorFeature>
                              force_sensor_feature,
                          StandaloneForceTorqueSensorFeature::Create(
                              config, std::move(force_torque_status_handle),
                              std::move(force_torque_command_handle)));

    INTR_RETURN_IF_ERROR(
        part->RegisterForceTorqueInterface(std::move(force_sensor_feature)));

    intrinsic_proto::icon::GenericPartConfig generic_config;
    generic_config.mutable_standalone_force_torque_sensor_config();

    return PartPtrAndGenericConfig{.part_ptr = std::move(part),
                                   .config = std::move(generic_config)};
  } else {
    // At this point, we assume the force torque sensor is attached to a robot
    // and have a further two cases. The input can be provided either by a
    // device which provides the ForceTorqueStatusHardwareInterface, or it can
    // be provided by a device (robot) that measures the external torque applied
    // on each joint.
    std::optional<ForceTorqueCommandHardwareInterface>
        force_torque_command_handle;
    std::optional<std::variant<ForceTorqueStatusHardwareInterface,
                               JointTorqueStateHardwareInterface>>
        input_status_handle;
    if (config.has_force_torque_state()) {
      // If we are using the force_torque_state interface, the part must also
      // present a force_torque_command interface so that the interaction with
      // the ft sensor can be handled appropriately.
      INTR_ASSIGN_OR_RETURN(
          input_status_handle,
          context.context
              .GetHardwareInterfaceHandle<intrinsic_fbs::ForceTorqueStatus>(
                  config.force_torque_state().module_name(),
                  config.force_torque_state().interface_name()));
      INTR_RETURN_IF_ERROR(
          part->AddHardwareModule(config.force_torque_state().module_name()));
      part->hwm_state_proxy_ =
          part->hardware_module_manager_->GetHardwareModuleProxy(
              config.force_torque_state().module_name());
      if (!config.has_force_torque_command()) {
        return absl::InvalidArgumentError(
            "force_torque_command must be provided if providing "
            "force_torque_state");
      }
      INTR_ASSIGN_OR_RETURN(
          force_torque_command_handle,
          context.context.GetMutableHardwareInterfaceHandle<
              intrinsic_fbs::ForceTorqueCommand>(
              config.force_torque_command().module_name(),
              config.force_torque_command().interface_name()));
      INTR_RETURN_IF_ERROR(
          part->AddHardwareModule(config.force_torque_command().module_name()));
    } else if (config.has_external_joint_torque_state()) {
      // On the other hand, if we are using external_joint_torque_state, the
      // configuration should not have a force_torque_command interface since
      // the commands (tare in particular) will be handled within the part.
      if (config.has_force_torque_command()) {
        return absl::InvalidArgumentError(
            "force_torque_command must NOT be provided if providing "
            "external_joint_torque_state");
      }
      INTR_ASSIGN_OR_RETURN(
          input_status_handle,
          context.context
              .GetHardwareInterfaceHandle<intrinsic_fbs::JointTorqueState>(
                  config.external_joint_torque_state().module_name(),
                  config.external_joint_torque_state().interface_name()));
      INTR_RETURN_IF_ERROR(part->AddHardwareModule(
          config.external_joint_torque_state().module_name()));
      part->hwm_state_proxy_ =
          part->hardware_module_manager_->GetHardwareModuleProxy(
              config.external_joint_torque_state().module_name());
    } else {
      return absl::InvalidArgumentError(
          "Only one of {force_torque_state, external_joint_torque_state} must "
          "be provided in the force torque sensor part config");
    }

    if (!input_status_handle) {
      return absl::InternalError(
          "Did not get an input source. This is an internal error");
    }

    INTR_ASSIGN_OR_RETURN(
        auto joint_position_handle,
        context.context
            .GetHardwareInterfaceHandle<intrinsic_fbs::JointPositionState>(
                config.joint_position_state().module_name(),
                config.joint_position_state().interface_name()));
    INTR_RETURN_IF_ERROR(
        part->AddHardwareModule(config.joint_position_state().module_name()));

    std::optional<JointVelocityStateHardwareInterface> joint_velocity_handle =
        std::nullopt;
    if (config.has_joint_velocity_state()) {
      INTR_ASSIGN_OR_RETURN(
          joint_velocity_handle,
          context.context
              .GetHardwareInterfaceHandle<intrinsic_fbs::JointVelocityState>(
                  config.joint_velocity_state().module_name(),
                  config.joint_velocity_state().interface_name()));
      INTR_RETURN_IF_ERROR(
          part->AddHardwareModule(config.joint_velocity_state().module_name()));
    }

    std::optional<JointAccelerationStateHardwareInterface>
        joint_acceleration_handle;
    if (config.has_joint_acceleration_state()) {
      INTR_ASSIGN_OR_RETURN(
          joint_acceleration_handle,
          context.context.GetHardwareInterfaceHandle<
              intrinsic_fbs::JointAccelerationState>(
              config.joint_acceleration_state().module_name(),
              config.joint_acceleration_state().interface_name()));
      INTR_RETURN_IF_ERROR(part->AddHardwareModule(
          config.joint_acceleration_state().module_name()));
    }

    const WorldService* world_service =
        context.context.GetServiceOrNull<WorldService>();
    if (world_service == nullptr) {
      return absl::NotFoundError(
          "HalForceTorqueSensorPart failed to get a valid WorldService");
    }

    std::string world_robot_collection_name;
    if (!config.world_robot_collection_name().empty()) {
      world_robot_collection_name = config.world_robot_collection_name();
    } else {
      world_robot_collection_name = context.hardware_resource_name;
    }

    std::shared_ptr<const world::ObjectWorldClient> world_client =
        world_service->GetObjectWorldClient();
    // First find the robot in the world.
    if (world_robot_collection_name.empty()) {
      // If we don't have a name, but there is exactly one robot (aka
      // KinematicObject) in the World, use that.
      INTR_ASSIGN_OR_RETURN(std::vector<world::WorldObject> objects,
                            world_client->ListObjects());
      std::vector<std::string> kinematic_object_names;
      for (const auto& object : objects) {
        if (object.Proto().has_kinematic_object_component()) {
          kinematic_object_names.push_back(object.Name().value());
        }
      }
      if (kinematic_object_names.size() != 1) {
        return absl::InvalidArgumentError(absl::StrCat(
            "ForceSensorController does not have robot_collection_name, but "
            "there are multiple robots in the World. Please set "
            "robot_collection_name to disambiguate between them. The available "
            "robots are:",
            absl::StrJoin(kinematic_object_names, ", ")));
      }
      world_robot_collection_name = kinematic_object_names.front();
    } else {
      // If we do have a name, ForceTorqueSensorFeature will confirm that it's
      // valid, nothing to do here.
    }

    INTR_ASSIGN_OR_RETURN(
        std::unique_ptr<ForceTorqueSensorFeature> force_sensor_feature,
        ForceTorqueSensorFeature::Create(
            config, world_service, *std::move(input_status_handle),
            std::move(force_torque_command_handle),
            std::move(joint_position_handle), world_robot_collection_name,
            context.control_frequency_hz, std::move(joint_velocity_handle),
            std::move(joint_acceleration_handle)));

    INTR_RETURN_IF_ERROR(
        part->RegisterForceTorqueInterface(std::move(force_sensor_feature)));

    // Register part properties.
    PartPropertyIds part_property_ids;
    // TODO (b/265641836): Remove config for default mounted payload and read
    // from world.
    RobotPayload default_mounted_payload;
    INTR_RETURN_IF_ERROR(
        default_mounted_payload.SetMass(config.support_mass()));
    eigenmath::Vector3d ft_t_cog{config.ft_t_cog().at(0),
                                 config.ft_t_cog().at(1),
                                 config.ft_t_cog().at(2)};
    INTR_RETURN_IF_ERROR(default_mounted_payload.SetTipTCog(Pose3d(ft_t_cog)));
    INTR_ASSIGN_OR_RETURN(part_property_ids.mounted_payload,
                          PayloadProperty::Create(kMountedPayloadPropertyName,
                                                  default_mounted_payload,
                                                  context.property_registry));

    INTR_ASSIGN_OR_RETURN(
        part_property_ids.grasped_payload,
        PayloadProperty::Create(kGraspedPayloadPropertyName, RobotPayload(),
                                context.property_registry));

    part->ft_sensor_part_property_ids_ = part_property_ids;

    // Fill the GenericPartConfig proto.
    intrinsic_proto::icon::GenericPartConfig generic_config;
    generic_config.mutable_force_torque_sensor_config();

    return PartPtrAndGenericConfig{.part_ptr = std::move(part),
                                   .config = std::move(generic_config)};
  }
}

absl::Status HalForceTorqueSensorPart::RegisterForceTorqueInterface(
    std::variant<std::unique_ptr<ForceTorqueSensorFeature>,
                 std::unique_ptr<StandaloneForceTorqueSensorFeature>>
        feature_interface) {
  if (std::holds_alternative<std::unique_ptr<ForceTorqueSensorFeature>>(
          feature_interface)) {
    INTR_RETURN_IF_ERROR(interface_registry_.RegisterAsCompatibleInterfaces(
        std::get<std::unique_ptr<ForceTorqueSensorFeature>>(feature_interface)
            .get()));
    force_torque_sensor_feature_ = std::move(feature_interface);
  } else if (std::holds_alternative<
                 std::unique_ptr<StandaloneForceTorqueSensorFeature>>(
                 feature_interface)) {
    INTR_RETURN_IF_ERROR(interface_registry_.RegisterAsCompatibleInterfaces(
        std::get<std::unique_ptr<StandaloneForceTorqueSensorFeature>>(
            feature_interface)
            .get()));
    force_torque_sensor_feature_ = std::move(feature_interface);
  }
  return absl::OkStatus();
}

RealtimeStatus HalForceTorqueSensorPart::ReadStatus(
    ReadStatusParameters params) {
  intrinsic_fbs::StateCode new_state =
      hwm_state_proxy_->GetHardwareModuleState()->code();
  if (state_ == intrinsic_fbs::StateCode::kMotionEnabling &&
      new_state == intrinsic_fbs::StateCode::kMotionEnabled) {
    // If the part becomes enabled, reset the ForceTorqueSensorFeature.
    INTRINSIC_RT_LOG(INFO) << "Resetting FT sensor";
    RealtimeStatus reset_status;
    if (std::holds_alternative<
            std::unique_ptr<StandaloneForceTorqueSensorFeature>>(
            force_torque_sensor_feature_)) {
      reset_status =
          std::get<std::unique_ptr<StandaloneForceTorqueSensorFeature>>(
              force_torque_sensor_feature_)
              ->Reset();
    } else if (std::holds_alternative<
                   std::unique_ptr<ForceTorqueSensorFeature>>(
                   force_torque_sensor_feature_)) {
      reset_status = std::get<std::unique_ptr<ForceTorqueSensorFeature>>(
                         force_torque_sensor_feature_)
                         ->Reset();
    }
    QCHECK(reset_status.ok()) << reset_status.message();
  }
  state_ = new_state;

  if (std::holds_alternative<
          std::unique_ptr<StandaloneForceTorqueSensorFeature>>(
          force_torque_sensor_feature_)) {
    INTRINSIC_RT_RETURN_IF_ERROR(
        std::get<std::unique_ptr<StandaloneForceTorqueSensorFeature>>(
            force_torque_sensor_feature_)
            ->ReadStatus(params));
  } else if (std::holds_alternative<std::unique_ptr<ForceTorqueSensorFeature>>(
                 force_torque_sensor_feature_)) {
    INTRINSIC_RT_RETURN_IF_ERROR(
        std::get<std::unique_ptr<ForceTorqueSensorFeature>>(
            force_torque_sensor_feature_)
            ->ReadStatus(params));

    INTRINSIC_RT_ASSIGN_OR_RETURN(
        RealtimeRobotPayload mounted_payload,
        FromPostSensorPayload(
            std::get<std::unique_ptr<ForceTorqueSensorFeature>>(
                force_torque_sensor_feature_)
                ->GetMountedPayload()));
    INTRINSIC_RT_RETURN_IF_ERROR(
        ft_sensor_part_property_ids_.mounted_payload.Write(
            params.part_properties, mounted_payload));

    // Publish the current grasped payload in part properties.
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        RealtimeRobotPayload grasped_payload,
        FromPostSensorPayload(
            std::get<std::unique_ptr<ForceTorqueSensorFeature>>(
                force_torque_sensor_feature_)
                ->GetGraspedPayload()));
    INTRINSIC_RT_RETURN_IF_ERROR(
        ft_sensor_part_property_ids_.grasped_payload.Write(
            params.part_properties, grasped_payload));
  }
  return OkStatus();
}

RealtimeStatus HalForceTorqueSensorPart::ApplyCommand(
    ApplyCommandParameters params) {
  if (std::holds_alternative<
          std::unique_ptr<StandaloneForceTorqueSensorFeature>>(
          force_torque_sensor_feature_)) {
    INTRINSIC_RT_RETURN_IF_ERROR(
        std::get<std::unique_ptr<StandaloneForceTorqueSensorFeature>>(
            force_torque_sensor_feature_)
            ->ApplyCommand(params));
  } else if (std::holds_alternative<std::unique_ptr<ForceTorqueSensorFeature>>(
                 force_torque_sensor_feature_)) {
    INTRINSIC_RT_RETURN_IF_ERROR(
        std::get<std::unique_ptr<ForceTorqueSensorFeature>>(
            force_torque_sensor_feature_)
            ->ApplyCommand(params));
    // Extract the mounted payload from the properties and save them to the part
    // variables.
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        std::optional<RealtimeRobotPayload> mounted_payload,
        ft_sensor_part_property_ids_.mounted_payload.Read(
            params.part_properties));
    std::get<std::unique_ptr<ForceTorqueSensorFeature>>(
        force_torque_sensor_feature_)
        ->SetMountedPayload(FromRobotPayload(mounted_payload));

    // Extract the grasped payload from the properties and save them to the part
    // variables.
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        std::optional<RealtimeRobotPayload> grasped_payload,
        ft_sensor_part_property_ids_.grasped_payload.Read(
            params.part_properties));
    std::get<std::unique_ptr<ForceTorqueSensorFeature>>(
        force_torque_sensor_feature_)
        ->SetGraspedPayload(FromRobotPayload(grasped_payload));
  }
  return OkStatus();
}

}  // namespace intrinsic::icon
