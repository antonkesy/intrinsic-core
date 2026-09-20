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

#include "intrinsic/simulation/gazebo/plugins/hardware_module_launcher.h"

#include <chrono>  // NOLINT(build/c++11)
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <future>  // NOLINT(build/c++11)
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "grpcpp/security/server_credentials.h"
#include "grpcpp/server_builder.h"
#include "grpcpp/server_context.h"
#include "gz/common/Profiler.hh"
#include "gz/msgs/MessageTypes.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Events.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Types.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "intrinsic/assets/services/proto/v1/service_state.pb.h"
#include "intrinsic/icon/hal/command_validator.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_module_interface.h"
#include "intrinsic/icon/hal/hardware_module_runtime.h"
#include "intrinsic/icon/hal/hardware_module_util.h"
#include "intrinsic/icon/hal/interfaces/force_torque.fbs.h"
#include "intrinsic/icon/hal/interfaces/io_controller.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/rangefinder.fbs.h"
#include "intrinsic/icon/hal/module_config.h"
#include "intrinsic/icon/hal/proto/hardware_module_config.pb.h"
#include "intrinsic/icon/hal/realtime_clock.h"
#include "intrinsic/icon/hardware_modules/sim_bus/sim_bus_hardware_module.pb.h"
#include "intrinsic/icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "intrinsic/icon/server/config/dio_config.pb.h"
#include "intrinsic/icon/server/config/icon_main_config.pb.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/resources/proto/resource_operational_status.pb.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "intrinsic/simulation/gazebo/asset_instances_client.h"
#include "intrinsic/simulation/gazebo/asset_instances_service_address_flag.h"
#include "intrinsic/simulation/gazebo/components/ppr_component.h"
#include "intrinsic/simulation/gazebo/components/service_state_component.h"
#include "intrinsic/simulation/gazebo/plugins/ecm_hardware_interface_conversion.h"
#include "intrinsic/simulation/gazebo/plugins/gazebo_hwm.h"
#include "intrinsic/simulation/gazebo/plugins/gravity_compensator.h"
#include "intrinsic/simulation/gazebo/plugins/infer_hardware_interfaces.h"
#include "intrinsic/simulation/gazebo/plugins/priority_constants.h"
#include "intrinsic/simulation/gazebo/plugins/sim_hardware_interface_data.h"
#include "intrinsic/simulation/gazebo/plugins/sim_hardware_module_config.pb.h"
#include "intrinsic/util/status/return.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/stop_token.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/util/thread/thread_options.h"
#include "intrinsic/util/thread/thread_utils.h"
#include "sdf/Element.hh"
#include "sdf/Joint.hh"
#include "sdf/Model.hh"

// TODO(scpeters@): change joint control based on this flag.
ABSL_FLAG(bool, use_sim_velocity_control, false,
          "Uses joint velocity control commands in simulation instead of "
          "resetting joint position and velocity to the desired trajectory.");

ABSL_FLAG(std::string, shared_memory_namespace_testonly, "",
          "Prefix for all shared memory connections (used for simulated "
          "hardware modules). Passing unique namespace is needed to make "
          "integration tests hermetic.");

namespace intrinsic::simulation {
namespace {

// Returns a map from each hardware module name to ICON instance name that
// controls that HWM.
absl::StatusOr<absl::flat_hash_map<std::string, std::string>>
LookupHwmToIconMapping(const AssetInstancesClient& client) {
  INTR_ASSIGN_OR_RETURN(auto instances, client.ListServiceAssets());

  absl::flat_hash_map<std::string, std::string> out;
  for (const auto& instance : instances) {
    intrinsic_proto::icon::IconMainConfig icon_main_config;
    if (!instance.config().service().service_config().UnpackTo(
            &icon_main_config)) {
      continue;
    }
    for (const auto& hwm_name : icon_main_config.hardware_module_names()) {
      out[hwm_name] = instance.name();
    }
  }
  return out;
}

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

// Finds exactly one entity that has *both* an Intrinsic ResourceName Component
// with `world_object_name` as its data, and a gz::sim::components::ModelSdf
// component.
//
// Returns NotFoundError if there are no matching entities.
// Returns InvalidArgumentError if there is more than one matching entity.
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

absl::StatusOr<absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>>
GetJointGroupEcmEntitiesForProtoConfig(
    const gz::sim::EntityComponentManager& ecm,
    const intrinsic_proto::sim::SimHardwareModuleConfig& sim_hwm_config) {
  absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>
      joint_group_name_to_joint_entities;
  for (const auto& [group_name, joint_group_proto] :
       sim_hwm_config.manual_hardware_interfaces().joint_groups()) {
    std::vector<gz::sim::Entity> joints_in_index_order;
    switch (joint_group_proto.group_descriptor_case()) {
      case intrinsic_proto::sim::JointGroup::GroupDescriptorCase::
          kNameOfObjectWithJointsInImplicitOrder: {
        INTR_ASSIGN_OR_RETURN(
            gz::sim::Entity entity_with_object_name,
            FindSingleEntityWithModelDataForWorldName(
                ecm, joint_group_proto
                         .name_of_object_with_joints_in_implicit_order()));
        // FindSingleEntityWithModelDataForWorldName should guarantee that the
        // entity has a `ModelSdf` component, but better safe than sorry (we
        // don't want to dereference a nullptr!)
        const gz::sim::components::ModelSdf* model_sdf_comp =
            ecm.Component<gz::sim::components::ModelSdf>(
                entity_with_object_name);
        if (!model_sdf_comp) {
          return absl::InternalError(absl::StrCat(
              "Failed to get ModelSdf component for Intrinsic World object '",
              joint_group_proto.name_of_object_with_joints_in_implicit_order(),
              "' even though FindEntitiesWithModelDataForWorldName() should "
              "guarantee its Entity (",
              entity_with_object_name, ") has that component."));
        }
        const ::sdf::Model& model_sdf = model_sdf_comp->Data();
        gz::sim::Model model_gazebo(entity_with_object_name);
        for (std::size_t j = 0; j < model_sdf.JointCount(); ++j) {
          const ::sdf::Joint* joint_sdf = model_sdf.JointByIndex(j);
          gz::sim::Entity joint_entity =
              model_gazebo.JointByName(ecm, joint_sdf->Name());
          if (joint_entity == gz::sim::kNullEntity) {
            return absl::InternalError(absl::StrCat(
                "The Gazebo model for the Intrinsic World object '",
                joint_group_proto
                    .name_of_object_with_joints_in_implicit_order(),
                "' does not have a joint called '", joint_sdf->Name(),
                "'. Gazebo's ECM is inconsistent!"));
          }
          switch (joint_sdf->Type()) {
            case ::sdf::JointType::REVOLUTE:
            case ::sdf::JointType::PRISMATIC:
              joints_in_index_order.push_back(joint_entity);
              break;
            default:
              break;
          }
        }
        break;
      }
      case intrinsic_proto::sim::JointGroup::GroupDescriptorCase::kJointList: {
        for (const intrinsic_proto::sim::ObjectNameAndJointName&
                 object_and_joint : joint_group_proto.joint_list().joints()) {
          INTR_ASSIGN_OR_RETURN(gz::sim::Entity entity_with_object_name,
                                FindSingleEntityWithModelDataForWorldName(
                                    ecm, object_and_joint.world_object_name()));
          // FindSingleEntityWithModelDataForWorldName should guarantee that the
          // entity has a `ModelSdf` component, but better safe than sorry (we
          // don't want to dereference a nullptr!)
          const gz::sim::components::ModelSdf* model_sdf_comp =
              ecm.Component<gz::sim::components::ModelSdf>(
                  entity_with_object_name);
          if (!model_sdf_comp) {
            return absl::InternalError(absl::StrCat(
                "Failed to get ModelSdf component for Intrinsic World object '",
                object_and_joint.world_object_name(),
                "' even though FindEntitiesWithModelDataForWorldName() should "
                "guarantee its Entity (",
                entity_with_object_name, ") has that component."));
          }
          gz::sim::Model model_gazebo(entity_with_object_name);
          gz::sim::Entity joint_entity =
              model_gazebo.JointByName(ecm, object_and_joint.joint_name());
          if (joint_entity == gz::sim::kNullEntity) {
            return absl::NotFoundError(absl::StrCat(
                "The Gazebo model for the Intrinsic World object '",
                object_and_joint.world_object_name(),
                "' does not have a joint called '",
                object_and_joint.joint_name(), "'"));
          }
          switch (
              ecm.ComponentData<gz::sim::components::JointType>(joint_entity)
                  .value_or(::sdf::JointType::INVALID)) {
            case ::sdf::JointType::REVOLUTE:
            case ::sdf::JointType::PRISMATIC:
              joints_in_index_order.push_back(joint_entity);
              break;
            default:
              return absl::InvalidArgumentError(absl::StrCat(
                  "The Gazebo model for the Intrinsic World object '",
                  object_and_joint.world_object_name(),
                  "' has a joint called '", object_and_joint.joint_name(),
                  "', but it is neither a prismatic nor a revolute joint. Only "
                  "those two types are supported."));
          }
        }
        break;
      }
      default:
        return absl::InvalidArgumentError(
            absl::StrCat("Joint group '", group_name,
                         "' does not have a valid group descriptor (World "
                         "object name or joint list)"));
    }
    joint_group_name_to_joint_entities.emplace(
        group_name, std::move(joints_in_index_order));
  }
  return joint_group_name_to_joint_entities;
}

HardwareModuleLauncher::HardwareModuleRuntimeData PrepareRuntimeData(
    const gz::sim::Model& model, const ::gz::sim::UpdateInfo& info,
    const AssetInstancesClient& asset_instances_client,
    ::gz::sim::EntityComponentManager& ecm) {
  GZ_PROFILE("PrepareRuntimeData");
  HardwareModuleLauncher::HardwareModuleRuntimeData data;
  // If we encounter an error, we still want to return a
  // HardwareModuleRuntimeData struct, but with the `init_error` member set.
  auto wrap_error_in_data = [&](const absl::Status& error) {
    data.init_error = error;
    return std::move(data);
  };

  // Set HWM name to resource name – this should be the way to go moving
  // forward, but we allow an override below.
  // (Currently, HWMs can and frequently do define a shared memory name that
  // **doesn't** match the resource name).
  std::string resource_name =
      ecm.ComponentData<ResourceName>(model.Entity()).value_or(kGazeboHwmName);
  LOG(INFO) << "Using resource name '" << resource_name
            << "' to look up HWM config";
  INTR_ASSIGN_OR_RETURN(
      intrinsic_proto::assets::v1::AssetInstance asset_instance,
      asset_instances_client.GetAssetInstance(resource_name),
      _.With(wrap_error_in_data));

  INTR_ASSIGN_OR_RETURN(
      data.hwm_config,
      AssetInstancesClient::ExtractServiceConfig<
          intrinsic_proto::icon::HardwareModuleConfig>(asset_instance),
      _.With(wrap_error_in_data));

  if (asset_instance.details().has_service()) {
    data.service_inspection_topic =
        asset_instance.details().service().service_inspection_topic();
  }

  // If the config doesn't override the HWM name, use the resource name
  // instead (cf.
  // http://intrinsic/icon/hal/hardware_module_main_util.cc;l=50;rcl=646822842)
  if (data.hwm_config.name().empty()) {
    data.hwm_config.set_name(resource_name);
  }
  // Always set the context name, even if the module config has a name.
  data.hwm_config.set_context_name(resource_name);

  INTR_ASSIGN_OR_RETURN(
      data.control_period,
      FindControlPeriod(
          data.hwm_config, model,
          data.hwm_config.simulation_module_config().new_sim_api_config(), ecm,
          asset_instances_client),
      _.With(wrap_error_in_data));
  LOG(INFO) << "[" << data.hwm_config.name() << "]: Control period is "
            << data.control_period;

  // TODO(b/389912730): `control_period` may be part of the model, not the
  // proto config.
  // Populate the config to ensure HardwareModuleRuntime correctly
  // populates the control_period interface.
  data.hwm_config.set_control_period_ns(
      absl::ToInt64Nanoseconds(data.control_period));

  // Infer the manual_hardware_interfaces map, if necessary
  INTR_ASSIGN_OR_RETURN(auto inferred_new_sim_api_config,
                        InferSimHardwareModuleConfig(
                            resource_name, data.hwm_config.name(), model,
                            data.hwm_config.simulation_module_config(), ecm),
                        _.With(wrap_error_in_data));

  // Extract JointGroup entities from ECM
  INTR_ASSIGN_OR_RETURN(
      data.joint_groups_by_name,
      GetJointGroupEcmEntitiesForProtoConfig(ecm, inferred_new_sim_api_config),
      _.With(wrap_error_in_data));

  for (const auto& [group_name, joint_group] : data.joint_groups_by_name) {
    std::vector<std::unique_ptr<GravityCompensator>> gravity_compensators;
    gravity_compensators.reserve(joint_group.size());
    for (size_t i = 0; i < joint_group.size(); ++i) {
      const gz::sim::Entity& joint_entity = joint_group.at(i);
      INTR_ASSIGN_OR_RETURN(
          gravity_compensators.emplace_back(),
          GravityCompensator::Create(joint_entity, &ecm,
                                     /*scope_separator=*/"::"),
          (_ << "building GravityCompensator for Joint entity " << joint_entity
             << " (at index " << i << ") in group '" << group_name
             << "' for simulated HWM '" << data.hwm_config.name() << "'")
              .With(wrap_error_in_data));
    }
    data.gravity_compensators_for_braking_by_joint_group.emplace(
        group_name, std::move(gravity_compensators));
  }

  INTR_ASSIGN_OR_RETURN(data.hardware_interface_name_to_gazebo_data,
                        hardware_interface_data::BuildHardwareInterfaceData(
                            data.hwm_config.name(),
                            /*sim_hwm_config=*/
                            inferred_new_sim_api_config,
                            /*joint_groups_by_name=*/data.joint_groups_by_name,
                            /*ecm=*/ecm,
                            /*rangefinder_topic_by_entity=*/
                            data.rangefinder_topic_by_entity),
                        _.With(wrap_error_in_data));

  return data;
}

// Sets up the GazeboHardwareModule that provides the interface between ICON
// and Gazebo for the model that this HardwareModuleLauncher is attached to.
//
// Returns a struct that bundles the HardwareModuleRuntime, a raw pointer to
// the GazeboHwm, and other related resources on success. In particular, the
// struct's *Nonnull* pointer members (`runtime` and `gazebo_hwm_rawptr`)
// are guaranteed to be non-null.
//
// Returns any error otherwise.
absl::StatusOr<HardwareModuleLauncher::HardwareModuleRuntimeData>
SetupHardwareModule(
    const gz::sim::Model& model, const ::gz::sim::UpdateInfo& info,
    const AssetInstancesClient& asset_instances_client,
    ::gz::sim::EntityComponentManager& ecm,
    std::weak_ptr<icon::SharedPromiseWrapper<icon::HardwareModuleExitCode>>
        exit_code_promise) {
  GZ_PROFILE("SetupHardwareModule");
  HardwareModuleLauncher::HardwareModuleRuntimeData data =
      PrepareRuntimeData(model, info, asset_instances_client, ecm);

  auto gazebo_hwm =
      std::make_unique<GazeboHardwareModule>(GazeboHardwareModule::Config{
          .init_error = data.init_error,
          // GazeboHardwareModule sets up hardware interfaces (and reports
          // errors) in its `Init()` method (part of `runtime->Run()` below).
          .joint_groups_by_name = data.joint_groups_by_name,
          .hardware_interface_name_to_gazebo_data =
              data.hardware_interface_name_to_gazebo_data,
          .hwm_name = data.hwm_config.name(),
      });

  // Hold on to a raw pointer to the HWM. We need to interact with it from the
  // PreUpdate() function, and we need to move the smart pointer into the HWM
  // runtime object.
  data.gazebo_hwm_rawptr = gazebo_hwm.get();
  std::unique_ptr<intrinsic::icon::RealtimeClock> realtime_clock = nullptr;

  std::string shm_namespace =
      absl::GetFlag(FLAGS_shared_memory_namespace_testonly);
  INTR_ASSIGN_OR_RETURN(
      auto shm_manager,
      icon::SharedMemoryManager::Create(shm_namespace, data.hwm_config.name()));

  // True for many HWMs, but not if a single server is connected to multiple
  // HWMs (only one can drive the clock for each ICON server).
  if (data.hwm_config.drives_realtime_clock()) {
    INTR_ASSIGN_OR_RETURN(realtime_clock,
                          icon::RealtimeClock::Create(*shm_manager));

    LOG(INFO) << "Created RealtimeClock";
    (void)realtime_clock->Reset(absl::InfiniteDuration());
  }
  data.hwm_config.set_disable_malloc_guard(true);
  {
    icon::ModuleConfig config(
        /*config=*/data.hwm_config, /*shared_memory_namespace=*/
        absl::GetFlag(FLAGS_shared_memory_namespace_testonly),
        /*realtime_clock=*/realtime_clock.get());
    INTR_ASSIGN_OR_RETURN(auto runtime,
                          icon::HardwareModuleRuntime::Create(
                              std::move(shm_manager),
                              icon::HardwareModule{
                                  .realtime_clock = std::move(realtime_clock),
                                  .instance = std::move(gazebo_hwm),
                                  .config = std::move(config),
                              },
                              exit_code_promise));
    data.runtime = std::move(runtime);
  }

  // The Gazebo HWM does not register any gRPC services. So we pass a gRPC
  // server builder to the runtime, but don't start the server.
  grpc::ServerBuilder server_builder;
  INTR_RETURN_IF_ERROR(data.runtime->Run(server_builder, /*is_realtime=*/false,
                                         {}, data.service_inspection_topic));

  LOG(INFO) << "HardwareModuleLauncher::SetupHardwareModule() done for "
            << model.Name(ecm);
  if (data.runtime == nullptr) {
    return absl::InternalError(absl::StrCat(
        "Hardware module runtime for simulated Hardware module '",
        model.Name(ecm), "' is not set. This is a bug, please report it."));
  }
  if (data.gazebo_hwm_rawptr == nullptr) {
    return absl::InternalError(absl::StrCat(
        "Hardware module pointer for simulated Hardware module '",
        model.Name(ecm), "' is not set. This is a bug, please report it."));
  }
  return data;
}

struct ApplyInterfaceDataToEcmVisitor {
  // If a joint group hasn't received either a position or a torque command by
  // the time we've handled all interfaces, we add a HaltMotion component to
  // each joint to keep them from flailing around.
  //
  // Note that joints may still flail around if someone intentionally sends a
  // zero torque command.
  absl::flat_hash_set<std::string>& joint_groups_with_command;
  // If a joint group applies position commands using position reset and
  // velocity reset components, we need to keep track of these groups so that we
  // can properly report the measured joint velocities.
  absl::flat_hash_set<std::string>& joint_groups_with_reset_commands;
  const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
      joint_groups_by_name;
  const GazeboHardwareModule::HardwareInterfaces::Handles&
      hardware_interface_handles;
  const ::gz::sim::UpdateInfo& info;
  const intrinsic::icon::Validator& command_validator;
  gz::sim::EntityComponentManager& ecm;
  absl::string_view interface_name;
  bool use_sim_velocity_control;

  absl::Status operator()(
      hardware_interface_data::StrictJointPositionCommandData&
          strict_jpos_data) {
    auto joint_group =
        joint_groups_by_name.find(strict_jpos_data.joint_group_name);
    if (joint_group == joint_groups_by_name.end()) {
      return absl::InternalError(absl::StrCat(
          "Interface '", interface_name, "' refers to unknown joint group '",
          strict_jpos_data.joint_group_name,
          "'. This should have come up earlier!"));
    }
    INTR_ASSIGN_OR_RETURN(
        const auto* interface_handle,
        hardware_interface_handles
            .GetStrictInterface<intrinsic_fbs::JointPositionCommand>(
                interface_name));
    INTR_ASSIGN_OR_RETURN(
        const intrinsic_fbs::JointPositionCommand* interface_flatbuffer,
        interface_handle->Value());
    if (!command_validator.WasUpdatedThisCycle(*interface_handle).ok()) {
      // It's okay to not receive a command. We might have received a torque
      // command or non-strict position command instead, and will deal with that
      // later.
      //
      // We do want to update the "previous position setpoint" to the current
      // position in this case, though.

      // If there are no previous setpoints, initialize them.
      if (strict_jpos_data.previous_setpoints == std::nullopt) {
        strict_jpos_data.previous_setpoints =
            std::vector<double>(interface_flatbuffer->position()->size(), 0);
      }
      // Read current joint positions into previous_setpoints.
      INTR_RETURN_IF_ERROR(ReadJointPositionsFromEcm(
          ecm, joint_group->second,
          absl::MakeSpan(*strict_jpos_data.previous_setpoints)));
      return absl::OkStatus();
    }
    // If previous_setpoints is unset, populate it with the current setpoints.
    // This should happen *at most* once at the start of the simulation, and
    // even then, only when the ECM doesn't have initial joint positions.
    if (strict_jpos_data.previous_setpoints == std::nullopt) {
      strict_jpos_data.previous_setpoints =
          std::vector<double>{interface_flatbuffer->position()->begin(),
                              interface_flatbuffer->position()->end()};
    }
    joint_groups_with_command.insert(strict_jpos_data.joint_group_name);
    if (use_sim_velocity_control) {
      INTR_RETURN_IF_ERROR(ApplyJointPositionCommandsToEcmUsingVelocityCommand(
          /*dt_seconds=*/std::chrono::duration<double>(info.dt).count(),
          *interface_flatbuffer, joint_group->second, ecm));
    } else {
      INTR_RETURN_IF_ERROR(
          ApplyJointPositionCommandsToEcmUsingPositionResetAndVelocityReset(
              /*dt_seconds=*/std::chrono::duration<double>(info.dt).count(),
              /*previous_position_setpoints*/
              strict_jpos_data.previous_setpoints.value(),
              /*gravity_compensators=*/strict_jpos_data.gravity_compensators,
              *interface_flatbuffer, joint_group->second, ecm));
      joint_groups_with_reset_commands.insert(
          strict_jpos_data.joint_group_name);
    }
    for (int i = 0; i < joint_group->second.size(); ++i) {
      strict_jpos_data.previous_setpoints.value()[i] =
          interface_flatbuffer->position()->Get(i);
    }
    return absl::OkStatus();
  }

  absl::Status operator()(
      hardware_interface_data::NonStrictJointPositionCommandData&
          non_strict_jpos_data) {
    auto joint_group =
        joint_groups_by_name.find(non_strict_jpos_data.joint_group_name);
    if (joint_group == joint_groups_by_name.end()) {
      return absl::InternalError(absl::StrCat(
          "Interface '", interface_name, "' refers to unknown joint group '",
          non_strict_jpos_data.joint_group_name,
          "'. This should have come up earlier!"));
    }
    INTR_ASSIGN_OR_RETURN(
        const auto* interface_handle,
        hardware_interface_handles
            .GetInterface<intrinsic_fbs::JointPositionCommand>(interface_name));
    const intrinsic_fbs::JointPositionCommand& interface_flatbuffer =
        ***interface_handle;
    if (!command_validator.WasUpdatedThisCycle(*interface_handle).ok()) {
      // It's okay to not receive a command. We might have received a torque
      // command or strict position command instead, and will deal with that
      // later.
      //
      // We do want to update the "previous position setpoint" to the current
      // position in this case, though.

      // If there are no previous setpoints, initialize them.
      if (non_strict_jpos_data.previous_setpoints == std::nullopt) {
        non_strict_jpos_data.previous_setpoints =
            std::vector<double>(interface_flatbuffer.position()->size(), 0);
      }
      // Read current joint positions into previous_setpoints.
      INTR_RETURN_IF_ERROR(ReadJointPositionsFromEcm(
          ecm, joint_group->second,
          absl::MakeSpan(*non_strict_jpos_data.previous_setpoints)));
      return absl::OkStatus();
    }
    // If previous_setpoints is unset, populate it with the current setpoints.
    // This should happen *at most* once at the start of the simulation, and
    // even then, only when the ECM doesn't have initial joint positions.
    if (non_strict_jpos_data.previous_setpoints == std::nullopt) {
      non_strict_jpos_data.previous_setpoints =
          std::vector<double>{interface_flatbuffer.position()->begin(),
                              interface_flatbuffer.position()->end()};
    }
    joint_groups_with_command.insert(non_strict_jpos_data.joint_group_name);
    if (use_sim_velocity_control) {
      INTR_RETURN_IF_ERROR(ApplyJointPositionCommandsToEcmUsingVelocityCommand(
          /*dt_seconds=*/std::chrono::duration<double>(info.dt).count(),
          interface_flatbuffer, joint_group->second, ecm));
    } else {
      INTR_RETURN_IF_ERROR(
          ApplyJointPositionCommandsToEcmUsingPositionResetAndVelocityReset(
              /*dt_seconds=*/std::chrono::duration<double>(info.dt).count(),
              /*previous_position_setpoints*/
              non_strict_jpos_data.previous_setpoints.value(),
              /*gravity_compensators=*/
              non_strict_jpos_data.gravity_compensators, interface_flatbuffer,
              joint_group->second, ecm));
      joint_groups_with_reset_commands.insert(
          non_strict_jpos_data.joint_group_name);
    }
    for (int i = 0; i < joint_group->second.size(); ++i) {
      non_strict_jpos_data.previous_setpoints.value()[i] =
          interface_flatbuffer.position()->Get(i);
    }
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::StrictJointTorqueCommandData&
          strict_jtorque_data) {
    auto joint_group =
        joint_groups_by_name.find(strict_jtorque_data.joint_group_name);
    if (joint_group == joint_groups_by_name.end()) {
      return absl::InternalError(absl::StrCat(
          "Interface '", interface_name, "' refers to unknown joint group '",
          strict_jtorque_data.joint_group_name,
          "'. This should have come up earlier!"));
    }
    INTR_ASSIGN_OR_RETURN(
        const auto* interface_handle,
        hardware_interface_handles
            .GetStrictInterface<intrinsic_fbs::JointTorqueCommand>(
                interface_name));
    INTR_ASSIGN_OR_RETURN(
        const intrinsic_fbs::JointTorqueCommand* interface_flatbuffer,
        interface_handle->Value());
    if (!command_validator.WasUpdatedThisCycle(*interface_handle).ok()) {
      // It's okay to not receive a command. We might have received a position
      // command or non-strict torque command instead, and will deal with that
      // later.
      return absl::OkStatus();
    }
    joint_groups_with_command.insert(strict_jtorque_data.joint_group_name);
    INTR_RETURN_IF_ERROR(
        ApplyJointTorqueCommandsToEcm(
            /*gravity_compensators=*/strict_jtorque_data.gravity_compensators,
            *interface_flatbuffer, joint_group->second, ecm))
        .LogError();
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::NonStrictJointTorqueCommandData&
          non_strict_jtorque_data) {
    auto joint_group =
        joint_groups_by_name.find(non_strict_jtorque_data.joint_group_name);
    if (joint_group == joint_groups_by_name.end()) {
      return absl::InternalError(absl::StrCat(
          "Interface '", interface_name, "' refers to unknown joint group '",
          non_strict_jtorque_data.joint_group_name,
          "'. This should have come up earlier!"));
    }
    INTR_ASSIGN_OR_RETURN(
        const auto* interface_handle,
        hardware_interface_handles
            .GetInterface<intrinsic_fbs::JointTorqueCommand>(interface_name));
    const intrinsic_fbs::JointTorqueCommand& interface_flatbuffer =
        ***interface_handle;
    if (!command_validator.WasUpdatedThisCycle(*interface_handle).ok()) {
      // It's okay to not receive a command. We might have received a position
      // command or strict torque command instead, and will deal with that
      // later.
      return absl::OkStatus();
    }
    joint_groups_with_command.insert(non_strict_jtorque_data.joint_group_name);
    INTR_RETURN_IF_ERROR(ApplyJointTorqueCommandsToEcm(
                             /*gravity_compensators=*/non_strict_jtorque_data
                                 .gravity_compensators,
                             interface_flatbuffer, joint_group->second, ecm))
        .LogError();
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::JointCommandedPositionData&
          position_state_data) {
    // No-op (this is a status interface, not a command one)
    return absl::OkStatus();
  }

  absl::Status operator()(const hardware_interface_data::JointPositionStateData&
                              position_state_data) {
    // No-op (this is a status interface, not a command one)
    return absl::OkStatus();
  }

  absl::Status operator()(const hardware_interface_data::JointVelocityStateData&
                              velocity_state_data) {
    // No-op (this is a status interface, not a command one)
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::JointAccelerationStateData&
          acceleration_state_data) {
    // No-op (this is a status interface, not a command one)
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::JointTorqueStateData& torque_state_data) {
    // No-op (this is a status interface, not a command one)
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::KinematicChainPayloadCommandData&
          payload_command_data) {
    auto joint_group =
        joint_groups_by_name.find(payload_command_data.joint_group_name);
    if (joint_group == joint_groups_by_name.end()) {
      return absl::InternalError(absl::StrCat(
          "Interface '", interface_name, "' refers to unknown joint group '",
          payload_command_data.joint_group_name,
          "'. This should have come up earlier!"));
    }
    INTR_ASSIGN_OR_RETURN(
        const auto* interface_handle,
        hardware_interface_handles.GetInterface<intrinsic_fbs::PayloadCommand>(
            interface_name));
    const intrinsic_fbs::PayloadCommand& interface_flatbuffer =
        ***interface_handle;
    return ApplyKinematicChainPayloadCommandToEcm(interface_flatbuffer,
                                                  joint_group->second, ecm);
  }

  absl::Status operator()(
      const hardware_interface_data::KinematicChainPayloadStateData&
          payload_state_data) {
    // No-op (this is a status interface, not a command one)
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::KinematicChainProcessWrenchCommandData&
          process_wrench_command_data) {
    // No-op (for now?)
    return absl::OkStatus();
  }

  absl::Status operator()(const hardware_interface_data::DigitalInputStatusData&
                              digital_input_status_data) {
    // No-op (this is a status interface, not a command one)
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::DigitalOutputCommandData&
          digital_output_command_data) {
    INTR_ASSIGN_OR_RETURN(
        const auto* interface_handle,
        hardware_interface_handles.GetInterface<intrinsic_fbs::DIOCommand>(
            interface_name));
    const intrinsic_fbs::DIOCommand& interface_flatbuffer = ***interface_handle;
    return ApplyDigitalOutputBlockCommandToEcm(
        interface_flatbuffer, digital_output_command_data.digital_output_entity,
        ecm);
  }

  absl::Status operator()(const hardware_interface_data::ForceTorqueCommandData&
                              force_torque_command_data) {
    INTR_ASSIGN_OR_RETURN(
        const auto* interface_handle,
        hardware_interface_handles
            .GetInterface<intrinsic_fbs::ForceTorqueCommand>(interface_name));
    const intrinsic_fbs::ForceTorqueCommand& interface_flatbuffer =
        ***interface_handle;
    return ApplyForceTorqueSensorCommandToEcm(
        interface_flatbuffer, force_torque_command_data.ft_sensor_entity, ecm);
  }

  absl::Status operator()(const hardware_interface_data::ForceTorqueStatusData&
                              force_torque_status_data) {
    // No-op (this is a status interface, not a command one)
    return absl::OkStatus();
  }

  absl::Status operator()(const hardware_interface_data::AnalogInputStatusData&
                              analog_input_status_data) {
    // No-op (this is a status interface, not a command one)
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::AnalogOutputCommandData&
          analog_output_command_data) {
    // No-op (for now?)
    return absl::OkStatus();
  }

  absl::Status operator()(const hardware_interface_data::JointLimitsCommandData&
                              joint_limits_command_data) {
    // No-op (for now?)
    return absl::OkStatus();
  }

  absl::Status operator()(const hardware_interface_data::RangefinderStatusData&
                              rangefinder_status_data) {
    // No-op (this is a status interface, not a command one)
    return absl::OkStatus();
  }
};

struct UpdateInterfaceWithEcmDataVisitor {
  intrinsic::Time now;
  // If a joint group applies position commands using position reset and
  // velocity reset components, we should report the measured joint velocities
  // as the target reset values instead of the measured values.
  absl::flat_hash_set<std::string>& joint_groups_with_reset_commands;
  absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
      joint_groups_by_name;
  GazeboHardwareModule::HardwareInterfaces::Handles& hardware_interface_handles;
  const gz::sim::EntityComponentManager& ecm;
  absl::flat_hash_map<gz::sim::Entity, gz::msgs::LaserScan>&
      rangefinder_data_by_entity;
  absl::string_view interface_name;
  bool use_sim_velocity_control;

  absl::Status operator()(
      const hardware_interface_data::StrictJointPositionCommandData&
          strict_jpos_data) {
    // No-op (this is a command interface, not a status one)
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::NonStrictJointPositionCommandData&
          non_strict_jpos_data) {
    // No-op (this is a command interface, not a status one)
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::StrictJointTorqueCommandData&
          strict_jtorque_data) {
    // No-op (this is a command interface, not a status one)
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::NonStrictJointTorqueCommandData&
          non_strict_jtorque_data) {
    // No-op (this is a command interface, not a status one)
    return absl::OkStatus();
  }

  absl::Status operator()(const hardware_interface_data::JointPositionStateData&
                              position_state_data) {
    auto joint_group =
        joint_groups_by_name.find(position_state_data.joint_group_name);
    if (joint_group == joint_groups_by_name.end()) {
      return absl::InternalError(absl::StrCat(
          "Interface '", interface_name, "' refers to unknown joint group '",
          position_state_data.joint_group_name,
          "'. This should have come up earlier!"));
    }
    INTR_ASSIGN_OR_RETURN(
        auto* interface_handle,
        hardware_interface_handles
            .GetMutableInterface<intrinsic_fbs::JointPositionState>(
                interface_name));
    intrinsic_fbs::JointPositionState& interface_flatbuffer =
        ***interface_handle;
    INTR_RETURN_IF_ERROR(ReadJointPositionsFromEcm(
        ecm, joint_group->second,
        absl::MakeSpan(interface_flatbuffer.mutable_position()->data(),
                       interface_flatbuffer.mutable_position()->size())));
    interface_handle->UpdatedAt(now);
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::JointCommandedPositionData&
          commanded_position_data) {
    auto joint_group =
        joint_groups_by_name.find(commanded_position_data.joint_group_name);
    if (joint_group == joint_groups_by_name.end()) {
      return absl::InternalError(absl::StrCat(
          "Interface '", interface_name, "' refers to unknown joint group '",
          commanded_position_data.joint_group_name,
          "'. This should have come up earlier!"));
    }
    INTR_ASSIGN_OR_RETURN(
        auto* interface_handle,
        hardware_interface_handles
            .GetMutableInterface<intrinsic_fbs::JointCommandedPosition>(
                interface_name));
    intrinsic_fbs::JointCommandedPosition& interface_flatbuffer =
        ***interface_handle;
    INTR_RETURN_IF_ERROR(ReadJointCommandedPositionsFromEcm(
        ecm, joint_group->second,
        absl::MakeSpan(interface_flatbuffer.mutable_position()->data(),
                       interface_flatbuffer.mutable_position()->size())));
    interface_handle->UpdatedAt(now);
    return absl::OkStatus();
  }

  absl::Status operator()(const hardware_interface_data::JointVelocityStateData&
                              velocity_state_data) {
    auto joint_group =
        joint_groups_by_name.find(velocity_state_data.joint_group_name);
    if (joint_group == joint_groups_by_name.end()) {
      return absl::InternalError(absl::StrCat(
          "Interface '", interface_name, "' refers to unknown joint group '",
          velocity_state_data.joint_group_name,
          "'. This should have come up earlier!"));
    }
    INTR_ASSIGN_OR_RETURN(
        auto* interface_handle,
        hardware_interface_handles
            .GetMutableInterface<intrinsic_fbs::JointVelocityState>(
                interface_name));
    intrinsic_fbs::JointVelocityState& interface_flatbuffer =
        ***interface_handle;
    if (!joint_groups_with_reset_commands.contains(
            velocity_state_data.joint_group_name)) {
      INTR_RETURN_IF_ERROR(ReadJointVelocitiesFromEcm(ecm, joint_group->second,
                                                      interface_flatbuffer));
    } else {
      INTR_RETURN_IF_ERROR(ReadJointVelocityResetFromEcm(
          ecm, joint_group->second, interface_flatbuffer));
    }
    interface_handle->UpdatedAt(now);
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::JointAccelerationStateData&
          acceleration_state_data) {
    // No-op (for now? We can't read accelerations from the ECM yet)
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::JointTorqueStateData& torque_state_data) {
    auto joint_group =
        joint_groups_by_name.find(torque_state_data.joint_group_name);
    if (joint_group == joint_groups_by_name.end()) {
      return absl::InternalError(absl::StrCat(
          "Interface '", interface_name, "' refers to unknown joint group '",
          torque_state_data.joint_group_name,
          "'. This should have come up earlier!"));
    }
    INTR_ASSIGN_OR_RETURN(
        auto* interface_handle,
        hardware_interface_handles
            .GetMutableInterface<intrinsic_fbs::JointTorqueState>(
                interface_name));
    intrinsic_fbs::JointTorqueState& interface_flatbuffer = ***interface_handle;
    INTR_RETURN_IF_ERROR(ReadJointTorquesFromEcm(ecm, joint_group->second,
                                                 interface_flatbuffer));
    interface_handle->UpdatedAt(now);
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::KinematicChainPayloadCommandData&
          payload_command_data) {
    // No-op (this is a command interface, not a status one)
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::KinematicChainPayloadStateData&
          payload_state_data) {
    auto joint_group =
        joint_groups_by_name.find(payload_state_data.joint_group_name);
    if (joint_group == joint_groups_by_name.end()) {
      return absl::InternalError(absl::StrCat(
          "Interface '", interface_name, "' refers to unknown joint group '",
          payload_state_data.joint_group_name,
          "'. This should have come up earlier!"));
    }
    INTR_ASSIGN_OR_RETURN(
        auto* interface_handle,
        hardware_interface_handles
            .GetMutableInterface<intrinsic_fbs::PayloadState>(interface_name));
    intrinsic_fbs::PayloadState& interface_flatbuffer = ***interface_handle;
    INTR_RETURN_IF_ERROR(ReadKinematicChainPayloadStateFromEcm(
        ecm, joint_group->second, interface_flatbuffer));
    interface_handle->UpdatedAt(now);
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::KinematicChainProcessWrenchCommandData&
          process_wrench_command_data) {
    // No-op (this is a command interface, not a status one)
    return absl::OkStatus();
  }

  absl::Status operator()(const hardware_interface_data::DigitalInputStatusData&
                              digital_input_status_data) {
    INTR_ASSIGN_OR_RETURN(
        auto* interface_handle,
        hardware_interface_handles
            .GetMutableInterface<intrinsic_fbs::DIOStatus>(interface_name));
    intrinsic_fbs::DIOStatus& interface_flatbuffer = ***interface_handle;
    INTR_RETURN_IF_ERROR(ReadDigitalInputBlockFromEcm(
        ecm, digital_input_status_data.digital_input_entity,
        interface_flatbuffer));
    interface_handle->UpdatedAt(now);
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::DigitalOutputCommandData&
          digital_output_command_data) {
    // No-op (this is a command interface, not a status one)
    return absl::OkStatus();
  }

  absl::Status operator()(const hardware_interface_data::ForceTorqueCommandData&
                              force_torque_command_data) {
    // No-op (this is a command interface, not a status one)
    return absl::OkStatus();
  }

  absl::Status operator()(const hardware_interface_data::ForceTorqueStatusData&
                              force_torque_status_data) {
    INTR_ASSIGN_OR_RETURN(
        auto* interface_handle,
        hardware_interface_handles
            .GetMutableInterface<intrinsic_fbs::ForceTorqueStatus>(
                interface_name));
    intrinsic_fbs::ForceTorqueStatus& interface_flatbuffer =
        ***interface_handle;

    INTR_RETURN_IF_ERROR(ReadForceTorqueSensorFromEcm(
        ecm, force_torque_status_data.ft_sensor_entity, interface_flatbuffer));
    interface_handle->UpdatedAt(now);
    return absl::OkStatus();
  }

  absl::Status operator()(const hardware_interface_data::AnalogInputStatusData&
                              analog_input_status_data) {
    // No-op (for now?)
    return absl::OkStatus();
  }

  absl::Status operator()(
      const hardware_interface_data::AnalogOutputCommandData&
          analog_output_command_data) {
    // No-op (this is a command interface, not a status one)
    return absl::OkStatus();
  }

  absl::Status operator()(const hardware_interface_data::JointLimitsCommandData&
                              joint_limits_command_data) {
    // No-op (this is a command interface, not a status one)
    return absl::OkStatus();
  }

  absl::Status operator()(const hardware_interface_data::RangefinderStatusData&
                              rangefinder_status_data) {
    INTR_ASSIGN_OR_RETURN(
        auto* interface_handle,
        hardware_interface_handles
            .GetMutableInterface<intrinsic_fbs::RangeFinderStatus>(
                interface_name));
    intrinsic_fbs::RangeFinderStatus& interface_flatbuffer =
        ***interface_handle;

    const auto& rangefinder_entity = rangefinder_status_data.rangefinder_entity;
    auto rangefinder_data_it =
        rangefinder_data_by_entity.find(rangefinder_entity);
    // Check if rangefinder data has been added to the map yet. If not, then
    // print a log message.
    if (rangefinder_data_it == rangefinder_data_by_entity.end()) {
      // Try to get the rangefinder name for a better log message.
      auto name_component =
          ecm.Component<gz::sim::components::Name>(rangefinder_entity);
      std::string log_suffix;
      if (name_component) {
        log_suffix = absl::StrCat("named: ", name_component->Data());
      } else {
        log_suffix = absl::StrCat("entity: ", rangefinder_entity);
      }
      LOG_EVERY_N_SEC(WARNING, 5)
          << "No sensor data has been received from rangefinder " << log_suffix;
      return absl::OkStatus();
    }
    const auto& rangefinder_data = rangefinder_data_it->second;
    if (rangefinder_data.ranges_size() == 0 ||
        rangefinder_data.intensities_size() == 0 ||
        std::isinf(rangefinder_data.intensities(0))) {
      LOG_EVERY_N_SEC(WARNING, 5)
          << "Received invalid laser scan from ray sensor: "
          << rangefinder_data;
      return absl::OkStatus();
    }
    interface_flatbuffer.mutate_distance(rangefinder_data.ranges(0));
    interface_flatbuffer.mutate_intensity(rangefinder_data.intensities(0));
    interface_handle->UpdatedAt(now);
    return absl::OkStatus();
  }
};

// Simulates applied brakes on the robot by either forcing the position of all
// `joint_entities` to stay constant, or (if `use_sim_velocity_control` is
// `true`) by sending a zero velocity command to all of them.
absl::Status ApplyBrakes(bool use_sim_velocity_control,
                         ::gz::sim::EntityComponentManager& ecm,
                         absl::Span<const ::gz::sim::Entity> joint_entities,
                         absl::Span<const std::unique_ptr<GravityCompensator>>
                             gravity_compensators) {
  if (use_sim_velocity_control) {
    return ApplyZeroVelocityCommandToEcm(joint_entities, ecm);
  } else {
    return ApplyCurrentPositionJointResetToEcm(joint_entities,
                                               gravity_compensators, ecm);
  }
}

}  // namespace

HardwareModuleLauncher::~HardwareModuleLauncher() {
  if (gazebo_hwm_.ok()) {
    LOG(INFO) << "[" << gazebo_hwm_->gazebo_hwm_rawptr->Name()
              << "] Stopping HWM runtime";
    if (auto stop_status = gazebo_hwm_->runtime->Stop(); !stop_status.ok()) {
      LOG(ERROR) << "[" << gazebo_hwm_->gazebo_hwm_rawptr->Name()
                 << "] Failed to stop HWM runtime: " << stop_status;
    }
  }
}

::gz::sim::System::PriorityType HardwareModuleLauncher::ConfigurePriority() {
  // Use constant from priority_constants.h to ensure this system executes
  // before Physics.
  return plugins::kHardwareModuleLauncherPriority;
}

void HardwareModuleLauncher::Configure(
    const ::gz::sim::Entity& entity,
    const std::shared_ptr<const ::sdf::Element>& sdf,
    ::gz::sim::EntityComponentManager& ecm,
    ::gz::sim::EventManager& event_manager) {
  use_sim_velocity_control_ = absl::GetFlag(FLAGS_use_sim_velocity_control);
  model_ = gz::sim::Model(entity);

  stop_connection_ = event_manager.Connect<gz::sim::events::Stop>(
      std::bind(&HardwareModuleLauncher::OnStop, this));
  if (!model_.Valid(ecm)) {
    LOG(ERROR) << "HardwareModuleLauncher plugin should be attached to a model "
               << "entity. Failed to initialize.";
    return;
  }

  // Use an intermediate variable to avoid assigning a potentially-nullptr value
  // to `asset_instances_client_`, which is marked as absl_nonnull.
  auto asset_instances_client = AssetInstancesClient::Create(
      absl::GetFlag(FLAGS_asset_instances_service_address));
  if (!asset_instances_client.ok()) {
    LOG(ERROR) << "Failed to create AssetInstancesClient: "
               << asset_instances_client.status();
    asset_instances_client_ = asset_instances_client.status();
    return;
  }
  if (*asset_instances_client == nullptr) {
    asset_instances_client_ = absl::InternalError(
        "AssetInstancesClient::Create() returned a nullptr! This is an "
        "internal error, please report it.");
    LOG(ERROR) << asset_instances_client_.status();
  } else {
    asset_instances_client_ = std::move(asset_instances_client);
  }
}

void HardwareModuleLauncher::PreUpdate(const ::gz::sim::UpdateInfo& info,
                                       ::gz::sim::EntityComponentManager& ecm) {
  // Skip first two iterations so that we can be sure both SetModelState and
  // Physics have run at least once
  if (info.iterations < 2) {
    return;
  }
  intrinsic::simulation::ServiceState* service_state_component =
      ecm.ComponentDefault<intrinsic::simulation::ServiceState>(
          model_.Entity());
  bool restart_hwm = restart_hwm_;
  // Before anything else, handle ClearFaults requests.
  if (service_state_component->Data().clear_faults_requested &&
      health_service_.has_value()) {
    grpc::ServerContext server_context;
    intrinsic_proto::services::v1::EnableRequest request;
    intrinsic_proto::services::v1::EnableResponse response;
    // Discard the status of the ClearFaults operation. By the time this line
    // runs, ServiceStateAggregator has already sent a status to whatever
    // called the ClearFault service anyway. We call Enable() here to clear the
    // faults.
    if (auto status =
            health_service_->Enable(&server_context, &request, &response);
        !status.ok()) {
      service_state_component->Data().state->set_state_code(
          intrinsic_proto::services::v1::SelfState::STATE_CODE_ERROR);
      service_state_component->Data()
          .state->mutable_extended_status()
          ->mutable_user_report()
          ->set_message(status.error_message());
      LOG(ERROR) << "Failed to enable hardware module '" << model_.Name(ecm)
                 << "': " << ToAbslStatus(status);
    }

    service_state_component->Data().clear_faults_requested = false;
    LOG(INFO) << "Restarting simulated HWM '" << model_.Name(ecm)
              << "' because of ClearFaults request";
    restart_hwm = true;
  }

  if (restart_hwm) {
    // The health service/ICON wants us to restart the HWM. We implement this by
    // * Stop the existing thread, if any. This is needed to avoid a data race
    // between the thread and the following code.
    restart_request_observer_thread_ = {};
    // * clearing gazebo_hwm_ to stop the existing GazeboHwm
    gazebo_hwm_ = absl::UnavailableError("Gazebo HWM not initialized yet");
    // * setting first_preupdate_ = true, so SetupHardwareModule() runs (see
    //   below)
    first_preupdate_ = true;
    service_state_component->Data().activated_cycle.reset();
    // Since the health service has now used up its promise, we reset it (we
    // also recreate it in the first_update_ block below).
    health_service_.reset();
    // * reset the exit code promise, so that it can be used again.
    shared_exit_code_promise_ = std::make_shared<
        icon::SharedPromiseWrapper<icon::HardwareModuleExitCode>>();
    // * reset the flag, so that we don't try to recreate the HWM again.
    restart_hwm_ = false;
  }

  if (first_preupdate_) {
    first_preupdate_ = false;
    gazebo_icon_time_delta_ = absl::ZeroDuration();
    if (!asset_instances_client_.ok()) {
      gazebo_hwm_ = absl::FailedPreconditionError(absl::StrCat(
          "Cannot set up hardware module because HardwareModuleLauncher could "
          "not connect to the Asset Instances Service: ",
          asset_instances_client_.status()));
    } else {
      // Make sure to discard any previous HWM before building a new one.
      gazebo_hwm_ = absl::UnavailableError("Gazebo HWM not initialized yet");
      gazebo_hwm_ = SetupHardwareModule(model_, info, **asset_instances_client_,
                                        ecm, shared_exit_code_promise_);
    }

    if (gazebo_hwm_.ok()) {
      // Put the icon resource id in the ServiceState for synchronization.
      // Note: the proto side is marked optional, but will be filled either by
      // the user, or by lookup through the resource registry.
      if (service_state_component->Data().icon_resource_id.empty()) {
        service_state_component->Data().icon_resource_id =
            gazebo_hwm_->hwm_config.simulation_module_config()
                .new_sim_api_config()
                .icon_resource_id();
      }
      service_state_component->Data().hwm_name = gazebo_hwm_->hwm_config.name();
      // Subscribe to rangefinder topics.
      for (const auto& [entity, topic] :
           gazebo_hwm_->rangefinder_topic_by_entity) {
        std::function<void(const gz::msgs::LaserScan&)> on_scan =
            [this, entity](const gz::msgs::LaserScan& scan) {
              absl::MutexLock lock(rangefinder_data_mtx_);
              rangefinder_data_by_entity_[entity] = scan;
            };
        node_.Subscribe(topic, on_scan);
      }
    }

    health_service_.emplace(
        std::weak_ptr<icon::SharedPromiseWrapper<icon::HardwareModuleExitCode>>(
            shared_exit_code_promise_));
    if (gazebo_hwm_.ok()) {
      health_service_->SetHardwareModuleRuntime(gazebo_hwm_->runtime.get());
    } else {
      health_service_->ActivateLameDuckMode(gazebo_hwm_.status());
    }
    // We need to create a thread that will observe the
    // shared_exit_code_promise_. We cannot
    // put it in this function since we need to trigger
    // `gazebo_hwm_->gazebo_hwm_rawptr->Shutdown()` asynchronously to avoid
    // waiting for a timeout. PreUpdate() is not called if the HWM is in a fatal
    // fault and ICON stops ticking due to that until said timeout is reached.
    auto restart_request_observer_thread = CreateThread(
        ThreadOptions().SetName("RestartReqObs"),
        [this, name = model_.Name(ecm),
         future = shared_exit_code_promise_->GetSharedFuture()](
            StopToken stop_token) {
          constexpr auto kPollIntervalMs = std::chrono::milliseconds(100);
          while (!stop_token.stop_requested() && future.valid()) {
            if (future.wait_for(kPollIntervalMs) == std::future_status::ready) {
              LOG(INFO) << "Restarting simulated HWM '" << name
                        << "' because of restart request: Exit code:"
                        << static_cast<int>(future.get());
              if (gazebo_hwm_.ok() &&
                  gazebo_hwm_->gazebo_hwm_rawptr != nullptr) {
                if (auto status = gazebo_hwm_->gazebo_hwm_rawptr->Shutdown();
                    !status.ok()) {
                  LOG(ERROR) << "Failed to shutdown GazeboHwm: " << status;
                }
              }
              // Set the flag to recreate the HWM. This will happen in the next
              // PreUpdate() call.
              restart_hwm_ = true;
              break;
            }
          }
          LOG(INFO) << "RestartReqObs thread ended";
        });
    if (!restart_request_observer_thread.ok()) {
      LOG(ERROR) << "Failed to create restart request observer thread: "
                 << restart_request_observer_thread.status();
    } else {
      restart_request_observer_thread_ =
          std::move(restart_request_observer_thread.value());
    }
  }
  if (!gazebo_hwm_.ok()) {
    LOG_EVERY_N_SEC(INFO, 10)
        << "HardwareModuleLauncher for " << model_.Name(ecm)
        << " failed to set up hardware module, ICON will not be able to "
           "connect to simulation.";
    LOG_EVERY_N_SEC(ERROR, 10) << "Error message: " << gazebo_hwm_.status();
    return;
  }

  // Seek and rewind operations set `info.iterations` to zero. We detect this
  // and skip updating the HWM (and ticking the HWM clock) for two reasons:
  //
  // 1. Seek and Rewind can actually lead to negative values for `info.dt`, and
  //    HWMs can't run in reverse.
  // 2. A forward seek could potentially cause a very large `info.dt` and lock
  //    up the simulation while the HWM and ICON try to catch up.
  if (info.iterations == 0) {
    LOG_EVERY_N_SEC(INFO, 5)
        << "Skipping GazeboHWM PreUpdate() because iteration count is zero. "
           "This could mean Gazebo has just started, or that there was a "
           "Seek or Rewind operation.";
    return;
  }

  GZ_PROFILE("HardwareModuleLauncher::PreUpdate");

  if (!gazebo_hwm_.value().gazebo_hwm_rawptr->IsActive()) {
    LOG_EVERY_N_SEC(INFO, 5)
        << "Skipping HWM<->Gazebo update because the HWM is not active";

    // Stop all joints, so that they don't flop around in the time it takes ICON
    // to activate the HWM.
    for (const auto& [joint_group_name, joints] :
         gazebo_hwm_.value().joint_groups_by_name) {
      auto gravity_compensators =
          gazebo_hwm_.value()
              .gravity_compensators_for_braking_by_joint_group.find(
                  joint_group_name);
      if (gravity_compensators ==
          gazebo_hwm_.value()
              .gravity_compensators_for_braking_by_joint_group.end()) {
        LOG_EVERY_N_SEC(ERROR, 10)
            << "There are no gravity compensators for joint group '"
            << joint_group_name << "'";
      }
      INTR_RETURN_IF_ERROR(ApplyBrakes(use_sim_velocity_control_, ecm, joints,
                                       gravity_compensators->second))
          .LogError()
          .With(ReturnVoid());
    }
    return;
  }

  std::string name = gazebo_hwm_.value().gazebo_hwm_rawptr->Name();

  // Look up which ICON server this HWM belongs to on the first cycle that the
  // HWM is active (remember, we return early above if the HWM *isn't* active).
  //
  // This happens in PreUpdate() so that all HardwareModuleLaunchers
  // populate their ServiceStateComponent before Update() synchronizes them
  // based on the ICON resource name (see below).
  if (service_state_component->Data().icon_resource_id.empty() &&
      asset_instances_client_.ok()) {
    // Update the ECM information that stores ICON<->HWM mappings, not just for
    // *this* HWM, but for all of them.
    // We update information for other HWMs because there are tricky edge cases
    // where one HWM receives the `Activate()` call in one simulation step, and
    // another in the next. This leads to a deadlock.
    // By updating the name of the associated ICON instance, we can detect this
    // case, and wait until all HWMs have received the `Activate()` call.
    absl::StatusOr<absl::flat_hash_map<std::string, std::string>>
        hwm_to_icon_mapping = LookupHwmToIconMapping(**asset_instances_client_);
    if (!hwm_to_icon_mapping.ok()) {
      LOG(ERROR) << "Failed to look up HWM to ICON mapping: "
                 << hwm_to_icon_mapping.status();
    }

    ecm.Each<ResourceName, intrinsic::simulation::ServiceState>(
        [&](const gz::sim::Entity& entity, const ResourceName* resource_name,
            intrinsic::simulation::ServiceState* service_state_component) {
          if (!service_state_component->Data().icon_resource_id.empty()) {
            // Don't overwrite the icon resource ID if already set.
            return true;
          }
          if (hwm_to_icon_mapping.ok()) {
            const std::string& hwm_name =
                service_state_component->Data().hwm_name;
            if (!hwm_to_icon_mapping->contains(hwm_name)) {
              std::vector<std::string> keys;
              for (const auto& [k, v] : *hwm_to_icon_mapping) {
                keys.push_back(absl::StrCat("'", k, "' (icon: '", v, "')"));
              }
              LOG_EVERY_N_SEC(WARNING, 10)
                  << "HWM '" << hwm_name
                  << "' was not found in the Resource Registry HWM-to-ICON "
                     "mapping! "
                  << "Available mappings: [" << absl::StrJoin(keys, ", ")
                  << "]. "
                  << "Ensure the HWM name in the simulation matches "
                     "the name in the ICON main configuration.";
              return true;
            }
            service_state_component->Data().icon_resource_id =
                hwm_to_icon_mapping->at(hwm_name);
          }
          return true;
        });
  }
  // Wait until the HWM has finished a tick (i.e. until it has written its
  // control values).
  //
  // NOTE: In the very first tick, this returns immediately – that is, Gazebo
  // runs before the HWM, and there won't necessarily be sensible control
  // values (though we should take care to initialize things sensibly).
  if (icon::RealtimeStatus status =
          gazebo_hwm_->gazebo_hwm_rawptr->WaitForIconTicksToFinish();
      !status.ok()) {
    LOG(INFO) << "[" << gazebo_hwm_->gazebo_hwm_rawptr->Name()
              << "] Failed to start Gazebo tick - HWM might be gone/broken. "
                 "Error message: "
              << status.ToString();
  }

  // Acquire lock for hardware interfaces. The HWM's clock thread holds this
  // while ICON is ticking, so by acquiring the mutex here we wait until the
  // ICON tick is done.
  auto& hardware_interfaces =
      gazebo_hwm_->gazebo_hwm_rawptr->GetHardwareInterfaces();
  {
    absl::MutexLock l(hardware_interfaces.mutex);
    HandleHwmCommands(info, ecm, hardware_interfaces);
    UpdateHwmSensorValues(info, ecm, hardware_interfaces);
  }
}

void HardwareModuleLauncher::Update(const ::gz::sim::UpdateInfo& info,
                                    ::gz::sim::EntityComponentManager& ecm) {
  // Skip first two iterations so that we can be sure both SetModelState and
  // Physics have run at least once
  if (info.iterations < 2) {
    return;
  }
  GZ_PROFILE("HardwareModuleLauncher::Update");
  intrinsic::simulation::ServiceState* service_state_component =
      ecm.ComponentDefault<intrinsic::simulation::ServiceState>(
          model_.Entity());
  if (health_service_.has_value()) {
    // Update ServiceState status. Note that we just delegate to
    // health_service_ here. It does the right thing:
    // * If the GazeboHwm is healthy, the service queries the HWM's
    //   OperationalState
    // * If the GazeboHwm failed to initialize (see the top of PreUpdate), the
    // service has a latched error message that it will forward.
    grpc::ServerContext server_context;
    intrinsic_proto::services::v1::GetStateRequest request;
    intrinsic_proto::services::v1::SelfState response;
    absl::Status health_state = ToAbslStatus(
        health_service_->GetState(&server_context, &request, &response));
    if (!health_state.ok()) {
      service_state_component->Data().state = health_state;
    } else {
      service_state_component->Data().state = response;
    }
  }
  // Same as in PreUpdate(): If `info.iterations` is zero, `info.dt` may take
  // odd values, so skip the Update() step and wait for the next Gazebo step.
  if (info.iterations == 0) {
    LOG_EVERY_N_SEC(INFO, 5)
        << "Skipping GazeboHWM Update() because iteration count is zero.";
    return;
  }
  // End the Gazebo tick and let the HWM do its thing. We don't care about the
  // exact details, and only check that the HWM is done in the next PreUpdate()
  // call.
  if (!gazebo_hwm_.ok()) {
    LOG_EVERY_N_SEC(ERROR, 10)
        << "There was an error setting up the hardware module for model "
        << model_.Name(ecm);
    LOG_EVERY_N_SEC(ERROR, 10) << "Error message: " << gazebo_hwm_.status();
    return;
  }

  // Time is frozen for GazeboHwm while it isn't activated
  if (!gazebo_hwm_.value().gazebo_hwm_rawptr->IsActive()) {
    service_state_component->Data().activated_cycle.reset();
    return;
  }

  // This block does synchronization between hardware_module_launchers. The
  // rough structure is:
  //   * each hardware_module_launcher (hml)
  //     * writes the cycle it first became active into the ServiceState (usage
  //       of ServiceState is just temporary for testing).
  //     * checks the ServiceState of all hml that belong to the same ICON
  //       * if all hml's have an activated cycle set, and the current iteration
  //         is past all of these, then we are free to proceed and step.

  // Note: We filled in the icon_resource_id field in PreUpdate() (above).
  const std::string& icon_resource_id =
      service_state_component->Data().icon_resource_id;
  std::string name = gazebo_hwm_.value().gazebo_hwm_rawptr->Name();
  if (icon_resource_id.empty()) {
    LOG_EVERY_N_SEC(WARNING, 10) << "[" << name
                                 << "] icon_resource_id is empty! Exiting "
                                    "Update early without ticking clock "
                                    "or synchronizing.";
    return;
  }
  if (!service_state_component->Data().activated_cycle.has_value()) {
    LOG(INFO) << "[" << name << ", icon:" << icon_resource_id
              << "] became active, ready to step in next cycle, current iter "
              << info.iterations;
    service_state_component->Data().activated_cycle = info.iterations;
  }
  // Check all other hml to see what cycle they activated.
  bool ready_to_step = true;
  ecm.Each<intrinsic::simulation::ServiceState>(
      [&](const gz::sim::Entity& entity,
          intrinsic::simulation::ServiceState* ss) {
        if (ss->Data().icon_resource_id.empty() ||
            ss->Data().icon_resource_id != icon_resource_id) {
          // Skip this entity, it doesn't belong to the same ICON.
          return true;
        }

        if (!ss->Data().activated_cycle.has_value() ||
            service_state_component->Data().activated_cycle.value() !=
                ss->Data().activated_cycle.value()) {
          ready_to_step = false;

          LOG(INFO)
              << "[" << name << ", icon:" << icon_resource_id
              << "] waiting for other HWM (" << ss->Data().hwm_name
              << ") connected to the same ICON instance (" << icon_resource_id
              << ") to become active, will retry in next cycle, current iter "
              << info.iterations;
          // Try again next cycle
          service_state_component->Data().activated_cycle =
              *service_state_component->Data().activated_cycle + 1;
        }
        return true;
      });
  if (!ready_to_step) {
    LOG(INFO) << "[" << name << ", icon:" << icon_resource_id
              << "] not all ready to step in iter " << info.iterations
              << ", waiting...";
    return;
  }

  LOG_EVERY_N_SEC(INFO, 5) << "Update: Starting Icon tick";
  // Add dt (the sim time elapsed since the last `Update()`) to
  // `gazebo_icon_time_delta_`.
  //
  // We use `gazebo_icon_time_delta_ to account for situations where the control
  // period for ICON (`gazebo_hwm_->control_period`) is different from Gazebo's
  // simulation step size (`info.dt`):
  //
  // * If `gazebo_hwm_->control_period` is *greater than* `info.dt` (ICON is
  //   "slower"):
  //
  //   We only update ICON once `gazebo_icon_time_delta_` adds up to at least
  //   `gazebo_hwm_->control_period`. If `gazebo_hwm_->control_period` is a
  //   clean multiple of `info.dt`, that means exactly one ICON update every N
  //   steps. But even if `gazebo_hwm_->control_period` *isn't* a clean multiple
  //   of `info.dt`, this works, because we don't blindly reset
  //   `gazebo_icon_time_delta_` to zero, but rather subtract
  //   `gazebo_hwm_->control_period` from it each time we update ICON.
  //
  // * If `gazebo_hwm_->control_period` is *less than* `info.dt` (ICON is
  //   "faster"):
  //
  //   We update ICON multiple times per `Update()`. That is
  //   `gazebo_icon_time_delta_` becomes greater than
  //   `gazebo_hwm_->control_period` in a single `Update()` call, and we execute
  //   the while loop below until `gazebo_icon_time_delta_` is *less than*
  //   `gazebo_hwm_->control_period`. This is also robust against non-integer
  //   factors between ICON and Gazebo: In that case, `gazebo_icon_time_delta_`
  //   is non-zero after exiting the while loop.
  gazebo_icon_time_delta_ += absl::FromChrono(info.dt);

  intrinsic::Time gazebo_now = intrinsic::Clock::Zero() + info.simTime;

  while (gazebo_icon_time_delta_ >= gazebo_hwm_->control_period) {
    // ICON ticks forward by one `gazebo_hwm_->control_period`, so we reduce the
    // delta between ICON and Gazebo.
    gazebo_icon_time_delta_ -= gazebo_hwm_->control_period;

    // `GazeboHardwareModule::RequestIconTick()` requires a timestamp that
    // indicates "now" to ICON.
    //
    // We calculate this by going *backwards* from the current sim timestamp
    // (`info.simTime`). Gazebo is `gazebo_icon_time_delta_` ahead of ICON, so
    // we need to subtract that from `gazebo_now` to get the correct ICON
    // timestamp for this ICON tick.
    intrinsic::Time tick_time =
        gazebo_now - absl::ToChronoMicroseconds(gazebo_icon_time_delta_);
    if (icon::RealtimeStatus status =
            gazebo_hwm_->gazebo_hwm_rawptr->RequestIconTick(
                /*sim_time=*/tick_time);
        !status.ok()) {
      LOG(ERROR) << "[" << gazebo_hwm_->gazebo_hwm_rawptr->Name()
                 << "] Failed to request Icon tick: " << status.ToString();
    }
  }
}

void HardwareModuleLauncher::HandleHwmCommands(
    const ::gz::sim::UpdateInfo& info, ::gz::sim::EntityComponentManager& ecm,
    GazeboHardwareModule::HardwareInterfaces& interfaces) {
  GZ_PROFILE("HardwareModuleLauncher::HandleHwmCommands");
  if (gazebo_hwm_.ok()) {
    absl::flat_hash_set<std::string> joint_groups_with_command;
    absl::flat_hash_set<std::string>& joint_groups_with_reset_commands =
        gazebo_hwm_.value().joint_groups_with_reset_commands;
    joint_groups_with_reset_commands.clear();
    for (auto& [interface_name, interface_data] :
         gazebo_hwm_->hardware_interface_name_to_gazebo_data) {
      INTR_RETURN_IF_ERROR(
          std::visit(
              ApplyInterfaceDataToEcmVisitor{
                  .joint_groups_with_command = joint_groups_with_command,
                  .joint_groups_with_reset_commands =
                      joint_groups_with_reset_commands,
                  .joint_groups_by_name = gazebo_hwm_->joint_groups_by_name,
                  .hardware_interface_handles = interfaces.handles,
                  .info = info,
                  .command_validator = interfaces.command_validator,
                  .ecm = ecm,
                  .interface_name = interface_name,
                  .use_sim_velocity_control = use_sim_velocity_control_,
              },
              interface_data))
          .LogError()
          .With(ReturnVoid());
    }
    for (const auto& [joint_group_name, joints] :
         gazebo_hwm_->joint_groups_by_name) {
      if (joint_groups_with_command.contains(joint_group_name)) {
        continue;
      }
      LOG_EVERY_N_SEC(INFO, 5)
          << "Joint group '" << joint_group_name
          << "' did not receive a fresh command, stopping motion";
      auto gravity_compensators =
          gazebo_hwm_.value()
              .gravity_compensators_for_braking_by_joint_group.find(
                  joint_group_name);
      if (gravity_compensators ==
          gazebo_hwm_.value()
              .gravity_compensators_for_braking_by_joint_group.end()) {
        LOG_EVERY_N_SEC(ERROR, 10)
            << "There are no gravity compensators for joint group '"
            << joint_group_name << "'";
      }
      INTR_RETURN_IF_ERROR(ApplyBrakes(use_sim_velocity_control_, ecm, joints,
                                       gravity_compensators->second))
          .LogError()
          .With(ReturnVoid());
    }
  }
}

void HardwareModuleLauncher::UpdateHwmSensorValues(
    const ::gz::sim::UpdateInfo& info,
    const ::gz::sim::EntityComponentManager& ecm,
    GazeboHardwareModule::HardwareInterfaces& interfaces) {
  GZ_PROFILE("HardwareModuleLauncher::UpdateHwmSensorValues");
  intrinsic::Time now = intrinsic::Clock::Zero() + info.simTime;
  if (gazebo_hwm_.ok()) {
    // Copy rangefinder_data_by_entity_ to local variable to avoid holding mutex
    // while updating the ECM.
    absl::flat_hash_map<gz::sim::Entity, gz::msgs::LaserScan>
        rangefinder_data_by_entity_copy;
    {
      absl::MutexLock lock(rangefinder_data_mtx_);
      rangefinder_data_by_entity_copy = rangefinder_data_by_entity_;
    }
    for (auto& [interface_name, interface_data] :
         gazebo_hwm_->hardware_interface_name_to_gazebo_data) {
      INTR_RETURN_IF_ERROR(
          std::visit(
              UpdateInterfaceWithEcmDataVisitor{
                  .now = now,
                  .joint_groups_with_reset_commands =
                      gazebo_hwm_->joint_groups_with_reset_commands,
                  .joint_groups_by_name = gazebo_hwm_->joint_groups_by_name,
                  .hardware_interface_handles = interfaces.handles,
                  .ecm = ecm,
                  .rangefinder_data_by_entity = rangefinder_data_by_entity_copy,
                  .interface_name = interface_name,
                  .use_sim_velocity_control = use_sim_velocity_control_,
              },
              interface_data))
          .LogError()
          .With(ReturnVoid());
    }
  }
}

void HardwareModuleLauncher::OnStop() {
  // Stop the thread that observes restart requests.
  restart_request_observer_thread_ = {};
  if (gazebo_hwm_.ok()) {
    LOG(INFO) << "[" << gazebo_hwm_->gazebo_hwm_rawptr->Name()
              << "] Stopping HWM runtime";
    // Shutdown grpc server before stopping runtime as this is promised to the
    // HWM.
    gazebo_hwm_->grpc_server.reset();
    if (absl::Status stop_status = gazebo_hwm_->runtime->Stop();
        !stop_status.ok()) {
      LOG(ERROR) << "[" << gazebo_hwm_->gazebo_hwm_rawptr->Name()
                 << "] Failed to stop Hardware Module: " << stop_status;
    }
  }
}

}  // namespace intrinsic::simulation
