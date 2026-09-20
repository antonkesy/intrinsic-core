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

#include "intrinsic/simulation/gazebo/plugins/add_hardware_module_launchers_system.h"

#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_set.h"
#include "absl/flags/flag.h"
#include "absl/functional/function_ref.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/System.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Static.hh"
#include "intrinsic/icon/hal/proto/hardware_module_config.pb.h"
#include "intrinsic/simulation/gazebo/asset_instances_client.h"
#include "intrinsic/simulation/gazebo/asset_instances_service_address_flag.h"
#include "intrinsic/simulation/gazebo/components/ppr_component.h"
#include "intrinsic/simulation/gazebo/plugins/hardware_module_launcher.h"
#include "intrinsic/util/status/status_macros.h"
#include "sdf/Element.hh"

namespace intrinsic::simulation {
namespace {

// Returns the names of all known resources that are hardware modules, as a set
// for more efficient `contains()` checks.
absl::StatusOr<absl::flat_hash_set<std::string>> FindAllHwmResources() {
  const std::string address =
      absl::GetFlag(FLAGS_asset_instances_service_address);
  if (address.empty()) {
    return absl::flat_hash_set<std::string>{};
  }

  INTR_ASSIGN_OR_RETURN(auto client, AssetInstancesClient::Create(address));
  // We query all asset instances that are either hardware devices or service
  // assets, since hardware modules can be registered under either type.
  //
  // Because we cannot restrict the query strictly to hardware modules at the
  // service filter level, we inspect all returned configurations and find those
  // that contain a `HardwareModuleConfig` payload.
  INTR_ASSIGN_OR_RETURN(auto instances,
                        client->ListHardwareDevicesAndServices());

  // No `reserve()` call, because we don't expect a large percentage of
  // resources to be hardware modules.
  absl::flat_hash_set<std::string> resource_names;
  for (const auto& instance : instances) {
    INTR_ASSIGN_OR_RETURN(google::protobuf::Any config,
                          AssetInstancesClient::ExtractServiceConfig(instance));

    if (config.Is<intrinsic_proto::icon::HardwareModuleConfig>()) {
      resource_names.insert(instance.name());
    }
  }
  return resource_names;
}

absl::StatusOr<std::vector<gz::sim::Entity>> FindHwmEntities(
    const absl::flat_hash_set<std::string>& hardware_module_resource_names,
    ::gz::sim::EntityComponentManager& ecm) {
  std::vector<gz::sim::Entity> entities;
  ecm.Each<intrinsic::simulation::ResourceName>(
      [&](const gz::sim::Entity& entity,
          const intrinsic::simulation::ResourceName* resource_name) {
        if (!hardware_module_resource_names.contains(resource_name->Data())) {
          LOG(INFO) << "The resource config proto for '"
                    << resource_name->Data()
                    << "' is not a HardwareModuleConfig, will not add a "
                       "HardwareModuleLauncher system to it";
          return true;
        }
        entities.push_back(entity);
        return true;
      });
  return entities;
}

// Searches `ecm` for entities that have a `ResourceName` Component with each of
// `hardware_module_resource_names`, and creates new entities for those names
// where there isn't an entity yet.
//
// Returns a list of the newly-created entities.
// Returns InternalError if anything goes wrong adding those new entities.
absl::StatusOr<std::vector<gz::sim::Entity>>
CreateEntitiesForHwmsWithoutWorldModel(
    const absl::flat_hash_set<std::string>& hardware_module_resource_names,
    ::gz::sim::EntityComponentManager& ecm) {
  std::vector<gz::sim::Entity> entities;
  for (const auto& hwm_name : hardware_module_resource_names) {
    if (ecm.EntityByComponents(ResourceName{hwm_name}) !=
        ::gz::sim::kNullEntity) {
      // There's already an entity for this HWM, nothing to do
      continue;
    }
    LOG(INFO) << "Creating ECM Model for HWM resource '" << hwm_name << "'";
    // There isn't a model entity for this HWM yet, create one.
    // The entity needs to be a Model because we later want to attach a
    // HardwareModuleLauncher system to it, and needs to be static (because it
    // doesn't have any links, and Gazebo only allows static Models to have no
    // links).
    ::gz::sim::Entity hwm_entity = ecm.CreateEntity();
    if (hwm_entity == ::gz::sim::kNullEntity) {
      return absl::InternalError(absl::StrCat(
          "Failed to create entity for HWM resource '", hwm_name, "'"));
    }
    if (!ecm.SetComponentData<::gz::sim::components::Name>(hwm_entity,
                                                           hwm_name)) {
      return absl::InternalError(absl::StrCat(
          "Failed to set Name on entity for HWM resource '", hwm_name, "'"));
    }
    if (!ecm.SetComponentData<ResourceName>(hwm_entity, hwm_name)) {
      return absl::InternalError(absl::StrCat(
          "Failed to set ResourceName on entity for HWM resource '", hwm_name,
          "'"));
    }
    if (!ecm.CreateComponent(hwm_entity, ::gz::sim::components::Model{})) {
      return absl::InternalError(
          absl::StrCat("Failed to mark entity for HWM resource '", hwm_name,
                       "' as a Model"));
    }
    if (!ecm.SetComponentData<::gz::sim::components::Static>(hwm_entity,
                                                             true)) {
      return absl::InternalError(absl::StrCat(
          "Failed to mark entity for HWM resource '", hwm_name, "' as static"));
    }
    entities.push_back(hwm_entity);
  }
  return entities;
}

}  // namespace

void AddHardwareModuleLaunchersSystem::Configure(
    const ::gz::sim::Entity& /*unused*/,
    const std::shared_ptr<const ::sdf::Element>& sdf,
    ::gz::sim::EntityComponentManager& ecm,
    ::gz::sim::EventManager& event_manager) {
  absl::MutexLock l(m_);
  // Throughout this function, we save any errors in
  // hardware_module_model_entities_. That way, the
  // `AddHardwareModuleLaunchers()` will report them.
  hardware_module_resource_names_ = FindAllHwmResources();
  if (!hardware_module_resource_names_.ok()) {
    hardware_module_model_entities_ = hardware_module_resource_names_.status();
    return;
  }
  hardware_module_model_entities_ =
      FindHwmEntities(*hardware_module_resource_names_, ecm);
  if (!hardware_module_model_entities_.ok()) {
    return;
  }
  auto new_hardware_module_model_entities =
      CreateEntitiesForHwmsWithoutWorldModel(*hardware_module_resource_names_,
                                             ecm);
  if (!new_hardware_module_model_entities.ok()) {
    hardware_module_model_entities_ =
        new_hardware_module_model_entities.status();
    return;
  }
  absl::c_move(std::move(*new_hardware_module_model_entities),
               std::back_inserter(*hardware_module_model_entities_));
}

absl::Status AddHardwareModuleLaunchersSystem::AddHardwareModuleLaunchers(
    absl::FunctionRef<absl::Status(std::shared_ptr<::gz::sim::System> system,
                                   ::gz::sim::Entity entity)>
        add_system_fn) const {
  absl::MutexLock l(m_);
  INTR_RETURN_IF_ERROR(hardware_module_model_entities_.status());
  for (const gz::sim::Entity& entity : *hardware_module_model_entities_) {
    INTR_RETURN_IF_ERROR(
        add_system_fn(std::make_unique<HardwareModuleLauncher>(), entity));
  }
  return absl::OkStatus();
}

}  // namespace intrinsic::simulation
