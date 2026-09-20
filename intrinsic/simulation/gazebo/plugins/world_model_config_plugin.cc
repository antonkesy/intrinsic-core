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

#include "intrinsic/simulation/gazebo/plugins/world_model_config_plugin.h"

#include <memory>
#include <string>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/World.hh"
#include "intrinsic/simulation/gazebo/components/ppr_component.h"
#include "intrinsic/simulation/gazebo/components/world_component.h"
#include "intrinsic/simulation/gazebo/plugins/world_model_config_plugin_constants.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "sdf/Element.hh"

namespace intrinsic {
namespace simulation {

namespace {

template <typename ComponentTy>
absl::Status AddComponentData(const gz::sim::Entity& entity,
                              gz::sim::EntityComponentManager& ecm,
                              sdf::ElementConstPtr sdf,
                              absl::string_view data_tag,
                              typename ComponentTy::Type default_data) {
  using ComponentDataTy = typename ComponentTy::Type;
  ComponentDataTy component_data;
  bool component_data_found = false;
  std::tie(component_data, component_data_found) =
      sdf->Get<ComponentDataTy>(std::string(data_tag), default_data);
  if (component_data_found) {
    if (ecm.EntityHasComponentType(entity, ComponentTy::typeId)) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << "Entity " << entity
             << " already has component: " << ComponentTy::typeName;
    } else {
      ComponentTy* comp =
          ecm.CreateComponent<ComponentTy>(entity, ComponentTy(component_data));
      if (nullptr == comp) {
        return intrinsic::InternalErrorBuilder()
               << "Unable to add component to entity: "
               << ComponentTy::typeName;
      }
    }
  } else {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "<" << data_tag << "> not set";
  }

  return absl::OkStatus();
}

absl::Status ConfigureLinkData(sdf::ElementConstPtr link_data_elem,
                               const gz::sim::Entity& model_entity,
                               gz::sim::EntityComponentManager& ecm) {
  if (!link_data_elem->HasAttribute("name")) {
    return absl::InvalidArgumentError(
        "<link_data> tag is missing 'name' attribute. Skipping.");
  }

  std::string link_name = link_data_elem->GetAttribute("name")->GetAsString();

  // Find the link based on its name in this model.
  gz::sim::Entity link_entity = gz::sim::kNullEntity;
  std::vector<gz::sim::Entity> possible_entities =
      ecm.EntitiesByComponents(gz::sim::components::ParentEntity(model_entity),
                               gz::sim::components::Name(link_name));
  for (const auto& e : possible_entities) {
    if (ecm.EntityHasComponentType(e, gz::sim::components::Link::typeId)) {
      if (link_entity != gz::sim::kNullEntity) {
        LOG(WARNING) << "Found multiple links in model with name '" << link_name
                     << "'. Only using first link.";
        break;
      }

      link_entity = e;
    }
  }

  if (link_entity == gz::sim::kNullEntity) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Could not find link in model with name: " << link_name;
  }

  INTR_RETURN_IF_ERROR(AddComponentData<WorldEntityId>(
      link_entity, ecm, link_data_elem, kWorldModelConfigPlugin_WorldIdTag,
      kRootEntityId.value()));

  return absl::OkStatus();
}

absl::Status ConfigureModelData(sdf::ElementConstPtr model_data_elem,
                                const gz::sim::Entity& entity,
                                gz::sim::EntityComponentManager& ecm) {
  INTR_RETURN_IF_ERROR(AddComponentData<WorldEntityId>(
      entity, ecm, model_data_elem, kWorldModelConfigPlugin_WorldIdTag,
      kRootEntityId.value()));

  if (model_data_elem->HasAttribute(
          std::string(kWorldModelConfigPlugin_WorldObjectNameTag))) {
    INTR_RETURN_IF_ERROR(AddComponentData<WorldObjectName>(
        entity, ecm, model_data_elem,
        kWorldModelConfigPlugin_WorldObjectNameTag, RootObjectName().value()));
  }

  bool has_resource_name = model_data_elem->HasAttribute(
      std::string(kWorldModelConfigPlugin_ResourceNameTag));
  if (has_resource_name) {
    LOG(INFO) << "Found resource name in model: "
              << model_data_elem
                     ->GetAttribute(
                         std::string(kWorldModelConfigPlugin_ResourceNameTag))
                     ->GetAsString();
  }
  if (has_resource_name) {
    INTR_RETURN_IF_ERROR(AddComponentData<ResourceName>(
        entity, ecm, model_data_elem, kWorldModelConfigPlugin_ResourceNameTag,
        ""));
  }

  return AddComponentData<WorldObjectResourceId>(
      entity, ecm, model_data_elem,
      kWorldModelConfigPlugin_WorldObjectResourceIdTag, RootObjectId().value());
}

}  // namespace

void WorldModelConfigPlugin::Configure(
    const gz::sim::Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& event_mgr) {
  // Go through each link info and read out the information
  for (sdf::ElementConstPtr data_elem = sdf->GetFirstElement();
       data_elem != nullptr; data_elem = data_elem->GetNextElement()) {
    if (data_elem->GetName() != kWorldModelConfigPlugin_LinkDataTag &&
        data_elem->GetName() != kWorldModelConfigPlugin_ModelDataTag) {
      continue;
    }

    if (data_elem->GetName() == kWorldModelConfigPlugin_ModelDataTag) {
      absl::Status model_data_status =
          ConfigureModelData(data_elem, entity, ecm);
      LOG_IF(ERROR, !model_data_status.ok())
          << "Error configuring model data: " << model_data_status.message();
    } else {
      absl::Status link_data_status = ConfigureLinkData(data_elem, entity, ecm);
      LOG_IF(ERROR, !link_data_status.ok())
          << "Error configuring link data: " << link_data_status.message();
    }
  }

  // If we've added world data, then we need to set the world dirty flag.
  gz::sim::Entity world_entity =
      ecm.EntityByComponents(gz::sim::components::World());
  CHECK_NE(world_entity, gz::sim::kNullEntity);
  auto world_dirty_flag =
      ecm.ComponentDefault<WorldDirtyFlag>(world_entity, true);
  if (world_dirty_flag == nullptr) {
    LOG(ERROR) << "Could not add dirty flag";
  } else {
    world_dirty_flag->Data() = true;
  }
}

}  // namespace simulation
}  // namespace intrinsic
