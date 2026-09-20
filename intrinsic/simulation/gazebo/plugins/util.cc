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

#include "intrinsic/simulation/gazebo/plugins/util.h"

#include "absl/log/check.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "intrinsic/util/status/status_builder.h"

namespace intrinsic {
namespace simulation {

using ::gz::sim::components::Model;
using ::gz::sim::components::Name;

absl::StatusOr<::gz::sim::Entity> GetChildModelByName(
    absl::string_view child_name, ::gz::sim::Entity parent_entity,
    const ::gz::sim::EntityComponentManager& ecm) {
  auto children_entities =
      ecm.ChildrenByComponents(parent_entity, ::gz::sim::components::Model());

  for (const auto& child_model_entity : children_entities) {
    const auto* name_comp = ecm.Component<Name>(child_model_entity);
    CHECK_NE(name_comp, nullptr)
        << "Found nested model with no name " << "component for parent entity "
        << parent_entity;
    if (name_comp->Data() == child_name) {
      return child_model_entity;
      break;
    }
  }

  return intrinsic::NotFoundErrorBuilder()
         << "Unable to find child model named " << child_name;
}

}  // namespace simulation
}  // namespace intrinsic
