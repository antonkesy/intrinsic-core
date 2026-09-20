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

#include "intrinsic/simulation/gazebo/plugins/link_device_util.h"

#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/components/DetachableJoint.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "intrinsic/simulation/gazebo/plugins/util.h"
#include "intrinsic/util/status/status_builder.h"

namespace intrinsic {
namespace simulation {

using ::gz::sim::kNullEntity;
using ::gz::sim::components::Link;
using ::gz::sim::components::Model;

using DetachableJointComponent = ::gz::sim::components::DetachableJoint;
using GazeboECM = ::gz::sim::EntityComponentManager;
using GazeboEntity = ::gz::sim::Entity;

absl::StatusOr<GazeboEntity> GetLinkEntityFromRelativeScopedName(
    absl::string_view relative_scoped_link_name,
    absl::string_view scope_separator, GazeboEntity parent_entity,
    const GazeboECM& ecm) {
  std::vector<std::string> parts =
      absl::StrSplit(relative_scoped_link_name, scope_separator);
  if (parts.size() > 1) {
    for (auto& part : parts) {
      auto child_model_entity = GetChildModelByName(part, parent_entity, ecm);
      if (!child_model_entity.ok()) {
        break;
      }
      parent_entity = *child_model_entity;
      relative_scoped_link_name = relative_scoped_link_name.substr(
          part.size() + scope_separator.size());
    }
  }

  auto model = ::gz::sim::Model(parent_entity);
  if (!model.Valid(ecm)) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Parent entity " << parent_entity << " is not a model";
  }

  auto link_entity =
      model.LinkByName(ecm, std::string(relative_scoped_link_name));
  if (link_entity == kNullEntity) {
    return intrinsic::NotFoundErrorBuilder()
           << "Unable to find link named " << relative_scoped_link_name;
  }

  return link_entity;
}

absl::StatusOr<GazeboEntity> AddDetachableJoint(GazeboEntity parent_link,
                                                GazeboEntity child_link,
                                                GazeboECM& ecm) {
  if (parent_link == kNullEntity) {
    return absl::InvalidArgumentError("Parent link is kNullEntity");
  }
  if (child_link == kNullEntity) {
    return absl::InvalidArgumentError("Child link is kNullEntity");
  }

  auto joint_entity = ecm.CreateEntity();
  if (joint_entity == kNullEntity) {
    return absl::InternalError("Failed to create entity");
  }

  ::gz::sim::components::DetachableJointInfo info(
      {.parentLink = parent_link, .childLink = child_link});

  auto* component =
      ecm.CreateComponent(joint_entity, DetachableJointComponent(info));
  if (component == nullptr) {
    return absl::InternalError("Failed to create component");
  }

  return joint_entity;
}

absl::Status RemoveDetachableJoint(GazeboEntity joint, GazeboECM& ecm) {
  if (joint == kNullEntity) {
    return absl::InvalidArgumentError("Joint is kNullEntity");
  }
  ecm.RequestRemoveEntity(joint);
  return absl::OkStatus();
}

}  // namespace simulation
}  // namespace intrinsic
