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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_WORLD_MODEL_CONFIG_PLUGIN_CONSTANTS_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_WORLD_MODEL_CONFIG_PLUGIN_CONSTANTS_H_

#include "absl/strings/string_view.h"

namespace intrinsic {
namespace simulation {
// Constants associated with the `WorldModelConfigPlugin`.
// See
// http://intrinsic/simulation/gazebo/plugins/world_model_config_plugin.h
// for details on how the constants are used in the plugin xml.
// TODO(b/352041315): Merge back into the plugin header once the plugin is
// externalized.
constexpr absl::string_view kWorldModelConfigPlugin_WorldIdTag =
    "world_entity_id";

constexpr absl::string_view kWorldModelConfigPlugin_WorldObjectResourceIdTag =
    "world_object_resource_id";

constexpr absl::string_view kWorldModelConfigPlugin_WorldObjectNameTag =
    "world_object_name";

constexpr absl::string_view kWorldModelConfigPlugin_ResourceNameTag =
    "resource_name";

constexpr absl::string_view kWorldModelConfigPlugin_LinkDataTag = "link_data";

constexpr absl::string_view kWorldModelConfigPlugin_ModelDataTag = "model_data";

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_WORLD_MODEL_CONFIG_PLUGIN_CONSTANTS_H_
