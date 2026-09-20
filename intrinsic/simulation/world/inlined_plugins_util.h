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

#ifndef INTRINSIC_SIMULATION_WORLD_INLINED_PLUGINS_UTIL_H_
#define INTRINSIC_SIMULATION_WORLD_INLINED_PLUGINS_UTIL_H_

#include <optional>
#include <string>

#include "absl/status/statusor.h"
#include "intrinsic/icon/hal/proto/v1/digital_input_output.pb.h"
#include "intrinsic/simulation/world/multi_camera_plugin_spec.h"
#include "intrinsic/world/entity.h"

namespace intrinsic {
namespace simulation {

// Struct containing parsed inlined plugin information extracted from an
// entity's UserDataComponent (specifically sdf::kGazeboPlugins).
struct InlinedPluginsInfo {
  // Inlined plugin strings which were not parsed. Does not include strings for
  // plugins that were parsed out (e.g. DIOs or MultiCameraPlugin).
  std::string unparsed_inlined_plugins;

  // Parsed DIO plugin info.
  std::optional<intrinsic_proto::icon::v1::DigitalInputOutput> dios_proto;

  // Parsed MultiCameraPlugin spec.
  std::optional<MultiCameraPluginSpec> multi_camera_plugin_spec;
};

// Extracts and parses inlined gazebo plugins from the entity's
// UserDataComponent under key `sdf::kGazeboPlugins`. Parses known plugins (like
// DIO and MultiCameraPlugin) and removes them from `unparsed_inlined_plugins`.
absl::StatusOr<InlinedPluginsInfo> ParseInlinedPlugins(
    const WorldEntity* entity);

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_WORLD_INLINED_PLUGINS_UTIL_H_
