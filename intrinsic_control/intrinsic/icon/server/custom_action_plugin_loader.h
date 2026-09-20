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

#ifndef INTRINSIC_ICON_SERVER_CUSTOM_ACTION_PLUGIN_LOADER_H_
#define INTRINSIC_ICON_SERVER_CUSTOM_ACTION_PLUGIN_LOADER_H_

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/control/c_api/c_plugin_api.h"
#include "intrinsic/icon/plugin_manager/plugin_loader.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

// Loads custom action plugins from a directory. Unloads the action plugins on
// destruction.
//
// Example usage:
//
// ```
//   // Load and register custom actions.
//   INTR_ASSIGN_OR_RETURN(auto action_plugins,
//                    CustomActionPluginLoader::LoadPlugins("enabled/"));
//   ...
//   // When action_plugins goes out of scope, plugins will be unloaded.
// ```
class CustomActionPluginLoader final {
 public:
  static constexpr char kPluginSuffix[] = ".so";

  // Constructs an empty CustomActionPluginLoader that does not load any
  // plugins.
  CustomActionPluginLoader() = default;

  // Reads each file in directory `enabled_plugins_path`. Any file that has
  // suffix kPluginSuffix is loaded. Invokes the method
  // IntrinsicIcon_ACTION_PLUGIN_ENTRY_POINT for each plugin (see
  // intrinsic/icon/control/c_api/c_plugin_api.h). Returns the first
  // error encountered.
  static absl::StatusOr<CustomActionPluginLoader> LoadPlugins(
      absl::string_view enabled_plugins_path,
      IntrinsicIconRegisterActionType register_function);

  // List the file basename of each loaded plugin.
  std::vector<std::string> ListPluginBasenames();

  // This class is move-only.
  CustomActionPluginLoader(CustomActionPluginLoader&& other) = default;
  CustomActionPluginLoader& operator=(CustomActionPluginLoader&& other) =
      default;

 private:
  // Private constructor used by LoadPlugins factory.
  explicit CustomActionPluginLoader(
      absl::flat_hash_map<std::string, PluginLoader>&& plugins_by_basename)
      : plugins_by_basename_(std::move(plugins_by_basename)) {}

  // Each PluginLoader dlopens/dlcloses a single file, so we need one per plugin
  // file.
  absl::flat_hash_map<std::string, PluginLoader> plugins_by_basename_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_SERVER_CUSTOM_ACTION_PLUGIN_LOADER_H_
