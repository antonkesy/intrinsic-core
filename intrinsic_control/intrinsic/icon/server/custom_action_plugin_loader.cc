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

#include "intrinsic/icon/server/custom_action_plugin_loader.h"

#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/control/c_api/c_plugin_api.h"
#include "intrinsic/icon/control/c_api/convert_c_realtime_status.h"
#include "intrinsic/icon/plugin_manager/plugin_loader.h"
#include "intrinsic/util/status/status_macros.h"
#include "ortools/base/filesystem.h"
#include "ortools/base/options.h"
#include "ortools/base/path.h"

namespace intrinsic::icon {

// Stringified name of the entrypoint function, for dynamic loading purposes.
#define STRINGIFY(s) STRINGIFY_IMPL(s)
#define STRINGIFY_IMPL(s) #s
#define INTRINSIC_ICON_ACTION_PLUGIN_ENTRY_POINT_STRINGIFIED \
  STRINGIFY(INTRINSIC_ICON_ACTION_PLUGIN_ENTRY_POINT)

absl::StatusOr<CustomActionPluginLoader> CustomActionPluginLoader::LoadPlugins(
    absl::string_view enabled_plugins_path,
    IntrinsicIconRegisterActionType register_function) {
  std::vector<std::string> files;
  absl::flat_hash_map<std::string, PluginLoader> plugins_by_basename;

  INTR_RETURN_IF_ERROR(
      file::IsDirectory(enabled_plugins_path, file::Defaults()));
  INTR_RETURN_IF_ERROR(file::Match(
      file::JoinPath(enabled_plugins_path, absl::StrCat("*", kPluginSuffix)),
      &files, file::Defaults()));

  for (const std::string& file : files) {
    LOG(INFO) << "PUBLIC: Attempt to load plugin file: " << file;

    // Dynamically load the plugin file.
    INTR_ASSIGN_OR_RETURN(PluginLoader plugin, PluginLoader::Create(file));

    // Lookup the entrypoint method.
    INTR_ASSIGN_OR_RETURN(
        auto entrypoint,
        plugin.GetSymbol<IntrinsicIconRegisterActionTypes>(
            INTRINSIC_ICON_ACTION_PLUGIN_ENTRY_POINT_STRINGIFIED),
        _ << "while loading plugin " << file);

    // Invoke the entrypoint to register custom actions.
    // IntrinsicIconRealtimeStatus status = entrypoint(register_function);
    INTR_RETURN_IF_ERROR(ToAbslStatus(entrypoint(register_function)))
        << "; returned from entrypoint "
        << INTRINSIC_ICON_ACTION_PLUGIN_ENTRY_POINT_STRINGIFIED
        << " in plugin file " << file;

    // Store the PluginLoader, keyed by file basename. We need to hold onto the
    // PluginLoader because it calls dlclose in its destructor.
    plugins_by_basename.emplace(file::Basename(file), std::move(plugin));
    LOG(INFO) << "PUBLIC: Loaded plugin " << file;
  }

  return CustomActionPluginLoader(std::move(plugins_by_basename));
}

std::vector<std::string> CustomActionPluginLoader::ListPluginBasenames() {
  std::vector<std::string> out;
  for (const auto& [key, _] : plugins_by_basename_) {
    out.emplace_back(key);
  }
  return out;
}

}  // namespace intrinsic::icon
