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

#ifndef INTRINSIC_ICON_SERVER_RUNTIME_OPTIONS_H_
#define INTRINSIC_ICON_SERVER_RUNTIME_OPTIONS_H_

#include <string>

namespace intrinsic {
namespace icon {

// Enum listing available runtime modes.
// TODO(b/197758673) Consider removing this and instead generating distinct
// configurations for --non_realtime and realtime modes.
enum class ServerRuntimeMode {
  // The server is running in a normal environment.
  kNormal,
  // The server is running in a non-real-time environment. This includes
  // simulation and testing.
  kNonRealtime,
};

// This holds runtime options that can affect all of ICON MainLoop.
//
// Prefer more specific places for configuration, for example part of action
// config, if possible.
struct ServerRuntimeOptions {
  // Server name, aka "robot name".
  std::string server_name;
  // If running as a resource instance, this the resource ID. Empty otherwise.
  std::string resource_id;
  // Runtime mode.
  ServerRuntimeMode mode = ServerRuntimeMode::kNormal;
  // Plugin directory where custom actions are loaded from.
  std::string plugin_dir;
  // Directory where runtime settings (i.e. speed override) are saved/loaded.
  std::string runtime_settings_dir = "/tmp/intrinsic_icon";
};

}  // namespace icon
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_SERVER_RUNTIME_OPTIONS_H_
