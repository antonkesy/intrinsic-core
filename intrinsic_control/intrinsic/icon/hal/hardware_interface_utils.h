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

#ifndef INTRINSIC_ICON_HAL_HARDWARE_INTERFACE_UTILS_H_
#define INTRINSIC_ICON_HAL_HARDWARE_INTERFACE_UTILS_H_

#include <stddef.h>

#include <string>
#include <string_view>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"

namespace intrinsic::icon::hal {

// Returns a list of all modules that are available in `memory_namespace`.
absl::StatusOr<std::vector<std::string>> FindModuleNames(
    std::string_view memory_namespace);

// Returns a map of available modules, their interfaces and corresponding type,
// under the `memory_namespace`. The map is structured as follows:
//
//   module_name -> interface_name -> interface_type
//
// Only returns modules for which there is a module info interface.
absl::StatusOr<absl::flat_hash_map<
    std::string, absl::flat_hash_map<std::string, std::string>>>
FindModulesAndInterfacesWithTypes(std::string_view memory_namespace = "");

}  // namespace intrinsic::icon::hal
#endif  // INTRINSIC_ICON_HAL_HARDWARE_INTERFACE_UTILS_H_
