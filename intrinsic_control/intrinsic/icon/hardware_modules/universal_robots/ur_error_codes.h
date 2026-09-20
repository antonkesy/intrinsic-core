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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_UR_ERROR_CODES_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_UR_ERROR_CODES_H_

#include <cstddef>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"

namespace intrinsic::icon {

// Singleton map of error code number to human-readable string representation.
// Taken from
// https://www.universal-robots.com/manuals/EN/HTML/SW5_21/Content/Landingpages/Web/ErrorCodes.htm.
// The returned map will be created on first call. It will never be deleted or
// recreated.
const absl::flat_hash_map<size_t, absl::string_view>&
UrErrorCodeToStringMapSingleton();

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_UR_ERROR_CODES_H_
