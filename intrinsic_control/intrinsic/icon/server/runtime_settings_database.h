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

#ifndef INTRINSIC_ICON_SERVER_RUNTIME_SETTINGS_DATABASE_H_
#define INTRINSIC_ICON_SERVER_RUNTIME_SETTINGS_DATABASE_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"

namespace intrinsic::icon {

// Runtime settings to be persisted over process restarts.
// These default values will be used if no settings were saved.
struct RuntimeSettings {
  bool operator==(const RuntimeSettings&) const = default;

  double speed_override = 1.0f;
};

// Loads runtime settings from `path`.
// If a field is not set, the default value is returned.
absl::StatusOr<RuntimeSettings> LoadRuntimeSettings(absl::string_view path);

// Saves runtime settings to `path`.
absl::Status SaveRuntimeSettings(absl::string_view path,
                                 const RuntimeSettings& settings);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_SERVER_RUNTIME_SETTINGS_DATABASE_H_
