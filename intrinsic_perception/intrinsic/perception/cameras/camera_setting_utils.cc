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

#include "intrinsic/perception/cameras/camera_setting_utils.h"

#include <string>
#include <string_view>
#include <variant>

#include "absl/status/statusor.h"
#include "google/protobuf/empty.pb.h"
#include "intrinsic/perception/cameras/camera_setting.h"
#include "intrinsic/perception/cameras/camera_setting_access.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::perception {

CameraSetting CreateEnumCameraSetting(std::string_view name,
                                      std::string_view enum_value) {
  return {
      .name = std::string(name),
      .value = CameraSetting::Enumeration{.value = std::string(enum_value)}};
}

CameraSetting CreateCommandCameraSetting(std::string_view name) {
  return {.name = std::string(name), .value = CameraSetting::Command()};
}

CameraSettingAccess CreateCameraSettingAccess(std::string_view name,
                                              CameraSettingAccess::Mode mode) {
  return {.name = std::string(name), .mode = mode};
}

absl::StatusOr<std::string> GetEnumerationValue(
    const CameraSetting& camera_setting) {
  if (const CameraSetting::Enumeration* val =
          std::get_if<CameraSetting::Enumeration>(&camera_setting.value)) {
    return val->value;
  }
  return intrinsic::InvalidArgumentErrorBuilder()
         << "`" << camera_setting.name
         << "` camera setting does not contain an enumeration value.";
}

}  // namespace intrinsic::perception
