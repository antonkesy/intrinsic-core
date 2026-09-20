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

#include "intrinsic/icon/common/topic_names.h"

#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

namespace {

absl::Status ValidateRobotName(absl::string_view robot_name) {
  if (robot_name.empty()) {
    return absl::InvalidArgumentError("Robot name must not be empty!");
  }
  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<std::string> TopicNameForRobotStatus(
    absl::string_view robot_name) {
  INTR_RETURN_IF_ERROR(ValidateRobotName(robot_name));
  return absl::StrCat("/icon/", robot_name, "/robot_status");
}

absl::StatusOr<std::string> TopicNameForRobotStatusThrottle(
    absl::string_view robot_name) {
  INTR_RETURN_IF_ERROR(ValidateRobotName(robot_name));
  return absl::StrCat("/icon/", robot_name, "/robot_status_throttle");
}

absl::StatusOr<std::string> TopicNameForCoeExport(
    absl::string_view robot_name) {
  INTR_RETURN_IF_ERROR(ValidateRobotName(robot_name));
  return absl::StrCat("/icon/", robot_name, "/coe_export");
}

absl::StatusOr<std::string> TopicNameForDiagnosisExport(
    absl::string_view robot_name) {
  INTR_RETURN_IF_ERROR(ValidateRobotName(robot_name));
  return absl::StrCat("/icon/", robot_name, "/diagnosis");
}

absl::StatusOr<std::string> TopicNameForStreamingOutput(
    absl::string_view robot_name, ActionInstanceId action_id) {
  INTR_RETURN_IF_ERROR(ValidateRobotName(robot_name));

  return absl::StrCat("/icon/", robot_name, "/output_streams/action_",
                      action_id.value());
}

}  // namespace intrinsic::icon
