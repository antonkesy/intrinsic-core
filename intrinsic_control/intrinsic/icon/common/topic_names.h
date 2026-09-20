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

#ifndef INTRINSIC_ICON_COMMON_TOPIC_NAMES_H_
#define INTRINSIC_ICON_COMMON_TOPIC_NAMES_H_

#include <string>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/common/id_types.h"

namespace intrinsic::icon {

// Returns a topic name for pubsub of robot status at the
// full controller rate (typically >= 500 Hz).
// Returns an InvalidArgumentError if `robot_name` is empty.
absl::StatusOr<std::string> TopicNameForRobotStatus(
    absl::string_view robot_name);

// Returns a topic name for pubsub of robot status that is
// always throttled to 50 Hz.
// Returns an InvalidArgumentError if `robot_name` is empty.
absl::StatusOr<std::string> TopicNameForRobotStatusThrottle(
    absl::string_view robot_name);

// Returns a topic for pubsub publishing.
// Returns an InvalidArgumentError if `robot_name` is empty.
absl::StatusOr<std::string> TopicNameForCoeExport(absl::string_view robot_name);

// Returns a topic for pubsub publishing of diagnosis messages.
// Returns an InvalidArgumentError if `robot_name` is empty.
absl::StatusOr<std::string> TopicNameForDiagnosisExport(
    absl::string_view robot_name);

// Returns a topic for pubsub publishing of streaming action outputs.
// Returns an InvalidArgumentError if `robot_name` is empty.
absl::StatusOr<std::string> TopicNameForStreamingOutput(
    absl::string_view robot_name, ActionInstanceId action_id);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_COMMON_TOPIC_NAMES_H_
