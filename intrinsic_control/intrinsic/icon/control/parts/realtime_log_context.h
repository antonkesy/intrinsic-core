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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_REALTIME_LOG_CONTEXT_H_
#define INTRINSIC_ICON_CONTROL_PARTS_REALTIME_LOG_CONTEXT_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <ostream>

#include "absl/status/statusor.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/logging/proto/context.pb.h"

namespace intrinsic::icon {

inline constexpr std::size_t kDeploymentIdSize = 40;

// RealtimeLogContext is the realtime version of
// intrinsic_proto::data_logger::Context in
// "intrinsic/logging/proto/context.proto". All fields are the same.
struct RealtimeLogContext {
  icon::FixedString<kDeploymentIdSize> deployment_id;
  uint64_t executive_session_id;
  uint64_t executive_plan_id;
  uint64_t executive_plan_action_id;
  uint64_t skill_id;
  uint64_t parent_skill_id;
  std::optional<uint64_t> icon_session_id;
  std::optional<uint64_t> icon_action_id;

  bool operator==(const RealtimeLogContext& other) const;
  bool operator!=(const RealtimeLogContext& other) const;
};

// Converts `realtime_log_context` into a proto.
//
// Not realtime safe.
intrinsic_proto::data_logger::Context ToProto(
    const RealtimeLogContext& realtime_log_context) INTRINSIC_NON_REALTIME_ONLY;

std::ostream& operator<<(std::ostream& stream,
                         const RealtimeLogContext& context);

}  // namespace intrinsic::icon

namespace intrinsic_proto::data_logger {
// Converts `proto` into a RealtimeLogContext.
//
// Not realtime safe.
absl::StatusOr<intrinsic::icon::RealtimeLogContext> FromProto(
    const Context& proto) INTRINSIC_NON_REALTIME_ONLY;
}  // namespace intrinsic_proto::data_logger

#endif  // INTRINSIC_ICON_CONTROL_PARTS_REALTIME_LOG_CONTEXT_H_
