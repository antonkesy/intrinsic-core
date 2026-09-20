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

#include "intrinsic/icon/control/parts/realtime_log_context.h"

#include <ostream>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "intrinsic/logging/proto/context.pb.h"

namespace intrinsic_proto::data_logger {

absl::StatusOr<intrinsic::icon::RealtimeLogContext> FromProto(
    const intrinsic_proto::data_logger::Context& proto) {
  if (proto.deployment_id().size() > intrinsic::icon::kDeploymentIdSize) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "The size of the deployment_id is %i, maximum is %i",
        proto.deployment_id().size(), intrinsic::icon::kDeploymentIdSize));
  }
  intrinsic::icon::RealtimeLogContext context;
  context.deployment_id.append(proto.deployment_id());
  context.executive_session_id = proto.executive_session_id();
  context.executive_plan_id = proto.executive_plan_id();
  context.executive_plan_action_id = proto.executive_plan_action_id();
  context.skill_id = proto.skill_id();
  context.parent_skill_id = proto.parent_skill_id();
  if (proto.has_icon_session_id()) {
    context.icon_session_id = proto.icon_session_id();
  }
  if (proto.has_icon_action_id()) {
    context.icon_action_id = proto.icon_action_id();
  }
  return context;
}
}  // namespace intrinsic_proto::data_logger

namespace intrinsic::icon {

intrinsic_proto::data_logger::Context ToProto(
    const RealtimeLogContext& realtime_log_context) {
  intrinsic_proto::data_logger::Context proto;
  proto.set_deployment_id(realtime_log_context.deployment_id);
  proto.set_executive_session_id(realtime_log_context.executive_session_id);
  proto.set_executive_plan_id(realtime_log_context.executive_plan_id);
  proto.set_executive_plan_action_id(
      realtime_log_context.executive_plan_action_id);
  proto.set_skill_id(realtime_log_context.skill_id);
  proto.set_parent_skill_id(realtime_log_context.parent_skill_id);
  if (realtime_log_context.icon_session_id.has_value()) {
    proto.set_icon_session_id(realtime_log_context.icon_session_id.value());
  }
  if (realtime_log_context.icon_action_id.has_value()) {
    proto.set_icon_action_id(realtime_log_context.icon_action_id.value());
  }
  return proto;
}

bool RealtimeLogContext::operator==(const RealtimeLogContext& other) const {
  return deployment_id == other.deployment_id &&
         executive_session_id == other.executive_session_id &&
         executive_plan_id == other.executive_plan_id &&
         executive_plan_action_id == other.executive_plan_action_id &&
         skill_id == other.skill_id &&
         parent_skill_id == other.parent_skill_id &&
         icon_session_id == other.icon_session_id &&
         icon_action_id == other.icon_action_id;
}

bool RealtimeLogContext::operator!=(const RealtimeLogContext& other) const {
  return !(*this == other);
}

std::ostream& operator<<(std::ostream& stream,
                         const RealtimeLogContext& context) {
  stream << absl::StrCat(ToProto(context));
  return stream;
}

}  // namespace intrinsic::icon
