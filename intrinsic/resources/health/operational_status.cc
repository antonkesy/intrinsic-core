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

#include "intrinsic/resources/health/operational_status.h"

#include <string>

#include "absl/log/log.h"
#include "intrinsic/assets/services/proto/v1/service_state.pb.h"
#include "intrinsic/util/status/extended_status.pb.h"

namespace intrinsic::resources {

void OperationalStateWrapper::TransitionTo(OperationalState new_state) {
  if (state_ != new_state) {
    LOG(INFO) << "Operational state transitioned from `"
              << OperationalStateToString(state_) << "` to `"
              << OperationalStateToString(new_state) << "`";
    state_ = new_state;
  }
}

std::string OperationalStateToString(OperationalState state) {
  switch (state) {
    case OperationalState::kUnspecified:
      return "Unspecified";
    case OperationalState::kDisabled:
      return "Disabled";
    case OperationalState::kFaulted:
      return "Faulted";
    case OperationalState::kEnabled:
      return "Enabled";
  }
}

intrinsic_proto::services::v1::SelfState::StateCode
OperationStateEnumToServiceStateCode(const OperationalState state) {
  switch (state) {
    case OperationalState::kEnabled:
      return intrinsic_proto::services::v1::SelfState::STATE_CODE_ENABLED;
    case OperationalState::kDisabled:
      return intrinsic_proto::services::v1::SelfState::STATE_CODE_DISABLED;
    case OperationalState::kFaulted:
      return intrinsic_proto::services::v1::SelfState::STATE_CODE_ERROR;
    case OperationalState::kUnspecified:
    default:
      return intrinsic_proto::services::v1::SelfState::STATE_CODE_UNSPECIFIED;
  }
}

intrinsic_proto::services::v1::SelfState ToServiceState(
    const OperationalState state) {
  intrinsic_proto::services::v1::SelfState service_state;
  service_state.set_state_code(OperationStateEnumToServiceStateCode(state));

  if (state == OperationalState::kUnspecified) {
    service_state.mutable_extended_status()->set_title(
        "Gripper health is unknown");
    service_state.mutable_extended_status()->mutable_user_report()->set_message(
        "The gripper is in an unknown state.");
  } else if (state == OperationalState::kFaulted) {
    service_state.mutable_extended_status()->set_title("Gripper errored");
    service_state.mutable_extended_status()->mutable_user_report()->set_message(
        "Try `Enable` to get back to a healthy state.");
  }
  return service_state;
}

}  // namespace intrinsic::resources
