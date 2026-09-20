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

#include "intrinsic/icon/utils/aggregated_health_to_state.h"

#include "intrinsic/assets/services/proto/v1/service_state.pb.h"
#include "intrinsic/simulation/gazebo/plugins/aggregated_resource_health.pb.h"
#include "intrinsic/util/status/extended_status.pb.h"

namespace intrinsic::icon {
intrinsic_proto::services::v1::SelfState
ConvertAggregatedResourceHealthToServiceState(
    const intrinsic_proto::simulation::ResourceHealthStatusResponse&
        health_status) {
  intrinsic_proto::services::v1::SelfState state;

  switch (health_status.status().state()) {
    case intrinsic_proto::simulation::OperationalStatus::DISABLED:
      state.set_state_code(
          intrinsic_proto::services::v1::SelfState::STATE_CODE_DISABLED);
      break;
    case intrinsic_proto::simulation::OperationalStatus::FAULTED:
      state.set_state_code(
          intrinsic_proto::services::v1::SelfState::STATE_CODE_ERROR);
      break;
    case intrinsic_proto::simulation::OperationalStatus::ENABLED:
      state.set_state_code(
          intrinsic_proto::services::v1::SelfState::STATE_CODE_ENABLED);
      break;
    default:
      state.set_state_code(
          intrinsic_proto::services::v1::SelfState::STATE_CODE_UNSPECIFIED);
      break;
  }

  if (!health_status.status().explanation().empty()) {
    state.mutable_extended_status()->mutable_user_report()->set_message(
        health_status.status().explanation());
  }

  return state;
}
}  // namespace intrinsic::icon
