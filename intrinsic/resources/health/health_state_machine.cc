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

#include "intrinsic/resources/health/health_state_machine.h"

#include <functional>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "intrinsic/assets/services/proto/v1/service_state.pb.h"
#include "intrinsic/resources/health/operational_status.h"
namespace intrinsic::resources {

HealthStateMachine::HealthStateMachine(const OperationalState initial_state,
                                       const absl::string_view resource_name)
    : state_(initial_state), resource_name_(resource_name) {}

intrinsic_proto::services::v1::SelfState HealthStateMachine::GetState() const {
  return ToServiceState(state_.State());
}

void HealthStateMachine::SetState(const OperationalState new_state) {
  state_.TransitionTo(new_state);
}

absl::Status HealthStateMachine::IsEnabled() const {
  const auto cur_state = state_.State();
  if (cur_state == intrinsic::resources::OperationalState::kEnabled) {
    return absl::OkStatus();
  }
  // Wraps `cur_state` in `OperationalStateWrapper` to print human readable
  // string for the state instead of an enum value.
  // TODO(dhirajgoel): Print capitalized resource name.
  return absl::FailedPreconditionError(
      absl::StrFormat("%s is not enabled (current operational state = %v).",
                      resource_name_, OperationalStateWrapper(cur_state)));
}

absl::Status HealthStateMachine::Enable() {
  switch (state_.State()) {
    case OperationalState::kEnabled:
      return absl::OkStatus();

    case OperationalState::kDisabled:
      state_.TransitionTo(OperationalState::kEnabled);
      return absl::OkStatus();

    case OperationalState::kUnspecified:
      return absl::FailedPreconditionError(absl::StrFormat(
          "Failed to enable %s since it is in unspecified state. "
          "Try `ClearFaults` to get to a known state.",
          resource_name_));

    case OperationalState::kFaulted:
      return absl::FailedPreconditionError(absl::StrFormat(
          "Failed to enable %s since it is in faulted state. Try "
          "`ClearFaults` to try to clear the fault.",
          resource_name_));
  }

  // Should never get here.
  return absl::InternalError(
      absl::StrCat("Invalid operational state = %v", state_));
}

absl::Status HealthStateMachine::Disable(
    const std::function<absl::Status()>& disable_action) {
  switch (state_.State()) {
    case OperationalState::kUnspecified:
      return absl::FailedPreconditionError(absl::StrFormat(
          "Unable to disable %s since it is in "
          "unspecified state. Try `ClearFaults` to get to a known state.",
          resource_name_));

    case OperationalState::kDisabled:
      return absl::OkStatus();

    case OperationalState::kFaulted:
      return absl::FailedPreconditionError(absl::StrFormat(
          "Unable to disable %s since it is in "
          "faulted state. Try `ClearFaults` to get to a known state.",
          resource_name_));

    case OperationalState::kEnabled: {
      if (const auto status = disable_action(); !status.ok()) {
        state_.TransitionTo(OperationalState::kFaulted);
        return absl::InternalError(
            absl::StrFormat("Disabling %s failed with the error: %s",
                            resource_name_, status.message()));
      }

      state_.TransitionTo(OperationalState::kDisabled);
      return absl::OkStatus();
    }
  }

  // Should never get here.
  return absl::InternalError(
      absl::StrCat("Invalid operational state = %v", state_));
}

absl::Status HealthStateMachine::ClearFaultsAndDisable(
    const std::function<absl::Status()>& clear_faults_action,
    const std::function<absl::Status()>& disable_action) {
  switch (state_.State()) {
    case OperationalState::kEnabled:
      state_.TransitionTo(OperationalState::kDisabled);
      return absl::OkStatus();

    case OperationalState::kDisabled:
      return absl::OkStatus();

    case OperationalState::kUnspecified:  // [[fallthrough]]
    case OperationalState::kFaulted: {
      if (const auto status = clear_faults_action(); !status.ok()) {
        state_.TransitionTo(OperationalState::kFaulted);
        return absl::InternalError(absl::StrCat(
            "Clearing the faults failed with the error: ", status.message()));
      }
      // Faults cleared. Let's disable the resource instance now.
      if (const auto status = disable_action(); !status.ok()) {
        state_.TransitionTo(OperationalState::kFaulted);
        return absl::InternalError(
            absl::StrFormat("Fault was cleared but disabling %s failed "
                            "with the error: %s",
                            resource_name_, status.message()));
      }

      // Everything good. Transition to disabled state.
      state_.TransitionTo(OperationalState::kDisabled);
      return absl::OkStatus();
    }
  }

  // Should never get here.
  return absl::InternalError(
      absl::StrCat("Invalid operational state = %v", state_));
}

}  // namespace intrinsic::resources
