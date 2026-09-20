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

#ifndef INTRINSIC_RESOURCES_HEALTH_HEALTH_STATE_MACHINE_H__
#define INTRINSIC_RESOURCES_HEALTH_HEALTH_STATE_MACHINE_H__

#include <functional>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "intrinsic/assets/services/proto/v1/service_state.pb.h"
#include "intrinsic/resources/health/operational_status.h"

namespace intrinsic::resources {

// Class to manage the state machine transitions related to service state
// (previously known as resource health).
// intrinsic/assets/services/proto/v1/service_state.proto
// The class methods are *not* thread-safe and this responsibility is left to
// the user of the class.
class HealthStateMachine {
 public:
  // Creates a new instance given an initial operational state and a name for
  // the resource instance. The name is only used for logging purposes and in
  // absl status messages.
  explicit HealthStateMachine(OperationalState initial_state,
                              absl::string_view resource_name);

  // Puts the state machine in a specific state. This is useful if the resource
  // instance is found to be in a faulty or healthy (enabled or disabled) state
  // independent of the current state.
  void SetState(OperationalState new_state);

  // Returns the current ServiceState.
  intrinsic_proto::services::v1::SelfState GetState() const;

  // Returns absl::OkStatus() if the resource instance is enabled.
  absl::Status IsEnabled() const;

  // Returns absl::OkStatus() if enabling the resource instance succeeded.
  absl::Status Enable();

  // Returns absl::OkStatus() if disabling the resource instance succeeded. The
  // `disable_action` is invoked as part of disabling the resource instance and
  // must return absl::OkStatus() for this to succeed.
  absl::Status Disable(const std::function<absl::Status()>& disable_action);

  // Returns absl::OkStatus() if faults are cleared and the resource instance is
  // disabled. `clear_faults_action` is invoked as part of clearing the faults
  // and `disable_action` is called as part of disabling the resource instance.
  // Both must succeed for `ClearFaults` API to return success.
  absl::Status ClearFaultsAndDisable(
      const std::function<absl::Status()>& clear_faults_action,
      const std::function<absl::Status()>& disable_action);

 private:
  // Current operational state.
  OperationalStateWrapper state_;

  // Name of the resource. Only used for logging and status messages.
  const std::string resource_name_;
};

}  // namespace intrinsic::resources

#endif  // INTRINSIC_RESOURCES_HEALTH_HEALTH_STATE_MACHINE_H__
