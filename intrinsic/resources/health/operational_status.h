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

#ifndef INTRINSIC_RESOURCES_HEALTH_OPERATIONAL_STATUS_H_
#define INTRINSIC_RESOURCES_HEALTH_OPERATIONAL_STATUS_H_

#include <string>

#include "intrinsic/assets/services/proto/v1/service_state.pb.h"

namespace intrinsic::resources {

// Indicates the current operational state of the gripper resource instance.
// This is C++ enum equivalent for OperationalStatus::State proto:
// cs/intrinsic/resources/proto/resource_operational_status.proto;l=5
enum class OperationalState {
  // Indicates that the gripper is in an unspecified state.
  kUnspecified,
  // Indicates that gripper resource is disabled and can't be used.
  kDisabled,
  // Indicates that gripper resource is in a faulted state.
  kFaulted,
  // Indicates that gripper is enabled and ready to use.
  kEnabled,
};

// Returns a string corresponding to the operational state enum.
std::string OperationalStateToString(OperationalState state);

// Convenience wrapper class to handle and log operational state transitions.
class OperationalStateWrapper {
 public:
  // Allows implicit conversion from `OperationalState` enum since this wrapper
  // class represents the same data as the enum itself. This, for example,
  // allows the use of `initializer_list`:
  // `OperationalStateWrapper wrapper = {OperationalState::kUnspecified};`
  // NOLINTNEXTLINE(google-explicit-constructor)
  OperationalStateWrapper(OperationalState initial_state)
      : state_(initial_state) {}

  // Transitions to the new state. Not thread safe.
  void TransitionTo(OperationalState new_state);

  // Supports abseil's string formatting.
  template <typename Sink>
  friend void AbslStringify(Sink& sink, const OperationalStateWrapper& w) {
    absl::Format(&sink, "%s", OperationalStateToString(w.State()));
  }

  OperationalState State() const { return state_; }

 private:
  // This is made private so that every state transition using `TransitionTo`
  // can be logged.
  OperationalState state_;
};

// Returns the corresponding ServiceState::StateCode enum given the
// C++ OperationalState enum.
intrinsic_proto::services::v1::SelfState::StateCode
OperationStateEnumToServiceStateCode(OperationalState state);

// Returns a ServiceState proto given the C++-specific enum.
intrinsic_proto::services::v1::SelfState ToServiceState(OperationalState state);

}  // namespace intrinsic::resources

#endif  // INTRINSIC_RESOURCES_HEALTH_OPERATIONAL_STATUS_H_
