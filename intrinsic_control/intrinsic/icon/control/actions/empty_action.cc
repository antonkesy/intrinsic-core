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

#include "intrinsic/icon/control/actions/empty_action.h"

#include <memory>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/actions/empty_action_signature.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/common/builtins.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

// static
absl::StatusOr<std::unique_ptr<EmptyAction>> EmptyAction::Create(
    ActionFactoryContext& context) {
  INTR_RETURN_IF_ERROR(context.GetSlotInfo(kEmptyActionSlotName).status());
  return std::make_unique<EmptyAction>();
}

RealtimeStatus EmptyAction::OnEnter(OnEnterParameters params) {
  return OkStatus();
}

RealtimeStatus EmptyAction::Sense(SenseParameters params) { return OkStatus(); }

RealtimeStatus EmptyAction::Control(ControlParameters params) {
  return OkStatus();
}

RealtimeStatusOr<StateVariableValue> EmptyAction::GetStateVariable(
    absl::string_view name) const {
  if (name == kIsDone) {
    // The state of an Empty action is always done.
    return StateVariableValue(true);
  }
  return NotFoundError(
      RealtimeStatus::StrCat(name, " is not a registered state variable."));
}

}  // namespace intrinsic::icon
