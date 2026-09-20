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

#ifndef INTRINSIC_ICON_CONTROL_MOCK_RTCL_ACTION_H_
#define INTRINSIC_ICON_CONTROL_MOCK_RTCL_ACTION_H_

#include <gmock/gmock.h>

#include <optional>

#include "absl/strings/string_view.h"
#include "intrinsic/icon/control/reaction.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/utils/malloc_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

class MockRtclAction : public RtclActionInterface {
 public:
  bool ignore_malloc_guard_in_mock_methods_ = true;

  MOCK_METHOD(RealtimeStatus, OnEnterMock, (OnEnterParameters params));
  MOCK_METHOD(RealtimeStatus, SenseMock, (SenseParameters params));
  MOCK_METHOD(RealtimeStatus, ControlMock, (ControlParameters params));
  MOCK_METHOD(RealtimeStatusOr<StateVariableValue>, GetStateVariableMock,
              (absl::string_view name), (const));

  RealtimeStatus OnEnter(OnEnterParameters params) override {
    std::optional<::intrinsic::icon::ScopedMallocGuardIgnore> ignore;
    if (ignore_malloc_guard_in_mock_methods_) ignore.emplace();
    return OnEnterMock(params);
  }

  RealtimeStatus Sense(SenseParameters params) override {
    std::optional<::intrinsic::icon::ScopedMallocGuardIgnore> ignore;
    if (ignore_malloc_guard_in_mock_methods_) ignore.emplace();
    return SenseMock(params);
  }

  RealtimeStatus Control(ControlParameters params) override {
    std::optional<::intrinsic::icon::ScopedMallocGuardIgnore> ignore;
    if (ignore_malloc_guard_in_mock_methods_) ignore.emplace();
    return ControlMock(params);
  }

  RealtimeStatusOr<StateVariableValue> GetStateVariable(
      absl::string_view name) const override {
    std::optional<::intrinsic::icon::ScopedMallocGuardIgnore> ignore;
    if (ignore_malloc_guard_in_mock_methods_) ignore.emplace();
    return GetStateVariableMock(name);
  }
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_MOCK_RTCL_ACTION_H_
