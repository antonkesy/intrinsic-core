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

#ifndef INTRINSIC_ICON_CONTROL_MOCK_REALTIME_CLOCK_H_
#define INTRINSIC_ICON_CONTROL_MOCK_REALTIME_CLOCK_H_

#include <gmock/gmock.h>

#include "intrinsic/icon/control/realtime_clock_interface.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

// Mock implementation of RealtimeClockInterface.
// Useful when testing buses without bringing up all of Timeslicer.
class MockRealtimeClock : public RealtimeClockInterface {
 public:
  MOCK_METHOD(RealtimeStatus, TickBlockingWithDeadline,
              (intrinsic::Time current_timestamp, absl::Time deadline),
              (override));
  MOCK_METHOD(RealtimeStatus, Reset, (absl::Duration timeout), (override));
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_MOCK_REALTIME_CLOCK_H_
