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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_MOCK_KUKA_RSI_CLIENT_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_MOCK_KUKA_RSI_CLIENT_H_

#include <gmock/gmock.h>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_client.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::kuka::testing {

class MockKukaRsiClient : public RealtimeKukaRsiClient {
 public:
  MOCK_METHOD(absl::Status, Init, (const KukaConfig& config), (override));

  MOCK_METHOD(RsiTelemetry, GetTelemetry, (), (const, override));

  MOCK_METHOD(intrinsic::icon::RealtimeStatus, SendCommand,
              (absl::Span<const double> joint_positions,
               absl::Span<const bool> digital_outputs),
              (override));

  MOCK_METHOD(icon::RealtimeStatus, Activate, (), (override));
  MOCK_METHOD(icon::RealtimeStatus, Deactivate, (), (override));

  MOCK_METHOD(bool, IsActive, (), (const, override));
  MOCK_METHOD(bool, IsEnabled, (), (const, override));

  MOCK_METHOD(icon::RealtimeStatus, GetFault, (), (const, override));

  MOCK_METHOD(absl::Status, StartRSI, (absl::Time deadline), (override));

  MOCK_METHOD(absl::Status, StopRSI, (), (override));
};

}  // namespace intrinsic::kuka::testing

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_MOCK_KUKA_RSI_CLIENT_H_
