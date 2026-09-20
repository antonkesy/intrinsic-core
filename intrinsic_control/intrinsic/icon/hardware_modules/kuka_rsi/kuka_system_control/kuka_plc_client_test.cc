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

#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_plc_client.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "absl/status/status.h"
#include "absl/status/status_matchers.h"
#include "absl/time/time.h"

namespace intrinsic::kuka {
using ::absl_testing::StatusIs;
using ::testing::HasSubstr;

TEST(KukaPlcClientTest, Create) {
  // More tests would need an actual OPC-UA server.
  EXPECT_THAT(KukaPlcClient::Create("foo", "robot", absl::Seconds(1)),
              StatusIs(absl::StatusCode::kUnavailable,
                       HasSubstr("not able to connect")));
}

}  // namespace intrinsic::kuka
