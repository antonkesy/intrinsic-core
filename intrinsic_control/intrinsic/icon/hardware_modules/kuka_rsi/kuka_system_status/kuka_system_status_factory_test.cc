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

#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_status/kuka_system_status_factory.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>

#include "absl/status/status.h"
#include "absl/status/status_matchers.h"
#include "internal/testing.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi.pb.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_status/kuka_system_status_interface.h"
#include "intrinsic/icon/utils/realtime_status_matchers.h"
#include "intrinsic/util/proto/parse_text_proto.h"

namespace intrinsic::kuka {
using ::absl_testing::IsOkAndHolds;
using ::absl_testing::StatusIs;
using ::intrinsic::ParseTextProtoOrDie;
using ::testing::HasSubstr;

TEST(KukaSystemStatusInterfaceFactoryTest, CreateEkiInstance) {
  intrinsic_proto::icon::KukaRsiModule proto_config = ParseTextProtoOrDie(
      R"pb(
        local_rsi_host: "192.170.10.123"
        local_rsi_port: 5852
        eki_status_client_params: {
          eki_address: "localhost"
          eki_port: 54600
          update_frequency: 10
        }
      )pb");
  ASSERT_OK_AND_ASSIGN(KukaConfig config, KukaConfig::FromProto(proto_config));
  // The EKI instance will try to connect to the KRC.
  EXPECT_THAT(CreateKukaSystemStatus(config),
              StatusIs(absl::StatusCode::kUnavailable,
                       HasSubstr("Failed to connect to address")));
}

TEST(KukaSystemStatusInterfaceFactoryTest, CreateNoOpInstance) {
  intrinsic_proto::icon::KukaRsiModule proto_config = ParseTextProtoOrDie(
      R"pb(
        local_rsi_host: "192.170.10.123" local_rsi_port: 5852
      )pb");
  ASSERT_OK_AND_ASSIGN(KukaConfig config, KukaConfig::FromProto(proto_config));
  ASSERT_OK_AND_ASSIGN(
      std::unique_ptr<KukaSystemStatusInterface> kuka_system_status,
      CreateKukaSystemStatus(config));

  EXPECT_NE(kuka_system_status, nullptr);
  EXPECT_NE(dynamic_cast<NoOpKukaSystemStatus*>(kuka_system_status.get()),
            nullptr);
  EXPECT_THAT(kuka_system_status->CurrentOpMode(),
              StatusIs(absl::StatusCode::kUnavailable));
  EXPECT_THAT(kuka_system_status->CurrentPosition(),
              icon::RealtimeStatusIs(absl::StatusCode::kUnavailable));

  EXPECT_THAT(kuka_system_status->ActiveErrorFlags(),
              IsOkAndHolds(KukaErrorFlagsMask::kNone));
}

}  // namespace intrinsic::kuka
