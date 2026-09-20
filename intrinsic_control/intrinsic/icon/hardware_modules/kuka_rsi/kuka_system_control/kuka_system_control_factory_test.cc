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

#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_system_control_factory.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>

#include "absl/status/status.h"
#include "absl/status/status_matchers.h"
#include "internal/testing.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi.pb.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_system_control_interface.h"
#include "intrinsic/util/proto/parse_text_proto.h"

namespace intrinsic::kuka {
using ::absl_testing::StatusIs;
using ::intrinsic::ParseTextProtoOrDie;
using ::testing::AnyOf;
using ::testing::HasSubstr;
using ::testing::UnorderedElementsAreArray;

TEST(KukaSystemControlInterfaceFactoryTest, CreatePlcInstance) {
  intrinsic_proto::icon::KukaRsiModule proto_config = ParseTextProtoOrDie(
      R"pb(
        local_rsi_host: "192.170.10.123"
        local_rsi_port: 5852
        plc_client_params {
          plc_opcua_address: "opc.tcp://172.16.14.1:4840"
          plc_opcua_node_name: "r5012"
        }
      )pb");
  ASSERT_OK_AND_ASSIGN(KukaConfig config, KukaConfig::FromProto(proto_config));
  // The PLC instance will try to connect to OPC-UA and fail on that.
  EXPECT_THAT(CreateKukaSystemControl(config),
              StatusIs(absl::StatusCode::kUnavailable,
                       AnyOf(HasSubstr("BadDisconnect"),
                             HasSubstr("BadConnectionClosed"))));
}

TEST(KukaSystemControlInterfaceFactoryTest, CreateNoOpInstance) {
  intrinsic_proto::icon::KukaRsiModule proto_config = ParseTextProtoOrDie(
      R"pb(
        local_rsi_host: "192.170.10.123" local_rsi_port: 5852
      )pb");
  ASSERT_OK_AND_ASSIGN(KukaConfig config, KukaConfig::FromProto(proto_config));
  ASSERT_OK_AND_ASSIGN(
      std::unique_ptr<KukaSystemControlInterface> kuka_system_control,
      CreateKukaSystemControl(config));

  EXPECT_NE(kuka_system_control, nullptr);
  EXPECT_NE(dynamic_cast<NoOpKukaSystemControl*>(kuka_system_control.get()),
            nullptr);

  EXPECT_THAT(
      kuka_system_control->CompatibleOperationModes(),
      UnorderedElementsAreArray({OpMode::kExternal, OpMode::kAutomatic}));
}

}  // namespace intrinsic::kuka
