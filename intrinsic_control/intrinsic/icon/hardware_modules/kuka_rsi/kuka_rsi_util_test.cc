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

#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <iterator>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

#include "Eigen/Core"
#include "absl/algorithm/container.h"
#include "absl/status/status.h"
#include "absl/status/status_matchers.h"
#include "absl/strings/ascii.h"
#include "absl/strings/string_view.h"
#include "absl/strings/substitute.h"
#include "absl/time/time.h"
#include "internal/testing.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi.pb.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/proto/parse_text_proto.h"
#include "intrinsic/world/robot_payload/robot_payload.h"

namespace intrinsic::kuka {
using ::absl_testing::StatusIs;
using ::intrinsic::ParseTextProtoOrDie;
using ::testing::Combine;
using ::testing::HasSubstr;
using ::testing::Matcher;
using ::testing::Values;

TEST(KukaRsiUtilTest, LoadConfigFromProto) {
  intrinsic_proto::icon::KukaRsiModule proto_config = ParseTextProtoOrDie(
      R"pb(
        local_rsi_host: "192.170.10.123"
        local_rsi_port: 5852
        digital_input_names: [ "di_1", "di_2", "di_3", "di_4" ]
        digital_output_names: [ "do_1", "do_2" ],
      )pb");
  auto config = KukaConfig::FromProto(proto_config);
  EXPECT_OK(config);

  EXPECT_EQ(config->local_rsi_host, "192.170.10.123");
  EXPECT_EQ(config->local_rsi_port, 5852);
  EXPECT_EQ(config->num_digital_inputs, 4);
  EXPECT_EQ(config->num_digital_outputs, 2);
  EXPECT_TRUE(
      std::holds_alternative<std::monostate>(config->control_client_params));
}

TEST(KukaRsiUtilTest, LoadConfigFromProtoWithPlcConfig) {
  intrinsic_proto::icon::KukaRsiModule proto_config = ParseTextProtoOrDie(
      R"pb(
        local_rsi_host: "192.170.10.123"
        local_rsi_port: 5852
        plc_client_params {
          plc_opcua_address: "opc.tcp://172.16.14.1:4840"
          plc_opcua_node_name: "r5012"
          plc_opcua_timeout: { seconds: 30 }
        }
      )pb");
  auto config = KukaConfig::FromProto(proto_config);
  EXPECT_OK(config);

  ASSERT_TRUE(std::holds_alternative<KukaPlcClientParams>(
      config->control_client_params));
  auto plc_client_params =
      std::get<KukaPlcClientParams>(config->control_client_params);
  EXPECT_EQ(plc_client_params.opcua_address, "opc.tcp://172.16.14.1:4840");
  EXPECT_EQ(plc_client_params.opcua_node_name, "r5012");
  EXPECT_EQ(plc_client_params.opcua_timeout, absl::Seconds(30));
}

TEST(KukaRsiUtilTest, LoadConfigFromProtoWithControlEkiConfig) {
  intrinsic_proto::icon::KukaRsiModule proto_config = ParseTextProtoOrDie(
      R"pb(
        local_rsi_host: "192.170.10.123"
        local_rsi_port: 5852
        eki_control_client_params: {
          eki_address: "localhost"
          eki_port: 54600
          update_frequency: 10
        }
      )pb");
  auto config = KukaConfig::FromProto(proto_config);
  EXPECT_OK(config);

  ASSERT_TRUE(std::holds_alternative<intrinsic::kuka::KukaEkiClientParams>(
      config->control_client_params));
  auto eki_control_client_params =
      std::get<intrinsic::kuka::KukaEkiClientParams>(
          config->control_client_params);
  EXPECT_EQ(eki_control_client_params.eki_address, "localhost");
  EXPECT_EQ(eki_control_client_params.eki_port, 54600);
  EXPECT_EQ(eki_control_client_params.update_frequency, 10);
}

TEST(KukaRsiUtilTest, LoadConfigFromProtoWithStatusEkiConfig) {
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
  auto config = KukaConfig::FromProto(proto_config);
  EXPECT_OK(config);

  ASSERT_TRUE(std::holds_alternative<intrinsic::kuka::KukaEkiClientParams>(
      config->status_client_params));
  auto status_client_params = std::get<intrinsic::kuka::KukaEkiClientParams>(
      config->status_client_params);
  EXPECT_EQ(status_client_params.eki_address, "localhost");
  EXPECT_EQ(status_client_params.eki_port, 54600);
  EXPECT_EQ(status_client_params.update_frequency, 10);
}

TEST(KukaRsiUtilTest, InvalidEkiAddress) {
  intrinsic_proto::icon::KukaRsiModule proto_config = ParseTextProtoOrDie(
      R"pb(
        local_rsi_host: "192.170.10.123"
        local_rsi_port: 5852
        eki_control_client_params: {
          eki_address: ""
          eki_port: 1
          update_frequency: 10
        }
      )pb");
  EXPECT_THAT(KukaConfig::FromProto(proto_config),
              StatusIs(absl::StatusCode::kInvalidArgument,
                       HasSubstr("EKI host address is empty")));
}

TEST(KukaRsiUtilTest, InvalidEkiPortLow) {
  intrinsic_proto::icon::KukaRsiModule proto_config = ParseTextProtoOrDie(
      R"pb(
        local_rsi_host: "192.170.10.123"
        local_rsi_port: 5852
        eki_control_client_params: {
          eki_address: "localhost"
          eki_port: 0
          update_frequency: 10
        }
      )pb");
  EXPECT_THAT(KukaConfig::FromProto(proto_config),
              StatusIs(absl::StatusCode::kOutOfRange, HasSubstr("EKI port")));
}

TEST(KukaRsiUtilTest, InvalidEkiPortHigh) {
  intrinsic_proto::icon::KukaRsiModule proto_config = ParseTextProtoOrDie(
      R"pb(
        local_rsi_host: "192.170.10.123"
        local_rsi_port: 5852
        eki_control_client_params: {
          eki_address: "localhost"
          eki_port: 65536
          update_frequency: 10
        }
      )pb");
  EXPECT_THAT(KukaConfig::FromProto(proto_config),
              StatusIs(absl::StatusCode::kOutOfRange, HasSubstr("EKI port")));
}

TEST(KukaRsiUtilTest, InvalidUpdateFrequencyZero) {
  intrinsic_proto::icon::KukaRsiModule proto_config = ParseTextProtoOrDie(
      R"pb(
        local_rsi_host: "192.170.10.123"
        local_rsi_port: 5852
        eki_control_client_params: {
          eki_address: "localhost"
          eki_port: 1
          update_frequency: 0
        }
      )pb");
  EXPECT_THAT(KukaConfig::FromProto(proto_config),
              StatusIs(absl::StatusCode::kOutOfRange,
                       HasSubstr("EKI update frequency of")));
}

TEST(KukaRsiUtilTest, InvalidUpdateFrequencyHigh) {
  intrinsic_proto::icon::KukaRsiModule proto_config = ParseTextProtoOrDie(
      R"pb(
        local_rsi_host: "192.170.10.123"
        local_rsi_port: 5852
        eki_control_client_params: {
          eki_address: "localhost"
          eki_port: 1
          update_frequency: 15.1
        }
      )pb");
  EXPECT_THAT(KukaConfig::FromProto(proto_config),
              StatusIs(absl::StatusCode::kOutOfRange,
                       HasSubstr("EKI update frequency of")));
}

TEST(KukaRsiUtilTest, LoadConfigFromProtoWithMissingRsiHost) {
  intrinsic_proto::icon::KukaRsiModule proto_config = ParseTextProtoOrDie(
      R"pb(
        local_rsi_port: 5852
      )pb");
  EXPECT_THAT(KukaConfig::FromProto(proto_config),
              StatusIs(absl::StatusCode::kInvalidArgument,
                       HasSubstr("local_rsi_host")));
}

TEST(KukaRsiUtilTest, LoadConfigFromProtoWithMissingRsiPort) {
  intrinsic_proto::icon::KukaRsiModule proto_config = ParseTextProtoOrDie(
      R"pb(
        local_rsi_host: "192.170.10.123"
      )pb");
  EXPECT_THAT(KukaConfig::FromProto(proto_config),
              StatusIs(absl::StatusCode::kInvalidArgument,
                       HasSubstr("local_rsi_port")));
}

TEST(KukaRsiUtilTest, LoadConfigFromProtoWithTooManyDigitalInputs) {
  intrinsic_proto::icon::KukaRsiModule proto_config = ParseTextProtoOrDie(
      R"pb(
        local_rsi_host: "192.170.10.123" local_rsi_port: 5852
      )pb");
  for (size_t i = 0; i < kKukaMaxDigitalSignals + 1; ++i) {
    proto_config.mutable_digital_input_names()->Add("di_X");
  }
  EXPECT_THAT(
      KukaConfig::FromProto(proto_config),
      StatusIs(absl::StatusCode::kOutOfRange, HasSubstr("digital inputs")));
}

TEST(KukaRsiUtilTest, LoadConfigFromProtoWithTooManyDigitalOutputs) {
  intrinsic_proto::icon::KukaRsiModule proto_config = ParseTextProtoOrDie(
      R"pb(
        local_rsi_host: "192.170.10.123" local_rsi_port: 5852
      )pb");
  for (size_t i = 0; i < kKukaMaxDigitalSignals + 1; ++i) {
    proto_config.mutable_digital_output_names()->Add("do_X");
  }
  EXPECT_THAT(
      KukaConfig::FromProto(proto_config),
      StatusIs(absl::StatusCode::kOutOfRange, HasSubstr("digital outputs")));
}

TEST(KukaRsiUtilTest, PayloadConversionRoundTripWorks) {
  constexpr double deg2rad = M_PI / 180.0;
  const double rx = 10;
  const double ry = 30;
  const double rz = 75;

  Eigen::Matrix3d rotation_matrix =
      (Eigen::AngleAxisd(rz * deg2rad, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(ry * deg2rad, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(rx * deg2rad, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  Pose3d com(rotation_matrix, eigenmath::Vector3d(1, 2, 3));
  ASSERT_OK_AND_ASSIGN(
      auto payload,
      RobotPayload::Create(42.0, com,
                           eigenmath::Vector3d(20, 40, 50).asDiagonal()));
  ASSERT_OK_AND_ASSIGN(auto kuka_payload, FromRobotPayload(payload));
  ASSERT_OK_AND_ASSIGN(auto roundtrip_payload, ToRobotPayload(kuka_payload));

  EXPECT_EQ(payload, roundtrip_payload);
}

TEST(KukaRsiUtilTest, IsEqualPayloadWorks) {
  KukaPayload payload1;
  payload1.mass = 10.0;
  payload1.center_of_gravity_x = 1.0;
  payload1.center_of_gravity_y = 2.0;
  payload1.center_of_gravity_z = 3.0;
  payload1.center_of_gravity_a = 4.0;
  payload1.center_of_gravity_b = 5.0;
  payload1.center_of_gravity_c = 6.0;
  payload1.inertia_xx = 10.0;
  payload1.inertia_yy = 20.0;
  payload1.inertia_zz = 30.0;

  KukaPayload payload2 = payload1;  // Create an identical payload

  EXPECT_OK(payload1.IsApprox(payload2));

  // Test cases for each entry individually unequal
  // Mass
  payload2 = payload1;
  payload2.mass += 0.1;
  EXPECT_THAT(payload1.IsApprox(payload2),
              StatusIs(absl::StatusCode::kUnknown, HasSubstr("mass")));

  // Center of Gravity
  payload2 = payload1;
  payload2.center_of_gravity_x += 0.1;
  EXPECT_THAT(payload1.IsApprox(payload2),
              StatusIs(absl::StatusCode::kUnknown, HasSubstr("gravity x")));

  payload2 = payload1;
  payload2.center_of_gravity_y += 0.1;
  EXPECT_THAT(payload1.IsApprox(payload2),
              StatusIs(absl::StatusCode::kUnknown, HasSubstr("gravity y")));

  payload2 = payload1;
  payload2.center_of_gravity_z += 0.1;
  EXPECT_THAT(payload1.IsApprox(payload2),
              StatusIs(absl::StatusCode::kUnknown, HasSubstr("gravity z")));

  payload2 = payload1;
  payload2.center_of_gravity_a += 0.1;
  EXPECT_THAT(payload1.IsApprox(payload2),
              StatusIs(absl::StatusCode::kUnknown, HasSubstr("gravity a")));

  payload2 = payload1;
  payload2.center_of_gravity_b += 0.1;
  EXPECT_THAT(payload1.IsApprox(payload2),
              StatusIs(absl::StatusCode::kUnknown, HasSubstr("gravity b")));

  payload2 = payload1;
  payload2.center_of_gravity_c += 0.1;
  EXPECT_THAT(payload1.IsApprox(payload2),
              StatusIs(absl::StatusCode::kUnknown, HasSubstr("gravity c")));

  // Inertia
  payload2 = payload1;
  payload2.inertia_xx += 0.1;
  EXPECT_THAT(payload1.IsApprox(payload2),
              StatusIs(absl::StatusCode::kUnknown, HasSubstr(" inertia_xx")));

  payload2 = payload1;
  payload2.inertia_yy += 0.1;
  EXPECT_THAT(payload1.IsApprox(payload2),
              StatusIs(absl::StatusCode::kUnknown, HasSubstr("inertia_yy")));

  payload2 = payload1;
  payload2.inertia_zz += 0.1;
  EXPECT_THAT(payload1.IsApprox(payload2),
              StatusIs(absl::StatusCode::kUnknown, HasSubstr("inertia_zz")));
}

TEST(KukaRsiUtilTest, IsSameVersionWorks) {
  EXPECT_OK(Version::IsCompatibleVersion("protocol", Version({1, 0, 0}),
                                         Version({1, 0, 0})));
  EXPECT_OK(Version::IsCompatibleVersion("protocol", Version({1, 1, 0}),
                                         Version({1, 1, 0})));
  EXPECT_OK(Version::IsCompatibleVersion("protocol", Version({1, 1, 1}),
                                         Version({1, 1, 1})));
  EXPECT_OK(Version::IsCompatibleVersion("protocol", Version({0, 1, 0}),
                                         Version({0, 1, 0})));
  EXPECT_OK(Version::IsCompatibleVersion("protocol", Version({0, 0, 1}),
                                         Version({0, 0, 1})));
  // Patch version is ignored.
  EXPECT_OK(Version::IsCompatibleVersion("protocol", Version({0, 0, 2}),
                                         Version({0, 0, 1})));

  EXPECT_THAT(Version::IsCompatibleVersion("protocol", Version({2, 0, 0}),
                                           Version({1, 0, 0})),
              StatusIs(absl::StatusCode::kOutOfRange));
  EXPECT_THAT(Version::IsCompatibleVersion("protocol", Version({0, 2, 0}),
                                           Version({0, 1, 0})),
              StatusIs(absl::StatusCode::kOutOfRange));
  EXPECT_THAT(Version::IsCompatibleVersion("protocol", Version({2, 1, 0}),
                                           Version({2, 0, 0})),
              StatusIs(absl::StatusCode::kOutOfRange));
  EXPECT_THAT(Version::IsCompatibleVersion("protocol", Version({2, 1, 2}),
                                           Version({2, 0, 0})),
              StatusIs(absl::StatusCode::kOutOfRange));
}

TEST(KukaRsiUtilTest, StringVersionConversionWorks) {
  // Returns false due to invalid format.
  EXPECT_THAT(Version::CreateFromString("2.1").status(),
              StatusIs(absl::StatusCode::kInvalidArgument));
  EXPECT_THAT(Version::CreateFromString("2.1.a").status(),
              StatusIs(absl::StatusCode::kInvalidArgument));
  EXPECT_THAT(Version::CreateFromString("2.1.1.2").status(),
              StatusIs(absl::StatusCode::kInvalidArgument));
}

TEST(KukaRsiUtilTest, IsInVersionRangeWorks) {
  EXPECT_OK(Version::IsInVersionRange("protocol", Version({1, 0, 0}),
                                      Version({2, 0, 0}), Version({1, 0, 0})));
  EXPECT_OK(Version::IsInVersionRange("protocol", Version({1, 0, 0}),
                                      Version({2, 0, 0}), Version({1, 0, 99})));
  EXPECT_OK(Version::IsInVersionRange("protocol", Version({1, 0, 0}),
                                      Version({2, 0, 0}), Version({1, 0, 99})));
  EXPECT_OK(Version::IsInVersionRange("protocol", Version({1, 0, 0}),
                                      Version({2, 0, 0}), Version({1, 9, 99})));

  EXPECT_THAT(Version::IsInVersionRange("protocol", Version({1, 0, 0}),
                                        Version({2, 0, 0}), Version({2, 0, 0})),
              StatusIs(absl::StatusCode::kOutOfRange));
  EXPECT_THAT(Version::IsInVersionRange("protocol", Version({1, 0, 0}),
                                        Version({2, 0, 0}), Version({0, 0, 9})),
              StatusIs(absl::StatusCode::kOutOfRange));
  EXPECT_THAT(Version::IsInVersionRange("protocol", Version({1, 0, 0}),
                                        Version({2, 0, 0}), Version({0, 9, 9})),
              StatusIs(absl::StatusCode::kOutOfRange));
  EXPECT_THAT(Version::IsInVersionRange("protocol", Version({1, 0, 0}),
                                        Version({2, 0, 0}), Version({2, 0, 1})),
              StatusIs(absl::StatusCode::kOutOfRange));
  EXPECT_THAT(Version::IsInVersionRange("protocol", Version({1, 0, 0}),
                                        Version({2, 0, 0}), Version({3, 0, 0})),
              StatusIs(absl::StatusCode::kOutOfRange));
  EXPECT_THAT(Version::IsInVersionRange("protocol", Version({1, 0, 0}),
                                        Version({2, 0, 0}), Version({2, 1, 0})),
              StatusIs(absl::StatusCode::kOutOfRange));
  EXPECT_THAT(Version::IsInVersionRange("protocol", Version({1, 0, 0}),
                                        Version({2, 0, 0}), Version({2, 0, 1})),
              StatusIs(absl::StatusCode::kOutOfRange));

  EXPECT_THAT(Version::IsInVersionRange("protocol", Version({2, 0, 0}),
                                        Version({2, 0, 0}), Version({2, 0, 0})),
              StatusIs(absl::StatusCode::kInvalidArgument));
}

TEST(Version, Compares) {
  EXPECT_GT(Version({1, 1, 6}), Version({1, 1, 5}));
  EXPECT_GT(Version({1, 2, 5}), Version({1, 1, 5}));
  EXPECT_GT(Version({2, 1, 5}), Version({1, 1, 5}));

  EXPECT_EQ(Version({1, 2, 3}), Version({1, 2, 3}));
  EXPECT_NE(Version({1, 2, 3}), Version({1, 2, 4}));
  EXPECT_NE(Version({1, 2, 3}), Version({1, 3, 3}));
  EXPECT_NE(Version({1, 2, 3}), Version({2, 2, 3}));

  EXPECT_GE(Version({1, 1, 5}), Version({1, 1, 1}));
  EXPECT_GE(Version({1, 2, 3}), Version({1, 2, 3}));

  EXPECT_LT(Version({1, 1, 4}), Version({1, 1, 5}));

  EXPECT_LE(Version({1, 1, 5}), Version({1, 1, 5}));
  EXPECT_LE(Version({1, 1, 5}), Version({1, 1, 5}));
}

struct KukaRsiTelemetryParsingTestData {
  std::string test_name;
  std::string xml_string;
  absl::StatusCode expected_status_code;
  std::vector<std::string> expected_error_strings;
};

struct XmlNumberValue {
  std::string xml_value;
};

class KukaRsiTelemtryParsingTest
    : public ::testing::TestWithParam<
          std::tuple<KukaRsiTelemetryParsingTestData, XmlNumberValue>> {};

TEST_P(KukaRsiTelemtryParsingTest, ParseRsiTelemtry) {
  std::vector<Matcher<std::string>> substr_matchers;
  const KukaRsiTelemetryParsingTestData& parameters = std::get<0>(GetParam());
  absl::string_view number_value = std::get<1>(GetParam()).xml_value;
  absl::c_transform(
      parameters.expected_error_strings, std::back_inserter(substr_matchers),
      [](const std::string& substring) { return HasSubstr(substring); });
  RsiTelemetry telemetry;
  auto xml_string = absl::Substitute(parameters.xml_string, number_value);
  RSIXML xml;
  EXPECT_THAT(
      xml.ParseTelemetryRT(xml_string, telemetry),
      StatusIs(parameters.expected_status_code, AllOfArray(substr_matchers)));
}

std::string MakeTestNameCompatible(absl::string_view test_name) {
  std::string result(test_name);
  for (char& c : result) {
    if (!absl::ascii_isalnum(c) && c != '_') {
      c = '_';
    }
  }
  return result;
}

INSTANTIATE_TEST_SUITE_P(
    IncorrectXmlValues, KukaRsiTelemtryParsingTest,
    Combine(Values(

                KukaRsiTelemetryParsingTestData{
                    .test_name = "pos_wrong",
                    .xml_string = R"xml(
  <Rob Type="KUKA">
      <Delay D="11"/>
      <Pos A1="$0" A2="-90.04850" A3="89.99563" A4="-0.00145" A5="37.24480"
      A6="0.01919"/> <Trq A1="-17.3" A2="-45.05416" A3="-112.10144" A4="16.06333"
      A5="78.58109" A6="-16.09991"/> <RsiCorrectionActive>3</RsiCorrectionActive>
      <ModeOp>2</ModeOp>
      <IPOC>34627498973</IPOC>
  </Rob>
  )xml",
                    .expected_status_code = absl::StatusCode::kInvalidArgument,
                    .expected_error_strings = {"not convert"},
                },
                KukaRsiTelemetryParsingTestData{
                    .test_name = "trq_wrong",
                    .xml_string = R"xml(
  <Rob Type="KUKA">
      <Delay D="11"/>
      <Pos A1="123" A2="-90.04850" A3="89.99563" A4="-0.00145" A5="37.24480"
      A6="0.01919"/> <Trq A1="$0" A2="-45.05416" A3="-112.10144" A4="16.06333"
      A5="78.58109" A6="-16.09991"/> <RsiCorrectionActive>3</RsiCorrectionActive>
      <ModeOp>2</ModeOp>
      <IPOC>34627498973</IPOC>
  </Rob>
  )xml",
                    .expected_status_code = absl::StatusCode::kInvalidArgument,
                    .expected_error_strings = {"not convert"},
                },
                KukaRsiTelemetryParsingTestData{
                    .test_name = "ipoc_wrong",
                    .xml_string = R"xml(
  <Rob Type="KUKA">
      <Delay D="11"/>
      <Pos A1="123" A2="-90.04850" A3="89.99563" A4="-0.00145" A5="37.24480"
      A6="0.01919"/> <Trq A1="-17.3" A2="-45.05416" A3="-112.10144" A4="16.06333"
      A5="78.58109" A6="-16.09991"/> <RsiCorrectionActive>1</RsiCorrectionActive>
      <ModeOp>2</ModeOp>
      <IPOC>$0</IPOC>
  </Rob>
  )xml",
                    .expected_status_code = absl::StatusCode::kInvalidArgument,
                    .expected_error_strings = {"not convert"},
                },
                KukaRsiTelemetryParsingTestData{
                    .test_name = "mode_op_wrong",
                    .xml_string = R"xml(
  <Rob Type="KUKA">
      <Delay D="11"/>
      <Pos A1="123" A2="-90.04850" A3="89.99563" A4="-0.00145" A5="37.24480"
      A6="0.01919"/> <Trq A1="-17.3" A2="-45.05416" A3="-112.10144" A4="16.06333"
      A5="78.58109" A6="-16.09991"/> <RsiCorrectionActive>3</RsiCorrectionActive>
      <ModeOp>$0</ModeOp>
      <IPOC>34627498973</IPOC>
  </Rob>
  )xml",
                    .expected_status_code = absl::StatusCode::kInvalidArgument,
                    .expected_error_strings = {"not convert"},
                },
                KukaRsiTelemetryParsingTestData{
                    .test_name = "delay_wrong",
                    .xml_string = R"xml(
  <Rob Type="KUKA">
      <Delay D="$0"/>
      <Pos A1="213" A2="-90.04850" A3="89.99563" A4="-0.00145" A5="37.24480"
      A6="0.01919"/> <Trq A1="-17.3" A2="-45.05416" A3="-112.10144" A4="16.06333"
      A5="78.58109" A6="-16.09991"/> <RsiCorrectionActive>3</RsiCorrectionActive>
      <ModeOp>1</ModeOp>
      <IPOC>34627498973</IPOC>
  </Rob>
  )xml",
                    .expected_status_code = absl::StatusCode::kInvalidArgument,
                    .expected_error_strings = {"not convert"},
                }),
            Values(XmlNumberValue{.xml_value = ""},
                   XmlNumberValue{.xml_value = "error"},
                   XmlNumberValue{.xml_value = "123error"},
                   XmlNumberValue{.xml_value = "error123"},
                   XmlNumberValue{.xml_value = "not-mapped:16"})),
    [](const ::testing::TestParamInfo<KukaRsiTelemtryParsingTest::ParamType>&
           info) {
      return MakeTestNameCompatible(std::get<0>(info.param).test_name + "_" +
                                    std::get<1>(info.param).xml_value);
    });

INSTANTIATE_TEST_SUITE_P(
    CorrectFloatXmlValues, KukaRsiTelemtryParsingTest,
    Combine(Values(KukaRsiTelemetryParsingTestData{
                .test_name = "rsi_xml_ok",
                .xml_string = R"xml(
  <Rob Type="KUKA">
      <Delay D="11"/>
      <Pos A1="$0" A2="$0" A3="$0" A4="$0" A5="$0"
      A6="$0"/> <Trq A1="$0" A2="$0" A3="$0" A4="$0"
      A5="$0" A6="$0"/> <RsiCorrectionActive>3</RsiCorrectionActive>
      <ModeOp>2</ModeOp>
      <IPOC>34627498973</IPOC>
  </Rob>
  )xml",
                .expected_status_code = absl::StatusCode::kOk,
                .expected_error_strings = {},
            }),

            Values(XmlNumberValue{.xml_value = "1"},
                   XmlNumberValue{.xml_value = "-1"},
                   XmlNumberValue{.xml_value = "1.111"},
                   XmlNumberValue{.xml_value = "-90.04850"},
                   XmlNumberValue{.xml_value = "1e9"})),
    [](const ::testing::TestParamInfo<KukaRsiTelemtryParsingTest::ParamType>&
           info) {
      return MakeTestNameCompatible(std::get<0>(info.param).test_name + "_" +
                                    std::get<1>(info.param).xml_value);
    });

class MissingKukaRsiTelemtryNodeTest
    : public ::testing::TestWithParam<KukaRsiTelemetryParsingTestData> {};

TEST_P(MissingKukaRsiTelemtryNodeTest, ParseRsiTelemtry) {
  std::vector<Matcher<std::string>> substr_matchers;
  const KukaRsiTelemetryParsingTestData& parameters = GetParam();

  absl::c_transform(
      parameters.expected_error_strings, std::back_inserter(substr_matchers),
      [](const std::string& substring) { return HasSubstr(substring); });
  RsiTelemetry telemetry;
  RSIXML xml;
  EXPECT_THAT(
      xml.ParseTelemetryRT(parameters.xml_string, telemetry),
      StatusIs(parameters.expected_status_code, AllOfArray(substr_matchers)));
}

INSTANTIATE_TEST_SUITE_P(
    MissingKukaRsiTelemtryNodeTests, MissingKukaRsiTelemtryNodeTest,
    Values(

        KukaRsiTelemetryParsingTestData{
            .test_name = "wrong_root_node_name",
            .xml_string = R"xml(
  <Rob2 Type="KUKA">
      <Pos A1="123" A2="-90.04850" A3="89.99563" A4="-0.00145" A5="37.24480"
      A6="0.01919"/> <Trq A1="-17.3" A2="-45.05416" A3="-112.10144" A4="16.06333"
      A5="78.58109" A6="-16.09991"/> <RsiCorrectionActive>3</RsiCorrectionActive>
      <ModeOp>2</ModeOp>
      <IPOC>34627498973</IPOC>
  </Rob2>
  )xml",
            .expected_status_code = absl::StatusCode::kNotFound,
            .expected_error_strings = {"Rob"},
        },
        KukaRsiTelemetryParsingTestData{
            .test_name = "Delay",
            .xml_string = R"xml(
  <Rob Type="KUKA">
      <Pos A1="123" A2="-90.04850" A3="89.99563" A4="-0.00145" A5="37.24480"
      A6="0.01919"/> <Trq A1="-17.3" A2="-45.05416" A3="-112.10144" A4="16.06333"
      A5="78.58109" A6="-16.09991"/> <RsiCorrectionActive>3</RsiCorrectionActive>
      <ModeOp>2</ModeOp>
      <IPOC>34627498973</IPOC>
  </Rob>
  )xml",
            .expected_status_code = absl::StatusCode::kNotFound,
            .expected_error_strings = {"Delay"},
        },
        KukaRsiTelemetryParsingTestData{
            .test_name = "Pos",
            .xml_string = R"xml(
  <Rob Type="KUKA">
      <Delay D="11"/>
      <Trq A1="-17.3" A2="-45.05416" A3="-112.10144" A4="16.06333"
      A5="78.58109" A6="-16.09991"/> <RsiCorrectionActive>3</RsiCorrectionActive>
      <ModeOp>2</ModeOp>
      <IPOC>34627498973</IPOC>
  </Rob>
  )xml",
            .expected_status_code = absl::StatusCode::kNotFound,
            .expected_error_strings = {"Pos"},
        },
        KukaRsiTelemetryParsingTestData{
            .test_name = "Trq",
            .xml_string = R"xml(
  <Rob Type="KUKA">
      <Delay D="11"/>
      <Pos A1="-0.00008" A2="-90.04850" A3="89.99563" A4="-0.00145" A5="37.24480"
      A6="0.01919"/>  <RsiCorrectionActive>3</RsiCorrectionActive>
      <ModeOp>2</ModeOp>
      <IPOC>34627498973</IPOC>
  </Rob>
  )xml",
            .expected_status_code = absl::StatusCode::kNotFound,
            .expected_error_strings = {"Trq"},
        },
        KukaRsiTelemetryParsingTestData{
            .test_name = "RsiCorrectionActive",
            .xml_string = R"xml(
  <Rob Type="KUKA">
      <Delay D="11"/>
      <Pos A1="-0.00008" A2="-90.04850" A3="89.99563" A4="-0.00145" A5="37.24480"
      A6="0.01919"/> <Trq A1="-17.3" A2="-45.05416" A3="-112.10144" A4="16.06333"
      A5="78.58109" A6="-16.09991"/>
      <ModeOp>2</ModeOp>
      <IPOC>34627498973</IPOC>
  </Rob>
  )xml",
            .expected_status_code =
                absl::StatusCode::kOk,  // Currently OK for backwards
                                        // compatibility.
            .expected_error_strings = {},
        },
        KukaRsiTelemetryParsingTestData{
            .test_name = "ModeOp",
            .xml_string = R"xml(
  <Rob Type="KUKA">
      <Delay D="11"/>
      <Pos A1="-0.00008" A2="-90.04850" A3="89.99563" A4="-0.00145" A5="37.24480"
      A6="0.01919"/> <Trq A1="-17.3" A2="-45.05416" A3="-112.10144" A4="16.06333"
      A5="78.58109" A6="-16.09991"/> <RsiCorrectionActive>3</RsiCorrectionActive>
      <IPOC>34627498973</IPOC>
  </Rob>
  )xml",
            .expected_status_code = absl::StatusCode::kNotFound,
            .expected_error_strings = {"ModeOp"},
        },
        KukaRsiTelemetryParsingTestData{
            .test_name = "IPOC",
            .xml_string = R"xml(
  <Rob Type="KUKA">
      <Delay D="11"/>
      <Pos A1="-0.00008" A2="-90.04850" A3="89.99563" A4="-0.00145" A5="37.24480"
      A6="0.01919"/> <Trq A1="-17.3" A2="-45.05416" A3="-112.10144" A4="16.06333"
      A5="78.58109" A6="-16.09991"/> <RsiCorrectionActive>3</RsiCorrectionActive>
      <ModeOp>2</ModeOp>
   </Rob>
  )xml",
            .expected_status_code = absl::StatusCode::kNotFound,
            .expected_error_strings = {"IPOC"},
        }),
    [](const ::testing::TestParamInfo<
        MissingKukaRsiTelemtryNodeTest::ParamType>& info) {
      return info.param.test_name;
    });

}  // namespace intrinsic::kuka
