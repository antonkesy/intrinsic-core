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

#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_eki_system_control.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/status_matchers.h"
#include "absl/strings/string_view.h"
#include "absl/strings/substitute.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "internal/testing.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/testing/mock_tcp_client.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/icon/utils/xml/realtime_xml_generator.h"

using ::absl_testing::IsOkAndHolds;
using ::absl_testing::StatusIs;
using ::testing::_;
using ::testing::HasSubstr;
using ::testing::Return;
using ::testing::UnorderedElementsAreArray;

namespace intrinsic::kuka {

icon::RealtimeStatusOr<std::string> CreateEkiTelemetryXml(
    const KukaEkiSystemControl::EkiTelemetry& telemetry) {
  RealtimeXmlGenerator generator(100, 100);

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto root, generator.AddRootElement("Robot"));

  INTRINSIC_RT_RETURN_IF_ERROR(
      root->AddElementWithText("OpMode", telemetry.op_mode).status());
  INTRINSIC_RT_RETURN_IF_ERROR(
      root->AddElementWithText("MessagesPresent", telemetry.messages_present)
          .status());
  INTRINSIC_RT_RETURN_IF_ERROR(
      root->AddElementWithText("RSIActive", telemetry.rsi_active).status());
  INTRINSIC_RT_RETURN_IF_ERROR(
      root->AddElementWithText("ProgramActive", telemetry.program_active)
          .status());
  INTRINSIC_RT_RETURN_IF_ERROR(
      root->AddElementWithText("ProgramSelected", telemetry.program_selected)
          .status());
  INTRINSIC_RT_RETURN_IF_ERROR(
      root->AddElementWithText("SelectedProgramPath",
                               telemetry.selected_program_path)
          .status());
  std::string xml_str;
  xml_str.resize(1000);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto size, generator.GenerateXMLString(
                     absl::MakeSpan(xml_str.data(), xml_str.size())));
  xml_str.resize(size);
  return xml_str;
}

std::string CreateEkiTelemetryXmlChecked(
    const KukaEkiSystemControl::EkiTelemetry& telemetry) {
  auto result = CreateEkiTelemetryXml(telemetry);
  CHECK(result.ok()) << result.status();
  return result.value();
}

class KukaEkiSystemControlTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto tcp_client = std::make_unique<testing::MockTcpClient>();
    tcp_client_ = tcp_client.get();
    ON_CALL(*tcp_client, Connect(_)).WillByDefault(Return(absl::OkStatus()));
    ON_CALL(*tcp_client, Disconnect()).WillByDefault(Return());
    ON_CALL(*tcp_client, IsConnected()).WillByDefault(Return(true));
    ON_CALL(*tcp_client, Send(_, _)).WillByDefault(Return(icon::OkStatus()));
    ON_CALL(*tcp_client, Receive(_, _, _, _, _))
        .WillByDefault(Return(icon::FailedPreconditionError("")));

    EXPECT_CALL(*tcp_client, IsConnected()).WillRepeatedly(Return(true));
    EXPECT_CALL(*tcp_client, Connect(_)).WillOnce((Return(absl::OkStatus())));

    ASSERT_OK_AND_ASSIGN(kuka_eki_system_control_,
                         KukaEkiSystemControl::Create(
                             KukaEkiClientParams{.eki_address = "localhost",
                                                 .eki_port = 54600,
                                                 .update_frequency = 10.0},
                             std::move(tcp_client)));
  }

  std::function<icon::RealtimeStatusOr<int32_t>(
      absl::Duration timeout, absl::Span<uint8_t> buffer, int recv_flags)>
  CreateEkiTelemetryMockFunction(const std::string& telemetry_str) {
    return [telemetry = telemetry_str](
               absl::Duration timeout, absl::Span<uint8_t> buffer,
               int recv_flags) -> icon::RealtimeStatusOr<int32_t> {
      CHECK(buffer.size() >= telemetry.size());
      (void)memcpy(buffer.data(), (telemetry.data()), telemetry.size());
      return telemetry.size();
    };
  }

  std::string CreateEkiCommandXml(
      KukaEkiSystemControl::EkiOperation operation) {
    return absl::Substitute(
        R"xml(<Intrinsic><Update/><Operation>$0</Operation></Intrinsic>)xml",
        KukaEkiSystemControl::EkiOperationToString(operation));
  }

 protected:
  std::unique_ptr<KukaEkiSystemControl> kuka_eki_system_control_;
  testing::MockTcpClient* tcp_client_;
};

TEST_F(KukaEkiSystemControlTest, CompatibleOperationModes) {
  EXPECT_THAT(kuka_eki_system_control_->CompatibleOperationModes(),
              UnorderedElementsAreArray({OpMode::kExternal}));
}

TEST_F(KukaEkiSystemControlTest, StartRSI) {
  std::string start_rsi_eki_command_xml =
      CreateEkiCommandXml(KukaEkiSystemControl::EkiOperation::kStartProgram);
  std::string default_eki_command_xml =
      CreateEkiCommandXml(KukaEkiSystemControl::EkiOperation::kNone);
  EXPECT_CALL(*tcp_client_,
              Send(absl::MakeSpan(reinterpret_cast<const uint8_t*>(
                                      default_eki_command_xml.data()),
                                  default_eki_command_xml.size()),
                   _))
      .WillRepeatedly(Return(icon::OkStatus()));
  // Register the specific "StartProgram" call with a callback that tells us
  // that this call happened.
  bool command_sent = false;
  EXPECT_CALL(*tcp_client_,
              Send(absl::MakeSpan(reinterpret_cast<const uint8_t*>(
                                      start_rsi_eki_command_xml.data()),
                                  start_rsi_eki_command_xml.size()),
                   _))
      .WillOnce(::testing::DoAll([&command_sent] { command_sent = true; },
                                 Return(icon::OkStatus())));
  std::string startup_telemetry_eki_xml = CreateEkiTelemetryXmlChecked(
      KukaEkiSystemControl::EkiTelemetry{.op_mode = OpMode::kExternal});
  std::string rsi_started_telemetry_eki_xml = CreateEkiTelemetryXmlChecked(
      KukaEkiSystemControl::EkiTelemetry{.op_mode = OpMode::kExternal,
                                         .rsi_active = true,
                                         .program_active = true});

  // Register a call that will return the start up telemetry data and will
  // return the "RSI started" data as soon as the "StartProgram" call was
  // registered.
  EXPECT_CALL(*tcp_client_, Receive(_, _, _, _, _))
      .WillRepeatedly([&command_sent, startup_telemetry_eki_xml,
                       rsi_started_telemetry_eki_xml](
                          absl::Duration timeout, bool recv_until_buffer_full,
                          int recv_flags, absl::Span<uint8_t> buffer,
                          size_t& bytes_read) {
        CHECK(buffer.size() >= startup_telemetry_eki_xml.size());
        CHECK(buffer.size() >= rsi_started_telemetry_eki_xml.size());
        // Check which data EKI would return.
        if (!command_sent) {
          (void)memcpy(buffer.data(), (startup_telemetry_eki_xml.data()),
                       startup_telemetry_eki_xml.size());
          bytes_read = startup_telemetry_eki_xml.size();
        } else {
          (void)memcpy(buffer.data(), (rsi_started_telemetry_eki_xml.data()),
                       rsi_started_telemetry_eki_xml.size());
          bytes_read = rsi_started_telemetry_eki_xml.size();
        }
        return icon::OkStatus();
      });

  // We need to wait until we have processed the first telemetry since the RSI
  // can only be started when in External mode.
  while (kuka_eki_system_control_->Telemetry().op_mode != OpMode::kExternal) {
    absl::SleepFor(absl::Seconds(0.1));
  }
  EXPECT_OK(kuka_eki_system_control_->StartRSI(absl::Seconds(2)));
  kuka_eki_system_control_
      .reset();  // This instance needs to be destroyed here so that it won't
                 // access the mock strings anymore.
}

TEST_F(KukaEkiSystemControlTest, StartRSIFailsDueToProgramNotActive) {
  std::string start_rsi_eki_command_xml =
      CreateEkiCommandXml(KukaEkiSystemControl::EkiOperation::kStartProgram);
  std::string default_eki_command_xml =
      CreateEkiCommandXml(KukaEkiSystemControl::EkiOperation::kNone);
  EXPECT_CALL(*tcp_client_,
              Send(absl::MakeSpan(reinterpret_cast<const uint8_t*>(
                                      default_eki_command_xml.data()),
                                  default_eki_command_xml.size()),
                   _))
      .WillRepeatedly(Return(icon::OkStatus()));
  // Register the specific "StartProgram" call with a callback that tells us
  // that this call happened.
  bool command_sent = false;
  EXPECT_CALL(*tcp_client_,
              Send(absl::MakeSpan(reinterpret_cast<const uint8_t*>(
                                      start_rsi_eki_command_xml.data()),
                                  start_rsi_eki_command_xml.size()),
                   _))
      .WillOnce(::testing::DoAll([&command_sent] { command_sent = true; },
                                 Return(icon::OkStatus())));
  std::string startup_telemetry_eki_xml = CreateEkiTelemetryXmlChecked(
      KukaEkiSystemControl::EkiTelemetry{.op_mode = OpMode::kExternal});
  std::string rsi_started_telemetry_eki_xml = CreateEkiTelemetryXmlChecked(
      KukaEkiSystemControl::EkiTelemetry{.op_mode = OpMode::kExternal,
                                         .rsi_active = true,
                                         .program_active = false});

  // Register a call that will return the start up telemetry data and will
  // return the "RSI started" data as soon as the "StartProgram" call was
  // registered.
  EXPECT_CALL(*tcp_client_, Receive(_, _, _, _, _))
      .WillRepeatedly([&command_sent, &startup_telemetry_eki_xml,
                       &rsi_started_telemetry_eki_xml](
                          absl::Duration timeout, bool recv_until_buffer_full,
                          int recv_flags, absl::Span<uint8_t> buffer,
                          size_t& bytes_read) {
        CHECK(buffer.size() >= startup_telemetry_eki_xml.size());
        CHECK(buffer.size() >= rsi_started_telemetry_eki_xml.size());
        // Check which data EKI would return.
        if (!command_sent) {
          (void)memcpy(buffer.data(), (startup_telemetry_eki_xml.data()),
                       startup_telemetry_eki_xml.size());
          bytes_read = startup_telemetry_eki_xml.size();
        } else {
          (void)memcpy(buffer.data(), (rsi_started_telemetry_eki_xml.data()),
                       rsi_started_telemetry_eki_xml.size());
          bytes_read = rsi_started_telemetry_eki_xml.size();
        }
        return icon::OkStatus();
      });

  // We need to wait until we have processed the first telemetry since the RSI
  // can only be started when in External mode.
  while (kuka_eki_system_control_->Telemetry().op_mode != OpMode::kExternal) {
    absl::SleepFor(absl::Seconds(0.1));
  }
  EXPECT_THAT(kuka_eki_system_control_->StartRSI(absl::Seconds(2)),
              StatusIs(absl::StatusCode::kDeadlineExceeded));
  kuka_eki_system_control_
      .reset();  // This instance needs to be destroyed here so that it won't
                 // access the mock strings anymore.
}

TEST_F(KukaEkiSystemControlTest, StartRSIFailsDueToWrongOpMode) {
  EXPECT_THAT(kuka_eki_system_control_->StartRSI(absl::Seconds(2)),
              StatusIs(absl::StatusCode::kUnavailable,
                       HasSubstr("KUKA is not in 'External' mode")));
}

TEST_F(KukaEkiSystemControlTest, StopRsi) {
  std::string stop_rsi_eki_command_xml =
      CreateEkiCommandXml(KukaEkiSystemControl::EkiOperation::kStopProgram);
  std::string cancel_program_eki_command_xml =
      CreateEkiCommandXml(KukaEkiSystemControl::EkiOperation::kCancelProgram);
  std::string default_eki_command_xml =
      CreateEkiCommandXml(KukaEkiSystemControl::EkiOperation::kNone);
  EXPECT_CALL(*tcp_client_,
              Send(absl::MakeSpan(reinterpret_cast<const uint8_t*>(
                                      default_eki_command_xml.data()),
                                  default_eki_command_xml.size()),
                   _))
      .WillRepeatedly(Return(icon::OkStatus()));
  // Register the specific "StopProgram" and "CancelProgram" calls with a
  // callback that tells us that these calls happened.
  bool stop_rsi_command_sent = false;
  bool cancel_program_command_sent = false;
  EXPECT_CALL(*tcp_client_,
              Send(absl::MakeSpan(reinterpret_cast<const uint8_t*>(
                                      stop_rsi_eki_command_xml.data()),
                                  stop_rsi_eki_command_xml.size()),
                   _))
      .WillOnce(::testing::DoAll(
          [&stop_rsi_command_sent] { stop_rsi_command_sent = true; },
          Return(icon::OkStatus())));
  EXPECT_CALL(*tcp_client_,
              Send(absl::MakeSpan(reinterpret_cast<const uint8_t*>(
                                      cancel_program_eki_command_xml.data()),
                                  cancel_program_eki_command_xml.size()),
                   _))
      .WillOnce(::testing::DoAll(
          [&cancel_program_command_sent] {
            cancel_program_command_sent = true;
          },
          Return(icon::OkStatus())));
  std::string startup_telemetry_eki_xml = CreateEkiTelemetryXmlChecked(
      KukaEkiSystemControl::EkiTelemetry{.op_mode = OpMode::kExternal});
  std::string program_still_selected_telemetry_eki_xml =
      CreateEkiTelemetryXmlChecked(KukaEkiSystemControl::EkiTelemetry{
          .op_mode = OpMode::kExternal, .program_selected = true});
  std::string rsi_started_telemetry_eki_xml = CreateEkiTelemetryXmlChecked(
      KukaEkiSystemControl::EkiTelemetry{.op_mode = OpMode::kExternal,
                                         .rsi_active = true,
                                         .program_active = true});

  // Register a call that will return the "RSI active" telemetry data and will
  // return the "RSI stopped" data as soon as the "StopProgram" call was
  // registered.
  EXPECT_CALL(*tcp_client_, Receive(_, _, _, _, _))
      .WillRepeatedly(
          [&stop_rsi_command_sent, &cancel_program_command_sent,
           &startup_telemetry_eki_xml, &rsi_started_telemetry_eki_xml,
           &program_still_selected_telemetry_eki_xml](
              absl::Duration timeout, bool recv_until_buffer_full,
              int recv_flags, absl::Span<uint8_t> buffer, size_t& bytes_read) {
            CHECK(buffer.size() >= startup_telemetry_eki_xml.size());
            CHECK(buffer.size() >= rsi_started_telemetry_eki_xml.size());
            // Check which data EKI would return.
            if (stop_rsi_command_sent && cancel_program_command_sent) {
              (void)memcpy(buffer.data(), (startup_telemetry_eki_xml.data()),
                           startup_telemetry_eki_xml.size());
              bytes_read = startup_telemetry_eki_xml.size();
            } else if (stop_rsi_command_sent) {
              (void)memcpy(buffer.data(),
                           (program_still_selected_telemetry_eki_xml.data()),
                           program_still_selected_telemetry_eki_xml.size());
              bytes_read = program_still_selected_telemetry_eki_xml.size();
            } else {
              (void)memcpy(buffer.data(),
                           (rsi_started_telemetry_eki_xml.data()),
                           rsi_started_telemetry_eki_xml.size());
              bytes_read = rsi_started_telemetry_eki_xml.size();
            }
            return icon::OkStatus();
          });

  // Wait until the Mock EKI reports that RSI is running.
  while (!kuka_eki_system_control_->Telemetry().rsi_active) {
    absl::SleepFor(absl::Seconds(0.1));
  }
  EXPECT_OK(kuka_eki_system_control_->StopRSI());

  kuka_eki_system_control_
      .reset();  // This instance needs to be destroyed here so that it won't
                 // access the mock strings anymore.
}

TEST_F(KukaEkiSystemControlTest, ClearFaults) {
  std::string clear_faults_eki_command_xml =
      CreateEkiCommandXml(KukaEkiSystemControl::EkiOperation::kClearFaults);
  std::string default_eki_command_xml =
      CreateEkiCommandXml(KukaEkiSystemControl::EkiOperation::kNone);
  EXPECT_CALL(*tcp_client_,
              Send(absl::MakeSpan(reinterpret_cast<const uint8_t*>(
                                      default_eki_command_xml.data()),
                                  default_eki_command_xml.size()),
                   _))
      .WillRepeatedly(Return(icon::OkStatus()));
  auto startup_telemetry_eki_xml = CreateEkiTelemetryXmlChecked(
      KukaEkiSystemControl::EkiTelemetry{.op_mode = OpMode::kExternal});
  // Register the specific "StopProgram" call with a callback that tells us
  // that this call happened.
  bool command_sent = false;
  EXPECT_CALL(*tcp_client_,
              Send(absl::MakeSpan(reinterpret_cast<const uint8_t*>(
                                      clear_faults_eki_command_xml.data()),
                                  clear_faults_eki_command_xml.size()),
                   _))
      .WillOnce(::testing::DoAll([&command_sent] { command_sent = true; },
                                 Return(icon::OkStatus())));
  std::string faulted_robot_telemetry_eki_xml = CreateEkiTelemetryXmlChecked(
      KukaEkiSystemControl::EkiTelemetry{.op_mode = OpMode::kExternal,
                                         .rsi_active = true,
                                         .messages_present = true,
                                         .program_active = true});

  // Register a call that will return the "RSI active" telemetry data and will
  // return the "RSI stopped" data as soon as the "StopProgram" call was
  // registered.
  EXPECT_CALL(*tcp_client_, Receive(_, _, _, _, _))
      .WillRepeatedly([&command_sent, &faulted_robot_telemetry_eki_xml,
                       &startup_telemetry_eki_xml](
                          absl::Duration timeout, bool recv_until_buffer_full,
                          int recv_flags, absl::Span<uint8_t> buffer,
                          size_t& bytes_read) {
        CHECK(buffer.size() >= startup_telemetry_eki_xml.size());
        CHECK(buffer.size() >= faulted_robot_telemetry_eki_xml.size());
        // Check which data EKI would return.
        if (command_sent) {
          (void)memcpy(buffer.data(), (startup_telemetry_eki_xml.data()),
                       startup_telemetry_eki_xml.size());
          bytes_read = startup_telemetry_eki_xml.size();
        } else {
          (void)memcpy(buffer.data(), (faulted_robot_telemetry_eki_xml.data()),
                       faulted_robot_telemetry_eki_xml.size());
          bytes_read = faulted_robot_telemetry_eki_xml.size();
        }
        return icon::OkStatus();
      });

  // Wait until the Mock EKI reports that RSI is running.
  while (!kuka_eki_system_control_->Telemetry().rsi_active) {
    absl::SleepFor(absl::Seconds(0.1));
  }
  EXPECT_OK(kuka_eki_system_control_->ClearFaults());
  kuka_eki_system_control_
      .reset();  // This instance needs to be destroyed here so that it won't
                 // access the mock strings anymore.
}

struct ActiveErrorFlagsTestData {
  std::string test_name;
  std::string eki_telemetry;
  KukaErrorFlagsMask expected_error_flags;
};

class ActiveErrorFlagsTest
    : public KukaEkiSystemControlTest,
      public ::testing::WithParamInterface<ActiveErrorFlagsTestData> {};

TEST_P(ActiveErrorFlagsTest, CorrectActiveErrorFlags) {
  std::string default_eki_command_xml =
      CreateEkiCommandXml(KukaEkiSystemControl::EkiOperation::kNone);
  EXPECT_CALL(*tcp_client_,
              Send(absl::MakeSpan(reinterpret_cast<const uint8_t*>(
                                      default_eki_command_xml.data()),
                                  default_eki_command_xml.size()),
                   _))
      .WillRepeatedly(Return(icon::OkStatus()));

  bool reply_received = false;
  EXPECT_CALL(*tcp_client_, Receive(_, _, _, _, _))
      .WillRepeatedly([&reply_received, telemetry = GetParam().eki_telemetry](
                          absl::Duration timeout, bool recv_until_buffer_full,
                          int recv_flags, absl::Span<uint8_t> buffer,
                          size_t& bytes_read) {
        CHECK(buffer.size() >= telemetry.size());
        (void)memcpy(buffer.data(), (telemetry.data()), telemetry.size());
        reply_received = true;
        bytes_read = telemetry.size();
        return icon::OkStatus();
      });

  while (!reply_received) {
    absl::SleepFor(absl::Seconds(0.1));
  }
  EXPECT_THAT(kuka_eki_system_control_->ActiveErrorFlags(),
              IsOkAndHolds(GetParam().expected_error_flags));
  kuka_eki_system_control_
      .reset();  // This instance needs to be destroyed here so that it won't
                 // access the mock strings anymore.
}

INSTANTIATE_TEST_SUITE_P(
    ActiveErrorFlagsTest, ActiveErrorFlagsTest,
    ::testing::Values(
        ActiveErrorFlagsTestData{.test_name = "normal_operation",
                                 .eki_telemetry = CreateEkiTelemetryXmlChecked(
                                     KukaEkiSystemControl::EkiTelemetry{
                                         .op_mode = OpMode::kExternal}),
                                 .expected_error_flags = KukaErrorFlagsMask{}},
        ActiveErrorFlagsTestData{
            .test_name = "messages_present",
            .eki_telemetry =
                CreateEkiTelemetryXmlChecked(KukaEkiSystemControl::EkiTelemetry{
                    .op_mode = OpMode::kExternal,
                    .messages_present = true,
                }),
            .expected_error_flags =
                {}  // Since the KUKA system variables $STOPMESS is only cleared
                    // when the $DRIVES_ON is set, there are many false
                    // positives if we report that messages are present.
                    // Therefore, `messages_present` is only checked when RSI is
                    // getting started.
        }),
    [](const ::testing::TestParamInfo<ActiveErrorFlagsTest::ParamType>& info) {
      return info.param.test_name;
    });

}  // namespace intrinsic::kuka
