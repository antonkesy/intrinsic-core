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

#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_plc_system_control.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/status_matchers.h"
#include "absl/time/time.h"
#include "internal/testing.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_plc_client.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/mock_kuka_plc_client.h"

namespace intrinsic::kuka {
namespace {
using ::absl_testing::IsOkAndHolds;
using ::absl_testing::StatusIs;
using ::testing::_;
using ::testing::Return;

class KukaPlcSystemControlTest : public ::testing::Test {
  void SetUp() override {
    auto mock_plc_client = std::make_unique<testing::MockKukaPlcClient>();
    mock_plc_client_ = mock_plc_client.get();
    system_control_ =
        std::make_unique<KukaPlcSystemControl>(std::move(mock_plc_client));
  }

 public:
  std::unique_ptr<KukaPlcSystemControl> system_control_;
  testing::MockKukaPlcClient* mock_plc_client_;
};

TEST_F(KukaPlcSystemControlTest, CompatibleOperationModes) {
  EXPECT_THAT(system_control_->CompatibleOperationModes(),
              ::testing::UnorderedElementsAreArray({OpMode::kExternal}));
}

TEST_F(KukaPlcSystemControlTest, StartRSI) {
  EXPECT_CALL(*mock_plc_client_, getState())
      .WillOnce(Return(KukaPlcState::kReady));
  EXPECT_CALL(*mock_plc_client_, RequestStartRSI())
      .WillOnce(Return(absl::OkStatus()));
  EXPECT_CALL(*mock_plc_client_, WaitForState(KukaPlcState::kRunning, _, _))
      .WillOnce(Return(absl::OkStatus()));

  EXPECT_OK(system_control_->StartRSI(absl::Seconds(1)));
}

TEST_F(KukaPlcSystemControlTest, StartRSIWhenAlreadyRunning) {
  EXPECT_CALL(*mock_plc_client_, getState())
      .WillOnce(Return(KukaPlcState::kRunning));

  EXPECT_OK(system_control_->StartRSI(absl::Seconds(1)));
}

TEST_F(KukaPlcSystemControlTest, StartRSIWhenEStopActive) {
  EXPECT_CALL(*mock_plc_client_, getState())
      .WillRepeatedly(Return(KukaPlcState::kEStop));

  EXPECT_THAT(system_control_->StartRSI(absl::Seconds(1)),
              StatusIs(absl::StatusCode::kFailedPrecondition));
}

TEST_F(KukaPlcSystemControlTest, StartRSIWhenUnknownState) {
  EXPECT_CALL(*mock_plc_client_, getState())
      .WillOnce(Return(static_cast<KukaPlcState>(-1)));
  EXPECT_CALL(*mock_plc_client_, getStateAsString()).WillOnce(Return(""));
  EXPECT_THAT(system_control_->StartRSI(absl::Seconds(1)),
              StatusIs(absl::StatusCode::kFailedPrecondition));
}

TEST_F(KukaPlcSystemControlTest, StartRSIWhenMessagesPresent) {
  EXPECT_CALL(*mock_plc_client_, getState())
      .WillOnce(Return(KukaPlcState::kStopMessagesActive));

  EXPECT_CALL(*mock_plc_client_, RequestAcknowledgeErrors())
      .WillOnce(Return(absl::OkStatus()));
  EXPECT_CALL(*mock_plc_client_, RequestStartRSI())
      .WillOnce(Return(absl::OkStatus()));
  EXPECT_CALL(*mock_plc_client_, WaitForState(KukaPlcState::kReady, _, _))
      .WillOnce(Return(absl::OkStatus()));
  EXPECT_CALL(*mock_plc_client_, WaitForState(KukaPlcState::kRunning, _, _))
      .WillOnce(Return(absl::OkStatus()));

  EXPECT_OK(system_control_->StartRSI(absl::Seconds(1)));
}

TEST_F(KukaPlcSystemControlTest, StartRSIWithClosedConnection) {
  EXPECT_CALL(*mock_plc_client_, getState())
      .WillOnce(Return(absl::UnavailableError("")))
      .WillOnce(Return(KukaPlcState::kReady));
  EXPECT_CALL(*mock_plc_client_, RequestStartRSI())
      .WillOnce(Return(absl::OkStatus()));
  EXPECT_CALL(*mock_plc_client_, WaitForState(KukaPlcState::kRunning, _, _))
      .WillOnce(Return(absl::OkStatus()));

  EXPECT_OK(system_control_->StartRSI(absl::Seconds(1)));
}

TEST_F(KukaPlcSystemControlTest, StopRSI) {
  EXPECT_CALL(*mock_plc_client_, getState())
      .WillOnce(Return(KukaPlcState::kRunning));
  EXPECT_CALL(*mock_plc_client_, RequestStopRSI())
      .WillOnce(Return(absl::OkStatus()));
  EXPECT_CALL(*mock_plc_client_, WaitForState(KukaPlcState::kReady, _, _))
      .WillOnce(Return(absl::OkStatus()));

  EXPECT_OK(system_control_->StopRSI());
}

TEST_F(KukaPlcSystemControlTest, StopRSIWithClosedConnection) {
  EXPECT_CALL(*mock_plc_client_, getState())
      .Times(2)
      .WillRepeatedly(Return(KukaPlcState::kRunning));
  EXPECT_CALL(*mock_plc_client_, RequestStopRSI())
      .WillOnce(Return(absl::UnavailableError("")))
      .WillOnce(Return(absl::OkStatus()));
  EXPECT_CALL(*mock_plc_client_, WaitForState(KukaPlcState::kReady, _, _))
      .WillOnce(Return(absl::OkStatus()));

  EXPECT_OK(system_control_->StopRSI());
}

TEST_F(KukaPlcSystemControlTest, StopRSIWhenAlreadyStopped) {
  EXPECT_CALL(*mock_plc_client_, getState())
      .WillOnce(Return(KukaPlcState::kReady));

  EXPECT_OK(system_control_->StopRSI());
}

TEST_F(KukaPlcSystemControlTest, EStopActiveErrorFlag) {
  EXPECT_CALL(*mock_plc_client_, getState())
      .WillOnce(Return(KukaPlcState::kEStop));

  EXPECT_THAT(
      system_control_->ActiveErrorFlags(),
      IsOkAndHolds(KukaErrorFlagsMask::kEmergencyStopActiveOrWrongOpMode));
}

TEST_F(KukaPlcSystemControlTest, MessagesPresentActiveErrorFlag) {
  EXPECT_CALL(*mock_plc_client_, getState())
      .WillOnce(Return(KukaPlcState::kStopMessagesActive));

  EXPECT_THAT(system_control_->ActiveErrorFlags(),
              IsOkAndHolds(KukaErrorFlagsMask::kMessagesPresent));
}

TEST_F(KukaPlcSystemControlTest, NoActiveErrorFlag) {
  EXPECT_CALL(*mock_plc_client_, getState())
      .WillOnce(Return(KukaPlcState::kRunning));

  EXPECT_THAT(system_control_->ActiveErrorFlags(),
              IsOkAndHolds(KukaErrorFlagsMask::kNone));
}

TEST_F(KukaPlcSystemControlTest,
       MessagesPresentActiveErrorFlagWithClosedConnection) {
  EXPECT_CALL(*mock_plc_client_, getState())
      .WillOnce(Return(absl::UnavailableError("")))
      .WillOnce(Return(KukaPlcState::kStopMessagesActive));

  EXPECT_THAT(system_control_->ActiveErrorFlags(),
              IsOkAndHolds(KukaErrorFlagsMask::kMessagesPresent));
}

}  // namespace
}  // namespace intrinsic::kuka
