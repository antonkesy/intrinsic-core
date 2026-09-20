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

#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_hardware_module.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/status_matchers.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "grpcpp/server_builder.h"
#include "internal/testing.h"
#include "intrinsic/icon/hal/default_hardware_interfaces.h"  // IWYU pragma: keep
#include "intrinsic/icon/hal/get_hardware_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_module_interface.h"
#include "intrinsic/icon/hal/hardware_module_proxy.h"
#include "intrinsic/icon/hal/hardware_module_runtime.h"
#include "intrinsic/icon/hal/icon_state_register.h"
#include "intrinsic/icon/hal/interfaces/adio.fbs.h"
#include "intrinsic/icon/hal/interfaces/icon_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/io_controller.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/hal/module_config.h"
#include "intrinsic/icon/hal/proto/hardware_module_config.pb.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi.pb.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/mock_kuka_rsi_client.h"
#include "intrinsic/icon/interprocess/shared_memory_manager/memory_segment.h"
#include "intrinsic/icon/interprocess/shared_memory_manager/segment_info.fbs.h"
#include "intrinsic/icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "intrinsic/icon/interprocess/shared_memory_manager/testing/unique_segment_name.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/current_cycle.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_matchers.h"
#include "intrinsic/kinematics/types/joint_limits.pb.h"
#include "intrinsic/util/path_resolver/path_resolver.h"
#include "ortools/base/helpers.h"
#include "ortools/base/options.h"

namespace intrinsic::icon {
namespace {

using ::absl_testing::IsOkAndHolds;
using ::absl_testing::StatusIs;
using kuka::testing::MockKukaRsiClient;
using ::testing::_;
using ::testing::DoubleEq;
using ::testing::DoubleNear;
using ::testing::Each;
using ::testing::ElementsAre;
using ::testing::HasSubstr;
using ::testing::IsSupersetOf;
using ::testing::NiceMock;
using ::testing::Pointwise;
using ::testing::Return;

class RsiHardwareModuleTest : public ::testing::Test {
 public:
  void SetUp() override {
    auto mock_client = std::make_unique<NiceMock<MockKukaRsiClient>>();
    mock_client_ = mock_client.get();
    sample_telemetry_.digital_input = {false, false, false, false};
    sample_telemetry_.digital_output = {false, false};
    sample_telemetry_.position = {1, 2, 3, 4, 5, 6};
    ON_CALL(*mock_client_, IsActive()).WillByDefault(Return(true));

    ON_CALL(*mock_client_,
            SendCommand(ElementsAre(1, 2, 3, 4, 5, 6), ElementsAre()))
        .WillByDefault(Return(OkStatus()));

    ON_CALL(*mock_client_, GetTelemetry())
        .WillByDefault(Return(sample_telemetry_));

    EXPECT_CALL(*mock_client_, Init(_)).WillOnce(Return(absl::OkStatus()));
    ON_CALL(*mock_client_, StartRSI(_)).WillByDefault(Return(absl::OkStatus()));
    ON_CALL(*mock_client_, StopRSI()).WillByDefault(Return(absl::OkStatus()));
    ON_CALL(*mock_client_, StopRSI()).WillByDefault(Return(absl::OkStatus()));

    intrinsic_proto::icon::HardwareModuleConfig config;
    CHECK_OK(file::GetTextProto(
        PathResolver::ResolveRunfilesPathForTest(
            "intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi/"
            "kuka_rsi_module_test_config.pbtxt"),
        &config, file::Defaults()));

    hwm_name_ = config.name();

    auto hwm = std::make_unique<KukaRsiHwModule>(std::move(mock_client));

    ASSERT_OK_AND_ASSIGN(auto shm_manager,
                         intrinsic::icon::SharedMemoryManager::Create(
                             memory_namespace_, hwm_name_));

    ASSERT_OK_AND_ASSIGN(auto runtime,
                         HardwareModuleRuntime::Create(
                             std::move(shm_manager),
                             {
                                 /*realtime_clock=*/nullptr,
                                 std::move(hwm),
                                 ModuleConfig(config, memory_namespace_,
                                              /*realtime_clock=*/nullptr),
                             }));

    runtime_ = std::move(runtime);

    grpc::ServerBuilder builder;
    ASSERT_OK(runtime_->Run(builder));

    ASSERT_OK_AND_ASSIGN(
        auto proxy, intrinsic::icon::HardwareModuleProxy::Attach(
                        memory_namespace_, hwm_name_, absl::ZeroDuration()));
    proxy_ = std::make_unique<HardwareModuleProxy>(std::move(proxy));

    ASSERT_OK_AND_ASSIGN(
        mutable_icon_state_handle_,
        proxy_->GetMutableHardwareInterface<intrinsic_fbs::IconState>(
            kIconStateInterfaceName));
  }

  // Updates `cycle` of the ICON state.
  void SetIconCycle(uint64_t cycle) {
    icon::Cycle::SetCurrentCycle(cycle);
    mutable_icon_state_handle_->mutate_current_cycle(cycle);
    mutable_icon_state_handle_.UpdatedAt(intrinsic::Clock::now());
  }

  // Tears down the test fixture.
  void TearDown() override {}

 protected:
  std::string memory_namespace_ = UniqueMemoryNamespace();
  std::string hwm_name_;
  kuka::RsiTelemetry sample_telemetry_;
  NiceMock<MockKukaRsiClient>* mock_client_;
  std::unique_ptr<HardwareModuleRuntime> runtime_;
  MutableHardwareInterfaceHandle<intrinsic_fbs::IconState>
      mutable_icon_state_handle_;

  std::unique_ptr<HardwareModuleProxy> proxy_;
};

TEST_F(RsiHardwareModuleTest, HardwareModuleRuntimeRunsSuccessfully) {}

TEST_F(RsiHardwareModuleTest, InterfacesAreCorrectlyAdvertised) {
  ASSERT_OK_AND_ASSIGN(
      auto hardware_info,
      proxy_->GetReadOnlyMemorySegment<intrinsic_fbs::SegmentInfo>(
          hal::kModuleInfoName));

  EXPECT_THAT(GetInterfacesFromModuleInfo(hardware_info.GetValue()),
              IsOkAndHolds(IsSupersetOf(
                  {"joint_position_command", "joint_position_state",
                   "joint_velocity_state", "hardware_module_state",
                   "joint_acceleration_state", "joint_torque_state"})));
}

TEST_F(RsiHardwareModuleTest, RejectsApplyCommandIfNotActive) {
  auto& hw_module = *runtime_->GetHardwareModule().instance;

  // ReadStatus should only work if RSI is active.
  EXPECT_CALL(*mock_client_, IsActive()).WillOnce(Return(false));

  EXPECT_THAT(hw_module.ApplyCommand(),
              StatusIs(absl::StatusCode::kUnavailable));
}

TEST_F(RsiHardwareModuleTest, ApplyCommandFailsWithWrongIconCycle) {
  auto& hw_module = *runtime_->GetHardwareModule().instance;

  EXPECT_CALL(*mock_client_, IsActive()).WillOnce(Return(true));
  EXPECT_OK(hw_module.Activate());
  // Explicitly sets a wrong ICON cycle.
  SetIconCycle(42);
  EXPECT_THAT(hw_module.ApplyCommand(),
              RealtimeStatusIs(absl::StatusCode::kFailedPrecondition));
}

TEST_F(RsiHardwareModuleTest, ReportsAndAcceptsPositions) {
  auto& hw_module = *runtime_->GetHardwareModule().instance;
  kuka::RsiTelemetry sample_telemetry_;
  sample_telemetry_.position = {1, 2, 3, 4, 5, 6};
  EXPECT_CALL(*mock_client_, GetTelemetry())
      .WillOnce(Return(sample_telemetry_));

  EXPECT_CALL(*mock_client_, IsActive()).WillOnce(Return(true));
  EXPECT_OK(hw_module.ReadStatus());
  ASSERT_OK_AND_ASSIGN(
      auto joint_position_state,
      proxy_->GetHardwareInterface<intrinsic_fbs::JointPositionState>(
          "joint_position_state"));

  EXPECT_THAT(*joint_position_state->position(),
              Pointwise(DoubleEq(), sample_telemetry_.position));

  // Set the joint position command as the sensed position and check that the
  // correct value is sent.
  ASSERT_OK_AND_ASSIGN(
      auto joint_position_command,
      proxy_->GetMutableHardwareInterface<intrinsic_fbs::JointPositionCommand>(
          "joint_position_command"));
  // Setting the ICON cycle to zero, because that is the initial value of the
  // command.
  SetIconCycle(0);

  for (int i = 0; i < joint_position_command->mutable_position()->size(); ++i) {
    joint_position_command->mutable_position()->Mutate(
        i, joint_position_state->position()->Get(i));
  }
  EXPECT_OK(hw_module.Activate());

  EXPECT_CALL(*mock_client_,
              SendCommand(Pointwise(DoubleEq(), sample_telemetry_.position), _))
      .WillOnce(Return(OkStatus()));
  EXPECT_OK(hw_module.ApplyCommand());
}

TEST_F(RsiHardwareModuleTest, ReportsTorques) {
  auto& hw_module = *runtime_->GetHardwareModule().instance;
  kuka::RsiTelemetry sample_telemetry_;
  sample_telemetry_.torque = {-1, -2, -3, -4, -5, -6};
  EXPECT_CALL(*mock_client_, GetTelemetry())
      .WillOnce(Return(sample_telemetry_));
  EXPECT_OK(hw_module.ReadStatus());

  ASSERT_OK_AND_ASSIGN(
      auto torque_state,
      proxy_->GetHardwareInterface<intrinsic_fbs::JointTorqueState>(
          "joint_torque_state"));
  EXPECT_THAT(*torque_state->torque(),
              Pointwise(DoubleEq(), sample_telemetry_.torque));
}

TEST_F(RsiHardwareModuleTest, ReportsCorrectVelocity) {
  auto& hw_module = *runtime_->GetHardwareModule().instance;

  kuka::RsiTelemetry sample_telemetry_;
  sample_telemetry_.position = {1, 2, 3, 4, 5, 6};

  constexpr int iterations = 100;
  constexpr double kTimestepSec = 0.01;
  constexpr double kExpectedVelocity = 1.0;
  for (int i = 0; i < iterations; ++i) {
    sample_telemetry_.interpolator_counter_ms += kTimestepSec * 1e3;
    for (int j = 0; j < sample_telemetry_.position.size(); ++j) {
      sample_telemetry_.position[j] += kExpectedVelocity * kTimestepSec;
    }
    LOG(INFO) << sample_telemetry_.interpolator_counter_ms;
    EXPECT_CALL(*mock_client_, GetTelemetry())
        .WillOnce(Return(sample_telemetry_));

    EXPECT_OK(hw_module.ReadStatus());
  }
  ASSERT_OK_AND_ASSIGN(
      auto joint_velocity_state,
      proxy_->GetHardwareInterface<intrinsic_fbs::JointVelocityState>(
          "joint_velocity_state"));
  EXPECT_THAT(*joint_velocity_state->velocity(),
              Each(DoubleNear(kExpectedVelocity, 1e-3)));
}

MATCHER(EqualDIO, "") {
  *result_listener << absl::StrCat("Values are ", std::get<0>(arg)->value(),
                                   " and ", std::get<1>(arg));
  return std::get<0>(arg)->value() == (std::get<1>(arg));
}

TEST_F(RsiHardwareModuleTest, ReportsDigitalInputsAndOutputs) {
  EXPECT_CALL(*mock_client_, IsEnabled()).WillRepeatedly(Return(true));
  auto& hw_module = *runtime_->GetHardwareModule().instance;

  sample_telemetry_.digital_input = {true, false, true, false};
  sample_telemetry_.digital_output = {false, true};
  EXPECT_CALL(*mock_client_, GetTelemetry())
      .WillOnce(Return(sample_telemetry_));
  EXPECT_OK(hw_module.ReadStatus());

  ASSERT_OK_AND_ASSIGN(auto inputs,
                       proxy_->GetHardwareInterface<intrinsic_fbs::DIOStatus>(
                           "digital_input_status"));

  EXPECT_THAT(*inputs->signals(),
              Pointwise(EqualDIO(), sample_telemetry_.digital_input));

  ASSERT_OK_AND_ASSIGN(auto outputs,
                       proxy_->GetHardwareInterface<intrinsic_fbs::DIOStatus>(
                           "digital_output_status"));
  EXPECT_THAT(*outputs->signals(),
              Pointwise(EqualDIO(), sample_telemetry_.digital_output));
}

TEST_F(RsiHardwareModuleTest, ReceivedTooManyDigitalInputs) {
  EXPECT_CALL(*mock_client_, IsEnabled()).WillRepeatedly(Return(true));
  auto& hw_module = *runtime_->GetHardwareModule().instance;

  sample_telemetry_.digital_input = {true, false, true, false, false};
  sample_telemetry_.digital_output = {false, true};
  EXPECT_CALL(*mock_client_, GetTelemetry())
      .WillOnce(Return(sample_telemetry_));
  EXPECT_THAT(hw_module.ReadStatus(),
              StatusIs(absl::StatusCode::kFailedPrecondition,
                       HasSubstr("digital input size does not match")));
}

TEST_F(RsiHardwareModuleTest, ReceivedTooManyDigitalOutputs) {
  EXPECT_CALL(*mock_client_, IsEnabled()).WillRepeatedly(Return(true));
  auto& hw_module = *runtime_->GetHardwareModule().instance;

  sample_telemetry_.digital_input = {true, false, true, false};
  sample_telemetry_.digital_output = {false, true, false};
  EXPECT_CALL(*mock_client_, GetTelemetry())
      .WillOnce(Return(sample_telemetry_));
  EXPECT_THAT(hw_module.ReadStatus(),
              StatusIs(absl::StatusCode::kFailedPrecondition,
                       HasSubstr("digital output size does not match")));
}

TEST_F(RsiHardwareModuleTest, ForwardsDigitalOutputs) {
  EXPECT_CALL(*mock_client_, IsEnabled()).WillRepeatedly(Return(true));
  auto& hw_module = *runtime_->GetHardwareModule().instance;

  ASSERT_OK_AND_ASSIGN(
      auto outputs,
      proxy_->GetMutableHardwareInterface<intrinsic_fbs::DIOCommand>(
          "digital_output_command"));
  outputs->mutable_signals()->GetMutableObject(0)->mutate_value(false);
  outputs->mutable_signals()->GetMutableObject(1)->mutate_value(true);

  EXPECT_CALL(*mock_client_, SendCommand(_, ElementsAre(false, true)))
      .WillOnce(Return(OkStatus()));
  // Setting the ICON cycle to zero, because that is the initial value of the
  // command.
  SetIconCycle(0);

  EXPECT_OK(hw_module.ApplyCommand());
}

}  // namespace
}  // namespace intrinsic::icon
