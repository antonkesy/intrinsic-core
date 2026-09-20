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

#include "intrinsic/hardware/gripper/eoat/gripper_utils.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <optional>
#include <string>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/status_matchers.h"
#include "internal/testing.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.pb.h"
#include "intrinsic/hardware/gpio/v1/signal.pb.h"
#include "intrinsic/hardware/gripper/eoat/eoat_service.pb.h"
#include "intrinsic/hardware/gripper/eoat/gripper_config.pb.h"

using ::intrinsic_proto::eoat::GpioServiceEndpoint;
using ::intrinsic_proto::eoat::PinchGripperConfig;
using ::intrinsic_proto::eoat::SuctionGripperConfig;
using ::intrinsic_proto::gpio::v1::GetSignalDescriptionsResponse;
using ::intrinsic_proto::gpio::v1::SignalDescription;
using ::intrinsic_proto::gpio::v1::SignalValue;

using ::absl_testing::StatusIs;
using ::testing::HasSubstr;

static SignalValue TrueValue() {
  SignalValue value;
  value.set_bool_value(true);
  return value;
}

static SignalValue FalseValue() {
  SignalValue value;
  value.set_bool_value(false);
  return value;
}

TEST(GripperUtils, CheckConfigValidity) {
  using intrinsic::gripper::GpioSrvEndpointIsValid;

  GpioServiceEndpoint config;
  EXPECT_THAT(GpioSrvEndpointIsValid(config),
              StatusIs(absl::StatusCode::kInvalidArgument));

  config.set_grpc_address("add:1234");
  EXPECT_THAT(GpioSrvEndpointIsValid(config),
              StatusIs(absl::StatusCode::kInvalidArgument));

  config.set_gpio_service_name("gpio-service");
  EXPECT_OK(GpioSrvEndpointIsValid(config));

  config.clear_grpc_address();
  EXPECT_THAT(GpioSrvEndpointIsValid(config),
              StatusIs(absl::StatusCode::kInvalidArgument));
}

TEST(PinchGripperUtils, CheckConfigValidity) {
  using ::intrinsic::gripper::GripperConfigIsValid;

  PinchGripperConfig config;
  EXPECT_FALSE(GripperConfigIsValid(config).ok());

  config.mutable_grasp()->mutable_value_set()->mutable_values()->insert(
      {"foo", TrueValue()});
  EXPECT_FALSE(GripperConfigIsValid(config).ok());

  config.mutable_release()->mutable_value_set()->mutable_values()->insert(
      {"bar", FalseValue()});
  EXPECT_FALSE(GripperConfigIsValid(config).ok());

  config.mutable_gripping_indicated()
      ->mutable_value_set()
      ->mutable_values()
      ->insert({"baz", FalseValue()});
  EXPECT_TRUE(GripperConfigIsValid(config).ok());

  config.mutable_grasp()->mutable_value_set()->mutable_values()->clear();
  EXPECT_FALSE(GripperConfigIsValid(config).ok());

  config.mutable_release()->mutable_value_set()->mutable_values()->clear();
  EXPECT_FALSE(GripperConfigIsValid(config).ok());
}

TEST(PinchGripperUtils, SignalsToClaimEmpty) {
  PinchGripperConfig config;

  auto null_set = ::intrinsic::gripper::SignalsToClaim(config);
  EXPECT_TRUE(null_set.empty());
}

TEST(PinchGripperUtils, SignalsToClaimNonEmpty) {
  PinchGripperConfig config;

  config.mutable_grasp()->mutable_value_set()->mutable_values()->insert(
      {"portG", TrueValue()});
  config.mutable_release()->mutable_value_set()->mutable_values()->insert(
      {"portR", FalseValue()});
  config.mutable_gripping_indicated()
      ->mutable_value_set()
      ->mutable_values()
      ->insert({"portGI", FalseValue()});

  auto all_write_signals = ::intrinsic::gripper::SignalsToClaim(config);
  EXPECT_EQ(all_write_signals,
            absl::flat_hash_set<std::string>({"portR", "portG"}));
}

TEST(PinchGripperUtils, SignalsToReadEmpty) {
  PinchGripperConfig config;

  auto null_set = ::intrinsic::gripper::SignalsToRead(config);
  EXPECT_TRUE(null_set.empty());
}

TEST(PinchGripperUtils, SignalsToReadNonEmpty) {
  PinchGripperConfig config;

  config.mutable_grasp()->mutable_value_set()->mutable_values()->insert(
      {"portG", TrueValue()});
  config.mutable_release()->mutable_value_set()->mutable_values()->insert(
      {"portR", FalseValue()});
  config.mutable_gripping_indicated()
      ->mutable_value_set()
      ->mutable_values()
      ->insert({"portGI.0", FalseValue()});
  config.mutable_gripping_indicated()
      ->mutable_value_set()
      ->mutable_values()
      ->insert({"portGI.1", TrueValue()});

  auto all_write_signals = ::intrinsic::gripper::SignalsToRead(config);
  EXPECT_EQ(all_write_signals,
            absl::flat_hash_set<std::string>({"portGI.0", "portGI.1"}));
}

TEST(SuctionGripperUtils, CheckConfigValidity) {
  using ::intrinsic::gripper::GripperConfigIsValid;

  SuctionGripperConfig config;
  EXPECT_FALSE(GripperConfigIsValid(config).ok());

  config.mutable_grasp()->mutable_value_set()->mutable_values()->insert(
      {"foo", TrueValue()});
  EXPECT_FALSE(GripperConfigIsValid(config).ok());

  config.mutable_release()->mutable_value_set()->mutable_values()->insert(
      {"bar", FalseValue()});
  EXPECT_FALSE(GripperConfigIsValid(config).ok());

  config.mutable_blowoff_on()->mutable_value_set()->mutable_values()->insert(
      {"barbaz", TrueValue()});
  EXPECT_FALSE(GripperConfigIsValid(config).ok());

  config.mutable_blowoff_off()->mutable_value_set()->mutable_values()->insert(
      {"barbazbaz", FalseValue()});
  EXPECT_FALSE(GripperConfigIsValid(config).ok());

  config.mutable_gripping_indicated()
      ->mutable_value_set()
      ->mutable_values()
      ->insert({"baz", FalseValue()});
  EXPECT_TRUE(GripperConfigIsValid(config).ok());

  config.mutable_grasp()->mutable_value_set()->mutable_values()->clear();
  EXPECT_FALSE(GripperConfigIsValid(config).ok());

  config.mutable_release()->mutable_value_set()->mutable_values()->clear();
  EXPECT_FALSE(GripperConfigIsValid(config).ok());
}

TEST(SuctionGripperUtils, SignalsToClaimEmpty) {
  SuctionGripperConfig config;

  auto null_set = ::intrinsic::gripper::SignalsToClaim(config);
  EXPECT_TRUE(null_set.empty());
}

TEST(SuctionGripperUtils, SignalsToClaimNonEmpty) {
  SuctionGripperConfig config;

  config.mutable_grasp()->mutable_value_set()->mutable_values()->insert(
      {"portG", TrueValue()});
  config.mutable_release()->mutable_value_set()->mutable_values()->insert(
      {"portR", FalseValue()});
  config.mutable_blowoff_on()->mutable_value_set()->mutable_values()->insert(
      {"portB", FalseValue()});
  config.mutable_blowoff_off()->mutable_value_set()->mutable_values()->insert(
      {"portB", TrueValue()});
  config.mutable_blowoff_off()->mutable_value_set()->mutable_values()->insert(
      {"portS", TrueValue()});
  config.mutable_gripping_indicated()
      ->mutable_value_set()
      ->mutable_values()
      ->insert({"portGI", FalseValue()});

  auto all_write_signals = ::intrinsic::gripper::SignalsToClaim(config);
  EXPECT_EQ(all_write_signals, absl::flat_hash_set<std::string>(
                                   {"portR", "portG", "portB", "portS"}));
}

TEST(SuctionGripperUtils, SignalsToReadEmpty) {
  SuctionGripperConfig config;

  auto null_set = ::intrinsic::gripper::SignalsToRead(config);
  EXPECT_TRUE(null_set.empty());
}

TEST(SuctionGripperUtils, SignalsToReadNonEmpty) {
  SuctionGripperConfig config;

  config.mutable_grasp()->mutable_value_set()->mutable_values()->insert(
      {"portG", TrueValue()});
  config.mutable_release()->mutable_value_set()->mutable_values()->insert(
      {"portR", FalseValue()});
  config.mutable_blowoff_on()->mutable_value_set()->mutable_values()->insert(
      {"portB", FalseValue()});
  config.mutable_blowoff_off()->mutable_value_set()->mutable_values()->insert(
      {"portB", TrueValue()});
  config.mutable_blowoff_off()->mutable_value_set()->mutable_values()->insert(
      {"portS", TrueValue()});
  config.mutable_gripping_indicated()
      ->mutable_value_set()
      ->mutable_values()
      ->insert({"portIndicated.0", FalseValue()});
  config.mutable_gripping_indicated()
      ->mutable_value_set()
      ->mutable_values()
      ->insert({"portIndicated.2", FalseValue()});

  auto all_write_signals = ::intrinsic::gripper::SignalsToRead(config);
  EXPECT_EQ(all_write_signals, absl::flat_hash_set<std::string>(
                                   {"portIndicated.0", "portIndicated.2"}));
}

TEST(SuctionGripperUtils, EmptyishSignalsAreValid) {
  SuctionGripperConfig config;
  GetSignalDescriptionsResponse valid_signals;
  EXPECT_OK(::intrinsic::gripper::SignalsAreValid(config, valid_signals));

  const std::string claimed_signal = "portGrasp";
  config.mutable_grasp()->mutable_value_set()->mutable_values()->insert(
      {claimed_signal, TrueValue()});
  EXPECT_THAT(::intrinsic::gripper::SignalsAreValid(config, valid_signals),
              StatusIs(absl::StatusCode::kInvalidArgument,
                       "Invalid write signal name(s) provided: portGrasp. "
                       "Valid signal names are: ."));

  SignalDescription signal_desc;
  signal_desc.set_signal_name(claimed_signal);
  signal_desc.set_can_write(true);
  *valid_signals.add_signal_descriptions() = signal_desc;
  EXPECT_OK(::intrinsic::gripper::SignalsAreValid(config, valid_signals));
}

static void AddReadSignal(
    GetSignalDescriptionsResponse& valid_signals, const std::string& name,
    std::optional<std::string> alternate_name = std::nullopt) {
  SignalDescription signal_desc;
  signal_desc.set_signal_name(name);
  signal_desc.set_can_read(true);
  if (alternate_name.has_value()) {
    signal_desc.add_alternate_signal_names(*alternate_name);
  }
  *valid_signals.add_signal_descriptions() = signal_desc;
}

static void AddWriteSignal(
    GetSignalDescriptionsResponse& valid_signals, const std::string& name,
    std::optional<std::string> alternate_name = std::nullopt) {
  SignalDescription signal_desc;
  signal_desc.set_signal_name(name);
  signal_desc.set_can_write(true);
  if (alternate_name.has_value()) {
    signal_desc.add_alternate_signal_names(*alternate_name);
  }
  *valid_signals.add_signal_descriptions() = signal_desc;
}

TEST(SuctionGripperUtils, SignalsAreValid) {
  SuctionGripperConfig config;

  config.mutable_grasp()->mutable_value_set()->mutable_values()->insert(
      {"portG", TrueValue()});
  config.mutable_release()->mutable_value_set()->mutable_values()->insert(
      {"portR", FalseValue()});
  config.mutable_blowoff_on()->mutable_value_set()->mutable_values()->insert(
      {"portB", FalseValue()});
  config.mutable_blowoff_off()->mutable_value_set()->mutable_values()->insert(
      {"portB", TrueValue()});
  config.mutable_blowoff_off()->mutable_value_set()->mutable_values()->insert(
      {"portS", TrueValue()});
  config.mutable_gripping_indicated()
      ->mutable_value_set()
      ->mutable_values()
      ->insert({"portIndicated.0", FalseValue()});
  config.mutable_gripping_indicated()
      ->mutable_value_set()
      ->mutable_values()
      ->insert({"portIndicated.2", FalseValue()});

  GetSignalDescriptionsResponse valid_signals;
  EXPECT_THAT(::intrinsic::gripper::SignalsAreValid(config, valid_signals),
              StatusIs(absl::StatusCode::kInvalidArgument));
  EXPECT_THAT(::intrinsic::gripper::SignalsAreValid(config, valid_signals),
              StatusIs(absl::StatusCode::kInvalidArgument,
                       "Invalid read signal name(s) provided: portIndicated.0, "
                       "portIndicated.2. Valid signal names are: . Invalid "
                       "write signal name(s) provided: portB, portG, portR, "
                       "portS. Valid signal names are: ."));

  // Adds one of the read signals.
  AddReadSignal(valid_signals, "portIndicated.0");
  EXPECT_THAT(
      ::intrinsic::gripper::SignalsAreValid(config, valid_signals),
      StatusIs(
          absl::StatusCode::kInvalidArgument,
          "Invalid read signal name(s) provided: portIndicated.2. Valid signal "
          "names are: portIndicated.0. Invalid write signal name(s) provided: "
          "portB, portG, portR, portS. Valid signal names are: ."));

  // Adds all the write signals except `portS`.
  for (const auto name : {"portG", "portR", "portB"}) {
    AddWriteSignal(valid_signals, name);
    EXPECT_THAT(::intrinsic::gripper::SignalsAreValid(config, valid_signals),
                StatusIs(absl::StatusCode::kInvalidArgument));
  }

  // Add the final read signal.
  AddReadSignal(valid_signals, "portIndicated.2");
  EXPECT_THAT(::intrinsic::gripper::SignalsAreValid(config, valid_signals),
              StatusIs(absl::StatusCode::kInvalidArgument,
                       "Invalid write signal name(s) provided: portS. Valid "
                       "signal names are: portB, portG, portR."));

  // Adds the final write signal.
  AddWriteSignal(valid_signals, "portS");
  EXPECT_OK(::intrinsic::gripper::SignalsAreValid(config, valid_signals));

  config.clear_grasp();
  EXPECT_OK(::intrinsic::gripper::SignalsAreValid(config, valid_signals));

  config.clear_release();
  EXPECT_OK(::intrinsic::gripper::SignalsAreValid(config, valid_signals));

  // There can be more valid signals than present in `config`.
  AddReadSignal(valid_signals, "some_read_signal_not_present_in_config");
  EXPECT_OK(::intrinsic::gripper::SignalsAreValid(config, valid_signals));
  AddWriteSignal(valid_signals, "some_write_signal_not_present_in_config");
  EXPECT_OK(::intrinsic::gripper::SignalsAreValid(config, valid_signals));
}

TEST(SuctionGripperUtils, AlternateSignalNamesAreValid) {
  SuctionGripperConfig config;

  config.mutable_grasp()->mutable_value_set()->mutable_values()->insert(
      {"alternate.portG", TrueValue()});
  config.mutable_grasp()->mutable_value_set()->mutable_values()->insert(
      {"alternate.portR", TrueValue()});
  config.mutable_gripping_indicated()
      ->mutable_value_set()
      ->mutable_values()
      ->insert({"alternate.portIndicated.0", FalseValue()});
  config.mutable_gripping_indicated()
      ->mutable_value_set()
      ->mutable_values()
      ->insert({"alternate.portIndicated.2", FalseValue()});

  GetSignalDescriptionsResponse valid_signals;

  // Add one of the write signals.
  AddWriteSignal(valid_signals, "portG", "alternate.portG");
  EXPECT_THAT(
      ::intrinsic::gripper::SignalsAreValid(config, valid_signals),
      StatusIs(
          absl::StatusCode::kInvalidArgument,
          HasSubstr("Invalid write signal name(s) provided: alternate.portR. "
                    "Valid signal names are: alternate.portG, portG")));

  // Adds one of the read signals.
  AddReadSignal(valid_signals, "portIndicated.0", "alternate.portIndicated.0");
  EXPECT_THAT(
      ::intrinsic::gripper::SignalsAreValid(config, valid_signals),
      StatusIs(absl::StatusCode::kInvalidArgument,
               HasSubstr("Invalid read signal name(s) provided: "
                         "alternate.portIndicated.2. Valid signal names are: "
                         "alternate.portIndicated.0, portIndicated.0.")));

  // Add remaining signals.
  AddWriteSignal(valid_signals, "portR", "alternate.portR");
  AddReadSignal(valid_signals, "portIndicated.2", "alternate.portIndicated.2");
  EXPECT_OK(::intrinsic::gripper::SignalsAreValid(config, valid_signals));
}
