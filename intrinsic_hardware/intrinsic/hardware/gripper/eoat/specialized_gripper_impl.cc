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

#include "intrinsic/hardware/gripper/eoat/specialized_gripper_impl.h"

#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/clock.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.grpc.pb.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.pb.h"
#include "intrinsic/hardware/gripper/eoat/eoat_service.pb.h"
#include "intrinsic/hardware/gripper/eoat/gripper_impl.h"
#include "intrinsic/hardware/gripper/eoat/gripper_utils.h"
#include "intrinsic/util/status/status_macros.h"

namespace {

using ::intrinsic_proto::eoat::GpioServiceEndpoint;
using ::intrinsic_proto::eoat::PinchGripperConfig;
using ::intrinsic_proto::eoat::SuctionGripperConfig;

// Boilerplate code to choose the correct `SignalsToClaim` function from
// the overloaded set.
typedef absl::flat_hash_set<std::string> (*pinch_signals_func)(
    const PinchGripperConfig&);
static constexpr pinch_signals_func kPinchSignalsFunc =
    ::intrinsic::gripper::SignalsToClaim;

typedef absl::flat_hash_set<std::string> (*suction_signals_func)(
    const SuctionGripperConfig&);
static constexpr suction_signals_func kSuctionSignalsFunc =
    ::intrinsic::gripper::SignalsToClaim;

template <class ActualGripperConfig, class GpioConfig>
absl::StatusOr<
    std::shared_ptr<::intrinsic::gripper::GripperImpl<ActualGripperConfig>>>
MakeGripperImplFromConfig(const ActualGripperConfig& gripper_config,
                          const GpioConfig& gpio_config) {
  static_assert(std::is_same_v<GpioConfig, absl::string_view> ||
                    std::is_same_v<GpioConfig,
                                   intrinsic_proto::eoat::GpioServiceEndpoint>,
                "gpio_config has to be one of: absl:string_view or "
                "intrinsic_proto::eoat::GpioServiceEndpoint");

  INTR_RETURN_IF_ERROR(
      ::intrinsic::gripper::GripperConfigIsValid(gripper_config));

  auto signals_func = []() {
    if constexpr (std::is_same_v<ActualGripperConfig, PinchGripperConfig>) {
      return kPinchSignalsFunc;
    }
    if constexpr (std::is_same_v<ActualGripperConfig, SuctionGripperConfig>) {
      return kSuctionSignalsFunc;
    }
  }();

  if constexpr (std::is_same_v<GpioConfig, absl::string_view>) {
    return std::make_shared<
        ::intrinsic::gripper::GripperImpl<ActualGripperConfig>>(
        gripper_config, signals_func, gpio_config);
  }

  if constexpr (std::is_same_v<GpioConfig, GpioServiceEndpoint>) {
    INTR_RETURN_IF_ERROR(
        ::intrinsic::gripper::GpioSrvEndpointIsValid(gpio_config));

    const auto deadline =
        absl::Now() + intrinsic::connect::kGrpcClientConnectDefaultTimeout;
    INTR_ASSIGN_OR_RETURN(auto channel,
                          intrinsic::connect::CreateClientChannel(
                              gpio_config.grpc_address(), deadline));

    return std::make_shared<
        ::intrinsic::gripper::GripperImpl<ActualGripperConfig>>(
        gripper_config, signals_func, gpio_config.gpio_service_name(),
        intrinsic_proto::gpio::v1::GPIOService::NewStub(channel));
  }
}

template <class ActualGripperConfig>
absl::StatusOr<
    std::shared_ptr<::intrinsic::gripper::GripperImpl<ActualGripperConfig>>>
MakeGripperImplFromConfig(
    const ActualGripperConfig& gripper_config,
    const absl::string_view gpio_service_name,
    std::unique_ptr<intrinsic_proto::gpio::v1::GPIOService::StubInterface>
        gpio_stub) {
  INTR_RETURN_IF_ERROR(
      ::intrinsic::gripper::GripperConfigIsValid(gripper_config));

  auto signals_func = []() {
    if constexpr (std::is_same_v<ActualGripperConfig, PinchGripperConfig>) {
      return kPinchSignalsFunc;
    }
    if constexpr (std::is_same_v<ActualGripperConfig, SuctionGripperConfig>) {
      return kSuctionSignalsFunc;
    }
  }();

  return std::make_shared<
      ::intrinsic::gripper::GripperImpl<ActualGripperConfig>>(
      gripper_config, signals_func, gpio_service_name, std::move(gpio_stub));
}

}  // namespace

namespace intrinsic::gripper {

absl::StatusOr<
    std::shared_ptr<::intrinsic::gripper::GripperImpl<PinchGripperConfig>>>
MakePinchGripperImplFromConfig(const PinchGripperConfig& config,
                               const absl::string_view gpio_service_address) {
  return MakeGripperImplFromConfig<>(config, gpio_service_address);
}

absl::StatusOr<
    std::shared_ptr<::intrinsic::gripper::GripperImpl<PinchGripperConfig>>>
MakePinchGripperImplFromConfig(
    const PinchGripperConfig& config,
    const intrinsic_proto::eoat::GpioServiceEndpoint& gpio_srv_endpoint) {
  return MakeGripperImplFromConfig<>(config, gpio_srv_endpoint);
}

absl::StatusOr<
    std::shared_ptr<::intrinsic::gripper::GripperImpl<SuctionGripperConfig>>>
MakeSuctionGripperImplFromConfig(const SuctionGripperConfig& config,
                                 const absl::string_view gpio_service_address) {
  return MakeGripperImplFromConfig<>(config, gpio_service_address);
}

absl::StatusOr<
    std::shared_ptr<::intrinsic::gripper::GripperImpl<SuctionGripperConfig>>>
MakeSuctionGripperImplFromConfig(
    const SuctionGripperConfig& config,
    const intrinsic_proto::eoat::GpioServiceEndpoint& gpio_srv_endpoint) {
  return MakeGripperImplFromConfig<>(config, gpio_srv_endpoint);
}

absl::StatusOr<
    std::shared_ptr<::intrinsic::gripper::GripperImpl<PinchGripperConfig>>>
MakePinchGripperImplFromConfig(
    const PinchGripperConfig& config, const absl::string_view gpio_service_name,
    std::unique_ptr<intrinsic_proto::gpio::v1::GPIOService::StubInterface>
        gpio_stub) {
  return MakeGripperImplFromConfig<>(config, gpio_service_name,
                                     std::move(gpio_stub));
}

absl::StatusOr<
    std::shared_ptr<::intrinsic::gripper::GripperImpl<SuctionGripperConfig>>>
MakeSuctionGripperImplFromConfig(
    const SuctionGripperConfig& config,
    const absl::string_view gpio_service_name,
    std::unique_ptr<intrinsic_proto::gpio::v1::GPIOService::StubInterface>
        gpio_stub) {
  return MakeGripperImplFromConfig<>(config, gpio_service_name,
                                     std::move(gpio_stub));
}

}  // namespace intrinsic::gripper
