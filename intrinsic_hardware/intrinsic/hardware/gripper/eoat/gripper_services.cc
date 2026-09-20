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

#include "intrinsic/hardware/gripper/eoat/gripper_services.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "grpcpp/impl/service_type.h"
#include "intrinsic/hardware/gpio/gpio_service_factory.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.grpc.pb.h"
#include "intrinsic/hardware/gripper/eoat/eoat_service.pb.h"
#include "intrinsic/hardware/gripper/eoat/generic_gripper_service.h"
#include "intrinsic/hardware/gripper/eoat/gripper_health_service.h"
#include "intrinsic/hardware/gripper/eoat/pinch_gripper_service.h"
#include "intrinsic/hardware/gripper/eoat/specialized_gripper_impl.h"
#include "intrinsic/hardware/gripper/eoat/suction_gripper_service.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::gripper {

namespace {
using ::intrinsic_proto::config::RuntimeContext;
using ::intrinsic_proto::eoat::GpioServiceConfig;
using ::intrinsic_proto::eoat::GpioServiceEndpoint;
using ::intrinsic_proto::eoat::PinchGripperConfig;
using ::intrinsic_proto::eoat::SuctionGripperConfig;
using ::intrinsic_proto::gpio::v1::GPIOService;

// Information about the GPIO service needed by the gripper service.
struct RequiredGpioService {
  // GPIO service endpoint for gripper service to connect to.
  // `gpio_service_name` field is only needed if the GPIO service is behind an
  // ingress.
  GpioServiceEndpoint endpoint;

  // GPIO service that would be owned by the gripper server. This service is not
  // started if the provided configuration specifies the address of an existing
  // GPIO service to connect to or if running in physics sim.
  std::optional<std::unique_ptr<GPIOService::Service>> service;
};

absl::StatusOr<RequiredGpioService> MakeGpioServiceFromConfig(
    const GpioServiceConfig& gpio_service_config,
    const intrinsic_proto::config::RuntimeContext& runtime_ctx) {
  // Starts a GPIO service for real hardware if needed.
  if (runtime_ctx.level() == intrinsic_proto::config::RuntimeContext::REALITY) {
    switch (gpio_service_config.srv_case()) {
      case intrinsic_proto::eoat::GpioServiceConfig::kEndpoint:
        return RequiredGpioService{.endpoint = gpio_service_config.endpoint(),
                                   .service = std::nullopt};

      case intrinsic_proto::eoat::GpioServiceConfig::kConfig: {
        INTR_ASSIGN_OR_RETURN(
            auto gpio_service,
            intrinsic::MakeGPIOService(runtime_ctx.name(),
                                       gpio_service_config.config()));
        const auto grpc_address =
            absl::StrCat("127.0.0.1:", runtime_ctx.port());
        LOG(INFO) << "Successfully configured gpio service at " << grpc_address;
        GpioServiceEndpoint endpoint;
        endpoint.set_grpc_address(grpc_address);
        return RequiredGpioService{.endpoint = endpoint,
                                   .service = std::move(gpio_service)};
      }

      case intrinsic_proto::eoat::GpioServiceConfig::SRV_NOT_SET:
        return absl::InvalidArgumentError(
            "`GpioServiceConfig` is empty. This configuration is needed by the "
            "gripper service to connect to the backend GPIO service.");
    }

    // Should never come here but handle the error in case a malformed config is
    // received.
    return absl::InvalidArgumentError(
        "`GpioServiceConfig` is invalid. This configuration is needed by the "
        "gripper service to connect to the backend GPIO service.");
  }

  // For physics simulation, connect to the simulated GPIO service.
  // TODO(b/285210704): Remove hard-coded sim ports from
  // http://intrinsic/simulation/templates/gzserver.yaml
  constexpr int32_t kSimGPIOPort = 12394;
  const auto grpc_address =
      absl::StrCat(runtime_ctx.simulation_server_address(), ":", kSimGPIOPort);
  LOG(INFO) << "Not starting gpio service in sim mode. Trying to connect to "
               "GPIO server in sim at "
            << grpc_address;

  GpioServiceEndpoint endpoint;
  endpoint.set_grpc_address(grpc_address);
  endpoint.set_gpio_service_name("gripper");
  return RequiredGpioService{.endpoint = endpoint, .service = std::nullopt};
}

template <class SuctionOrPinchConfig, class GpioConfig>
static absl::StatusOr<GripperServices> MakeGripperServices(
    const SuctionOrPinchConfig& gripper_config, const GpioConfig& gpio_config) {
  static_assert(std::is_same_v<SuctionOrPinchConfig, SuctionGripperConfig> ||
                std::is_same_v<SuctionOrPinchConfig, PinchGripperConfig>);
  // Creates a gripper_impl object that will be shared among services.
  INTR_ASSIGN_OR_RETURN(auto gripper_impl, [&]() {
    // TODO: b/390472491 - Start gRPC services even when the configuration is
    // invalid and forward the configuration error to the frontend using
    // ServiceState's extended status field.
    if constexpr (std::is_same_v<SuctionOrPinchConfig, SuctionGripperConfig>) {
      return MakeSuctionGripperImplFromConfig(gripper_config, gpio_config);
    }
    if constexpr (std::is_same_v<SuctionOrPinchConfig, PinchGripperConfig>) {
      return MakePinchGripperImplFromConfig(gripper_config, gpio_config);
    }
  }());

  // Creates the gripper service.
  auto gripper_srv = [gripper_impl]() {
    if constexpr (std::is_same_v<SuctionOrPinchConfig, SuctionGripperConfig>) {
      return MakeSuctionGripperService(gripper_impl);
    }
    if constexpr (std::is_same_v<SuctionOrPinchConfig, PinchGripperConfig>) {
      return MakePinchGripperService(gripper_impl);
    }
  }();

  // Creates generic gripper service.
  auto generic_gripper_srv = MakeGenericGripperService(gripper_impl);

  // Creates the health service.
  auto health_srv = MakeGripperHealthService(gripper_impl);

  // Creates the ServiceState service.
  auto state_srv = MakeGripperServiceState(gripper_impl);

  // Populates all the instantiated services to be returned to the caller.
  std::vector<std::unique_ptr<grpc::Service>> services;
  services.push_back(std::move(health_srv));
  services.push_back(std::move(state_srv));
  services.push_back(std::move(gripper_srv));
  services.push_back(std::move(generic_gripper_srv));

  // Function to be called after services have been started.
  auto clear_faults_and_enable = [gripper_impl]() -> absl::Status {
    INTR_RETURN_IF_ERROR(ToAbslStatus(gripper_impl->ClearFaults())).LogError();
    // TODO(b/262820386): Start in enabled state to avoid any regression in
    // behavior until the front end (or some other entity) supports health
    // reporting APIs (e.g. enable before starting app).
    return ToAbslStatus(gripper_impl->Enable());
  };

  return GripperServices(std::move(services), clear_faults_and_enable);
}

template <class GpioConfig>
absl::StatusOr<GripperServices> MakeSuctionOrPinchGripperServices(
    const intrinsic_proto::eoat::GripperConfig& config,
    const GpioConfig& gpio_config) {
  switch (config.gripper_config_case()) {
    case intrinsic_proto::eoat::GripperConfig::kPinch:
      return MakeGripperServices<>(config.pinch(), gpio_config);

    case intrinsic_proto::eoat::GripperConfig::kSuction:
      return MakeGripperServices<>(config.suction(), gpio_config);

    case intrinsic_proto::eoat::GripperConfig::GRIPPER_CONFIG_NOT_SET:
      return absl::InvalidArgumentError(
          "Invalid intrinsic_proto::eoat::GripperConfig passed to gripper "
          "service.");
  }
}

}  // namespace

absl::StatusOr<GripperServices> MakeGripperAndGpioServices(
    const intrinsic_proto::eoat::GripperConfig& config,
    const RuntimeContext& runtime_ctx) {
  // Creates the GPIO service, or returns the address to the connect to an
  // existing one.
  INTR_ASSIGN_OR_RETURN(
      auto gpio_service,
      MakeGpioServiceFromConfig(config.gpio_service(), runtime_ctx));

  if (gpio_service.service.has_value()) {
    // GPIO service is being started along with gripper services. So only pass
    // the grpc address to allow delayed connection from the gripper service to
    // the gpio service.
    INTR_ASSIGN_OR_RETURN(auto services,
                          MakeSuctionOrPinchGripperServices(
                              config, gpio_service.endpoint.grpc_address()));
    services.services.push_back(std::move(*gpio_service.service));
    return services;
  }

  // TODO(dhiajgoel): allow delayed connection to an existing GPIO endpoint and
  // consolidate the handling of gpio `endpoint` and `grpc_address`.
  return MakeSuctionOrPinchGripperServices(config, gpio_service.endpoint);
}

}  // namespace intrinsic::gripper
