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

#include "intrinsic/hardware/gpio/gpio_service_factory.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "intrinsic/hardware/gpio/icon_gpio_service.h"
#include "intrinsic/hardware/gpio/icon_gpio_service_config.pb.h"
#include "intrinsic/hardware/gpio/opcua_gpio_service.h"
#include "intrinsic/hardware/gpio/opcua_gpio_service_config.pb.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.grpc.pb.h"
#include "intrinsic/util/grpc/channel.h"
#include "intrinsic/util/grpc/connection_params.h"
#include "intrinsic/util/proto/any.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

using intrinsic_proto::gpio::IconGpioServiceConfig;
using intrinsic_proto::gpio::OpcuaGpioServiceConfig;

absl::StatusOr<std::unique_ptr<intrinsic_proto::gpio::v1::GPIOService::Service>>
MakeGPIOService(const std::string& instance_name,
                const google::protobuf::Any& config_pb) {
  if (const auto config =
          intrinsic::UnpackAny<OpcuaGpioServiceConfig>(config_pb);
      config.ok()) {
    return intrinsic::MakeOpcuaGPIOService(instance_name, *config);
  }

  if (const auto config =
          intrinsic::UnpackAny<IconGpioServiceConfig>(config_pb);
      config.ok()) {
    std::string header = "x-resource-instance-name";
    if (!config->equipment_instance_header().empty()) {
      header = config->equipment_instance_header();
    }
    INTR_ASSIGN_OR_RETURN(
        auto channel,
        intrinsic::Channel::MakeFromAddress(intrinsic::ConnectionParams{
            .address = config->icon_address(),
            .instance_name = config->equipment_instance_name(),
            .header = header,
        }));

    LOG(INFO) << "Created ICON channel for address " << config->icon_address()
              << ", instance name " << config->equipment_instance_name()
              << ", header " << header;

    // IconGpioService retains ownership of the channel
    return intrinsic::MakeIconGPIOService(*config, std::move(channel));
  }

  return absl::InvalidArgumentError(
      "Unable to unpack: Invalid GPIO service configuration message type");
}

}  // namespace intrinsic
