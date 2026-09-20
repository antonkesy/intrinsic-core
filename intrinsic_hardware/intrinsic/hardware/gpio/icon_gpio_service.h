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

#ifndef INTRINSIC_HARDWARE_GPIO_ICON_GPIO_SERVICE_H_
#define INTRINSIC_HARDWARE_GPIO_ICON_GPIO_SERVICE_H_

#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/hardware/gpio/icon_gpio_service_config.pb.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.grpc.pb.h"
#include "intrinsic/util/grpc/channel_interface.h"

namespace intrinsic {

// Connects to ICON and sets up a GPIOService instance that interacts with the
// DIO signals of the given part.
absl::StatusOr<std::unique_ptr<intrinsic_proto::gpio::v1::GPIOService::Service>>
MakeIconGPIOService(const intrinsic_proto::gpio::IconGpioServiceConfig& config,
                    std::shared_ptr<intrinsic::ChannelInterface> icon_channel);

}  // namespace intrinsic

#endif  // INTRINSIC_HARDWARE_GPIO_ICON_GPIO_SERVICE_H_
