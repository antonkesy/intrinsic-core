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

#ifndef INTRINSIC_HARDWARE_GPIO_GPIO_SERVICE_FACTORY_H_
#define INTRINSIC_HARDWARE_GPIO_GPIO_SERVICE_FACTORY_H_

#include <memory>
#include <string>

#include "absl/status/statusor.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.grpc.pb.h"

namespace intrinsic {

// Creates a GPIO service (OpcuaGpioService or BeckhoffGpioService).
// For OpcuaGpioService, OpcuaGpioServiceConfig proto should be passed.
// For BeckhoffGpioService, BeckhoffGpioServiceConfig should be passed.
absl::StatusOr<std::unique_ptr<intrinsic_proto::gpio::v1::GPIOService::Service>>
MakeGPIOService(const std::string& instance_name,
                const google::protobuf::Any& config_pb);

}  // namespace intrinsic

#endif  // INTRINSIC_HARDWARE_GPIO_GPIO_SERVICE_FACTORY_H_
