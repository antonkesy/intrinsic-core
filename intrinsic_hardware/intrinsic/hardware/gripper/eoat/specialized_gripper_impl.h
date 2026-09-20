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

#ifndef INTRINSIC_HARDWARE_GRIPPER_EOAT_SPECIALIZED_GRIPPER_IMPL_H_
#define INTRINSIC_HARDWARE_GRIPPER_EOAT_SPECIALIZED_GRIPPER_IMPL_H_

#include <memory>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.grpc.pb.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.pb.h"
#include "intrinsic/hardware/gripper/eoat/eoat_service.pb.h"
#include "intrinsic/hardware/gripper/eoat/gripper_impl.h"
#include "intrinsic/util/grpc/grpc.h"

namespace intrinsic::gripper {

// The functions in this header create a template specialization of
// `GripperImpl` given the gripper configuration. A shared pointer is returned
// since the `GripperImpl` object may be shared among different GRPC services.

// Creates a specialization for pinch gripper that delays connecting to a GPIO
// service `gpio_service_address` till a method is called. This is useful in
// situations where the GPIO service is started in the same server as the
// gripper service.
absl::StatusOr<std::shared_ptr<::intrinsic::gripper::GripperImpl<
    intrinsic_proto::eoat::PinchGripperConfig>>>
MakePinchGripperImplFromConfig(
    const intrinsic_proto::eoat::PinchGripperConfig& config,
    absl::string_view gpio_service_address);

// Creates a specialization for suction gripper that delays connecting to a GPIO
// service `gpio_service_address` till a method is called.  This is useful in
// situations where the GPIO service is started in the same server as the
// gripper service.
absl::StatusOr<std::shared_ptr<::intrinsic::gripper::GripperImpl<
    intrinsic_proto::eoat::SuctionGripperConfig>>>
MakeSuctionGripperImplFromConfig(
    const intrinsic_proto::eoat::SuctionGripperConfig& config,
    absl::string_view gpio_service_address);

// Creates a specialization for pinch gripper that tries to connect to an
// existing GPIO service specified by `gpio_srv_endpoint`.
absl::StatusOr<std::shared_ptr<::intrinsic::gripper::GripperImpl<
    intrinsic_proto::eoat::PinchGripperConfig>>>
MakePinchGripperImplFromConfig(
    const intrinsic_proto::eoat::PinchGripperConfig& config,
    const intrinsic_proto::eoat::GpioServiceEndpoint& gpio_srv_endpoint);

// Creates a specialization for suction gripper that tries to connect to an
// existing GPIO service specified by `gpio_srv_endpoint`.
absl::StatusOr<std::shared_ptr<::intrinsic::gripper::GripperImpl<
    intrinsic_proto::eoat::SuctionGripperConfig>>>
MakeSuctionGripperImplFromConfig(
    const intrinsic_proto::eoat::SuctionGripperConfig& config,
    const intrinsic_proto::eoat::GpioServiceEndpoint& gpio_srv_endpoint);

// Creates a specialization for pinch gripper given a client stub to the GPIO
// service. This is mainly useful in testing with mocked GPIO service.
absl::StatusOr<std::shared_ptr<::intrinsic::gripper::GripperImpl<
    intrinsic_proto::eoat::PinchGripperConfig>>>
MakePinchGripperImplFromConfig(
    const intrinsic_proto::eoat::PinchGripperConfig& config,
    absl::string_view gpio_service_name,
    std::unique_ptr<intrinsic_proto::gpio::v1::GPIOService::StubInterface>
        gpio_stub);

// Creates a specialization for suction gripper given a client stub to the GPIO
// service. This is mainly useful in testing with mocked GPIO service.
absl::StatusOr<std::shared_ptr<::intrinsic::gripper::GripperImpl<
    intrinsic_proto::eoat::SuctionGripperConfig>>>
MakeSuctionGripperImplFromConfig(
    const intrinsic_proto::eoat::SuctionGripperConfig& config,
    absl::string_view gpio_service_name,
    std::unique_ptr<intrinsic_proto::gpio::v1::GPIOService::StubInterface>
        gpio_stub);

}  // namespace intrinsic::gripper

#endif  // INTRINSIC_HARDWARE_GRIPPER_EOAT_SPECIALIZED_GRIPPER_IMPL_H_
