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

#ifndef INTRINSIC_HARDWARE_GRIPPER_EOAT_GRIPPER_SERVICES_H_
#define INTRINSIC_HARDWARE_GRIPPER_EOAT_GRIPPER_SERVICES_H_

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "grpcpp/grpcpp.h"
#include "grpcpp/impl/service_type.h"
#include "intrinsic/hardware/gripper/eoat/eoat_service.pb.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"

namespace intrinsic::gripper {

// Wrapper class to include services to pass to the gripper server.
struct GripperServices {
  // Type erased services to start as part of EOAT gripper server.
  std::vector<std::unique_ptr<grpc::Service>> services;
  // Callable to invoke after services have been started
  // (e.g. to check that signal descriptions are correct).
  std::function<absl::Status()> post_services_callable;

  explicit GripperServices(
      std::vector<std::unique_ptr<grpc::Service>> services_in,
      std::function<absl::Status()> callable)
      : services(std::move(services_in)),
        post_services_callable(std::move(callable)) {}

  ~GripperServices() = default;
  GripperServices(GripperServices&&) = default;
  GripperServices& operator=(GripperServices&&) = default;
  GripperServices(const GripperServices&) = delete;
  GripperServices& operator=(GripperServices&) = delete;
};

// Creates gripper services that expose APIs to control the gripper using a GPIO
// service and report its health.
absl::StatusOr<GripperServices> MakeGripperAndGpioServices(
    const intrinsic_proto::eoat::GripperConfig& gripper_config,
    const intrinsic_proto::config::RuntimeContext& runtime_ctx);

}  // namespace intrinsic::gripper

#endif  // INTRINSIC_HARDWARE_GRIPPER_EOAT_GRIPPER_SERVICES_H_
