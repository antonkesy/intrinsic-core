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

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/notification.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/hardware/gripper/gripper_equipment.pb.h"
#include "intrinsic/hardware/gripper/service/generic_gripper_service.h"
#include "intrinsic/hardware/gripper/service/health_service.h"
#include "intrinsic/hardware/gripper/service/pinch_gripper_server_async_callback_impl.h"
#include "intrinsic/hardware/gripper/service/proto/generic_gripper.grpc.pb.h"
#include "intrinsic/hardware/gripper/service/sim/health_service.h"
#include "intrinsic/hardware/gripper/service/sim/pinch_gripper_service.h"
#include "intrinsic/hardware/gripper/service/sim/sim_pinch_gripper_impl.h"
#include "intrinsic/icon/release/file_helpers.h"
#include "intrinsic/icon/release/portable/init_intrinsic.h"
#include "intrinsic/resources/proto/resource_registry.pb.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "intrinsic/stats/opencensus.h"
#include "intrinsic/util/grpc/grpc.h"
#include "intrinsic/util/proto/any.h"
#include "intrinsic/util/proto_time.h"

ABSL_FLAG(std::string, runtime_context_file, "/etc/intrinsic/runtime_config.pb",
          "The path to the runtime context file containing "
          "intrinsic_proto.config.RuntimeContext binary proto.");

static std::unique_ptr<intrinsic_proto::gripper::GenericGripper::Service>
MakeGenericGripperService(const int port) {
  const std::string addr = absl::StrCat("0.0.0.0:", port);
  return intrinsic::gripper::MakeGenericGripperService(addr);
}

int main(int argc, char* argv[]) {
  InitIntrinsic(argv[0], argc, argv);
  intrinsic::OpenCensusPlugin opencensus;

  LOG(INFO) << "Reading runtime context from "
            << absl::GetFlag(FLAGS_runtime_context_file);
  const auto runtime_ctx =
      intrinsic::GetBinaryProto<intrinsic_proto::config::RuntimeContext>(
          absl::GetFlag(FLAGS_runtime_context_file));
  QCHECK_OK(runtime_ctx);
  LOG(INFO) << "Runtime context level is: " << runtime_ctx->level();

  // TODO(b/298575301): Unconditionally use runtime_ctx->instance().id().
  auto service_config =
      intrinsic::UnpackAny<intrinsic_proto::gripper::PinchGripperPart>(
          runtime_ctx->config());
  const std::string& name = runtime_ctx->name();

  QCHECK(service_config->has_config()) << "config field must be provided";
  if (service_config->config().name().empty()) {
    LOG(INFO) << "Setting service configuration name to " << name;
    service_config->mutable_config()->set_name(name);
  } else if (service_config->config().name() != name) {
    LOG(ERROR) << "Mismatch between pinch gripper service config name"
               << service_config->config().name()
               << " and runtime_context resource instance name" << name;
  }
  LOG(INFO) << "Successfully unpacked service configuration";

  const auto default_timeout =
      service_config->grpc_config().has_grpc_timeout()
          ? std::make_optional(intrinsic::ToAbslDurationNoValidation(
                service_config->grpc_config().grpc_timeout()))
          : std::nullopt;

  const auto port = runtime_ctx->port();

  // TODO(dhirajgoel): Move the logic to start different services into another
  // compilation unit.

  if (runtime_ctx->level() ==
      intrinsic_proto::config::RuntimeContext::REALITY) {
    LOG(INFO) << "Creating the pinch gripper service";

    std::shared_ptr<intrinsic::gripper::PinchGripperServerAsyncCallbackImpl>
        pinch_gripper_service;

    if (service_config->config().has_generic_pinch_gripper_config()) {
      pinch_gripper_service = std::make_shared<
          intrinsic::gripper::PinchGripperServerAsyncCallbackImpl>(
          service_config->config(),
          service_config->config()
              .generic_pinch_gripper_config()
              .pinch_gripper_driver_name(),
          default_timeout);
    } else {
      LOG(ERROR) << "Unsupported pinch gripper service type";
    }

    LOG(INFO) << "Creating the gripper health service";
    auto health_service =
        intrinsic::gripper::MakeVariablePinchGripperHealthService(
            pinch_gripper_service);

    LOG(INFO) << "Creating the gripper ServiceState service";
    auto service_state =
        intrinsic::gripper::MakeVariablePinchGripperServiceState(
            pinch_gripper_service);

    LOG(INFO) << "Creating the generic gripper service";
    auto generic_gripper_service = MakeGenericGripperService(port);

    LOG(INFO) << "Creating the gRPC server";
    auto port = runtime_ctx->port();
    auto server = intrinsic::CreateServer(
        port, {pinch_gripper_service.get(), health_service.get(),
               service_state.get(), generic_gripper_service.get()});
    QCHECK_OK(server);

    LOG(INFO) << "Successfully started pinch gripper service on port " << port;

    if (auto status = pinch_gripper_service->InitializeAndEnable();
        !status.ok()) {
      LOG(ERROR) << "Failed to initialize pinch gripper: "
                 << status.error_message();
    }

    // Registers SIGTERM signal handler and waits till shutdown.
    absl::Notification registered;
    QCHECK_OK(intrinsic::RegisterSignalHandlerAndWait(
        (*server).get(), intrinsic::ShutdownParams::Aggressive(), registered));
  } else {
    // TODO(b/285210704): Remove hard-coded sim ports from
    // http://intrinsic/simulation/templates/gzserver.yaml
    constexpr int32_t kSimGripperPort = 12393;
    const auto grpc_address = absl::StrCat(
        runtime_ctx->simulation_server_address(), ":", kSimGripperPort);
    LOG(INFO) << "Create client channel on: " << grpc_address;

    const auto deadline = absl::Now() + absl::Seconds(300);
    const auto channel =
        intrinsic::connect::CreateClientChannel(grpc_address, deadline);
    QCHECK_OK(channel);

    auto gripper_impl =
        std::make_shared<intrinsic::gripper::simulation::SimPinchGripperImpl>(
            service_config->config(),
            intrinsic_proto::gripper::PinchGripperServer::NewStub(*channel));

    LOG(INFO) << "Creating the sim pinch gripper service.";
    auto gripper_service =
        intrinsic::gripper::simulation::MakeSimPinchGripperService(
            gripper_impl);

    LOG(INFO) << "Creating the sim health service.";
    auto health_service =
        intrinsic::gripper::simulation::MakeVariablePinchGripperHealthService(
            gripper_impl);

    LOG(INFO) << "Creating the sim ServiceState service.";
    auto service_state =
        intrinsic::gripper::simulation::MakeVariablePinchGripperServiceState(
            gripper_impl);

    LOG(INFO) << "Creating the generic gripper service";
    auto generic_gripper_service = MakeGenericGripperService(port);

    auto server = intrinsic::CreateServer(
        port, {gripper_service.get(), health_service.get(), service_state.get(),
               generic_gripper_service.get()});
    QCHECK_OK(server);
    LOG(INFO) << "Successfully started sim pinch gripper service(s) on port "
              << port;

    absl::Status status;
    while (absl::Now() < deadline) {
      status = gripper_impl->InitializeAndEnable();
      if (status.ok()) {
        break;
      }
      LOG(WARNING) << "Waiting for sim pinch gripper to initialize (retrying): "
                   << status;
      absl::SleepFor(absl::Seconds(1));
    }
    if (!status.ok()) {
      LOG(ERROR) << "Failed to initialize sim pinch gripper: " << status;
    }

    // Registers SIGTERM signal handler and waits till shutdown.
    absl::Notification registered;
    QCHECK_OK(intrinsic::RegisterSignalHandlerAndWait(
        (*server).get(), intrinsic::ShutdownParams::Aggressive(), registered));
  }
  return EXIT_SUCCESS;
}
