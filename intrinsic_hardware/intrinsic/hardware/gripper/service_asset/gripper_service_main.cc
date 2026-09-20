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

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/synchronization/notification.h"
#include "grpcpp/impl/service_type.h"
#include "intrinsic/hardware/gripper/eoat/eoat_service.pb.h"
#include "intrinsic/hardware/gripper/eoat/gripper_services.h"
#include "intrinsic/hardware/gripper/service_asset/gripper_service_utils.h"
#include "intrinsic/hardware/gripper/service_asset/pinch_gripper_opcua_service_config.pb.h"
#include "intrinsic/hardware/gripper/service_asset/pinch_gripper_realtime_control_service_config.pb.h"
#include "intrinsic/hardware/gripper/service_asset/suction_gripper_opcua_service_config.pb.h"
#include "intrinsic/hardware/gripper/service_asset/suction_gripper_realtime_control_service_config.pb.h"
#include "intrinsic/icon/release/file_helpers.h"
#include "intrinsic/icon/release/portable/init_intrinsic.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "intrinsic/stats/opencensus.h"
#include "intrinsic/util/grpc/grpc.h"
#include "intrinsic/util/proto/any.h"
#include "intrinsic/util/status/status_macros.h"

ABSL_FLAG(std::string, runtime_context_file, "/etc/intrinsic/runtime_config.pb",
          "The path to the runtime context file containing "
          "intrinsic_proto.config.RuntimeContext binary proto.");

namespace intrinsic {
namespace gripper {

using ::intrinsic_proto::gripper_service::PinchGripperOpcuaServiceConfig;
using ::intrinsic_proto::gripper_service::
    PinchGripperRealtimeControlServiceConfig;
using ::intrinsic_proto::gripper_service::SuctionGripperOpcuaServiceConfig;
using ::intrinsic_proto::gripper_service::
    SuctionGripperRealtimeControlServiceConfig;

absl::StatusOr<GripperServices> MakeGripperServices(
    const intrinsic_proto::config::RuntimeContext& context) {
  ::intrinsic_proto::eoat::GripperConfig gripper_config;
  if (const auto config =
          intrinsic::UnpackAny<SuctionGripperRealtimeControlServiceConfig>(
              context.config());
      config.status().ok()) {
    LOG(INFO) << "Successfully unpacked suction gripper using realtime "
                 "control service configuration";
    INTR_ASSIGN_OR_RETURN(gripper_config,
                          MakeSuctionGripperRealtimeControlConfig(*config));
  } else if (const auto config =
                 intrinsic::UnpackAny<SuctionGripperOpcuaServiceConfig>(
                     context.config());
             config.status().ok()) {
    LOG(INFO) << "Successfully unpacked suction gripper using OPC UA service "
                 "configuration";
    INTR_ASSIGN_OR_RETURN(gripper_config,
                          MakeSuctionGripperOpcuaConfig(*config));
  } else if (const auto config =
                 intrinsic::UnpackAny<PinchGripperRealtimeControlServiceConfig>(
                     context.config());
             config.status().ok()) {
    LOG(INFO) << "Successfully unpacked pinch gripper using realtime control "
                 "service configuration";
    INTR_ASSIGN_OR_RETURN(gripper_config,
                          MakePinchGripperRealtimeControlConfig(*config));
  } else if (const auto config =
                 intrinsic::UnpackAny<PinchGripperOpcuaServiceConfig>(
                     context.config());
             config.status().ok()) {
    LOG(INFO) << "Successfully unpacked pinch gripper using OPC UA service "
                 "configuration";
    INTR_ASSIGN_OR_RETURN(gripper_config, MakePinchGripperOpcuaConfig(*config));
  } else {
    LOG(FATAL) << "Could not unpack gripper service configuration";
  }

  auto gripper_services = MakeGripperAndGpioServices(gripper_config, context);
  QCHECK_OK(gripper_services);
  return std::move(*gripper_services);
}

}  // namespace gripper
}  // namespace intrinsic

int main(int argc, char* argv[]) {
  InitIntrinsic(argv[0], argc, argv);
  intrinsic::OpenCensusPlugin opencensus;

  LOG(INFO) << "Reading runtime context from "
            << absl::GetFlag(FLAGS_runtime_context_file);
  const auto runtime_ctx =
      intrinsic::GetBinaryProto<intrinsic_proto::config::RuntimeContext>(
          absl::GetFlag(FLAGS_runtime_context_file));
  QCHECK_OK(runtime_ctx);

  // Configures the gripper services.
  auto gripper_services = intrinsic::gripper::MakeGripperServices(*runtime_ctx);
  QCHECK_OK(gripper_services);
  LOG(INFO) << "Successfully configured gripper services.";

  std::vector<::grpc::Service*> services;
  std::for_each(gripper_services->services.begin(),
                gripper_services->services.end(),
                [&services](std::unique_ptr<grpc::Service>& srv) {
                  services.push_back(srv.get());
                });

  const auto port = runtime_ctx->port();
  auto server = intrinsic::CreateServer(port, services);
  QCHECK_OK(server);
  LOG(INFO) << "Successfully started gripper service(s) on port " << port;

  // Invokes the post services callable
  if (const auto status = gripper_services->post_services_callable();
      !status.ok()) {
    LOG(WARNING) << "Post services callable failed with error: " << status;
  } else {
    LOG(INFO) << "Post services callable succeeded.";
  }

  // Registers SIGTERM signal handler and waits till shutdown.
  absl::Notification registered;
  QCHECK_OK(intrinsic::RegisterSignalHandlerAndWait(
      (*server).get(), intrinsic::ShutdownParams::Aggressive(), registered));
  return EXIT_SUCCESS;
}
