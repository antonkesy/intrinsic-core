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

#include "intrinsic/icon/server/testing/in_process_server.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "absl/base/attributes.h"
#include "absl/base/nullability.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "grpcpp/channel.h"
#include "grpcpp/support/channel_arguments.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.grpc.pb.h"
#include "intrinsic/icon/proto/v1/jogging_service.grpc.pb.h"
#include "intrinsic/icon/proto/v1/service.grpc.pb.h"
#include "intrinsic/icon/server/grpc_envelope.h"
#include "intrinsic/icon/server/icon_api_service.h"
#include "intrinsic/icon/server/robot_connection_interface.h"
#include "intrinsic/icon/server/service.h"

namespace intrinsic {
namespace icon {

namespace {

class IconOnlyImpl final : public IconImplInterface {
 public:
  explicit IconOnlyImpl(std::unique_ptr<IconApiService> icon_service)
      : icon_service_(std::move(icon_service)) {}

  absl::StatusOr<icon::IconApiService* absl_nonnull> IconService()
      ABSL_ATTRIBUTE_LIFETIME_BOUND override {
    return icon_service_.get();
  }
  absl::StatusOr<intrinsic_proto::gpio::v1::GPIOService::Service* absl_nonnull>
  GpioService() ABSL_ATTRIBUTE_LIFETIME_BOUND override {
    return absl::UnimplementedError(
        "IconOnlyImpl does not support the GPIO service");
  }
  absl::StatusOr<
      intrinsic_proto::icon::v1::JoggingService::Service* absl_nonnull>
  JoggingService() ABSL_ATTRIBUTE_LIFETIME_BOUND override {
    return absl::UnimplementedError(
        "IconOnlyImpl does not support the Jogging service");
  }

 private:
  std::unique_ptr<IconApiService> icon_service_;
};

}  // namespace

InProcessApplicationLayerServer::InProcessApplicationLayerServer(
    RobotConnectionInterface& robot_connection, Options options) {
  GrpcEnvelope::Config config{
      .grpc_address = options.use_in_process_channel
                          ? std::nullopt
                          : std::optional<std::string>{absl::StrCat(
                                "[::1]:", options.port)},
      .icon_impl_factory =
          [&]() -> absl::StatusOr<std::unique_ptr<IconImplInterface>> {
        {
          absl::MutexLock l(factory_calls_mutex_);
          icon_impl_factory_calls_++;
        }
        return std::make_unique<IconOnlyImpl>(
            CreateApplicationLayerService(robot_connection));
      },
      .icon_name = robot_connection.config().name(),
  };
  grpc_envelope_ = std::make_unique<GrpcEnvelope>(std::move(config));
  if (!options.use_in_process_channel) {
    channel_ = connect::CreateClientChannel(
                   absl::StrCat("[::1]:", options.port),
                   absl::Now() + connect::kGrpcClientConnectDefaultTimeout)
                   .value();
  } else {
    channel_ = grpc_envelope_->InProcChannel({});
  }
}

std::unique_ptr<intrinsic_proto::icon::v1::IconApi::StubInterface>
InProcessApplicationLayerServer::MakeStub() const {
  return std::make_unique<intrinsic_proto::icon::v1::IconApi::Stub>(channel_);
}

std::shared_ptr<grpc::Channel> InProcessApplicationLayerServer::GetChannel()
    const {
  return channel_;
}

int InProcessApplicationLayerServer::IconImplFactoryCalls() const {
  absl::MutexLock l(factory_calls_mutex_);
  return icon_impl_factory_calls_;
}

}  // namespace icon
}  // namespace intrinsic
