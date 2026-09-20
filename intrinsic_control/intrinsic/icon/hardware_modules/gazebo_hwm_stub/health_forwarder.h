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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_GAZEBO_HWM_STUB_HEALTH_FORWARDER_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_GAZEBO_HWM_STUB_HEALTH_FORWARDER_H_

#include <memory>
#include <string>
#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/assets/services/proto/v1/service_state.grpc.pb.h"
#include "intrinsic/assets/services/proto/v1/service_state.pb.h"
#include "intrinsic/simulation/gazebo/plugins/aggregated_resource_health.grpc.pb.h"
#include "intrinsic/simulation/gazebo/plugins/aggregated_resource_health.pb.h"

namespace intrinsic::icon {

// Forwards ServiceState requests to an AggregatedResourceHealth service stub.
class HealthForwarder
    : public intrinsic_proto::services::v1::ServiceState::Service {
 public:
  explicit HealthForwarder(absl::string_view resource_name)
      : name_(resource_name), stub_(nullptr) {}

  explicit HealthForwarder(
      absl::string_view resource_name,
      std::unique_ptr<
          intrinsic_proto::simulation::AggregatedResourceHealth::Stub>
          stub)
      : name_(resource_name), stub_(std::move(stub)) {}

  void SetResourceHealthStub(
      std::unique_ptr<
          intrinsic_proto::simulation::AggregatedResourceHealth::Stub>
          new_stub);

  // ServiceState implementation.
  grpc::Status GetState(
      grpc::ServerContext* context,
      const intrinsic_proto::services::v1::GetStateRequest* request,
      intrinsic_proto::services::v1::SelfState* response) override;

  grpc::Status Enable(
      grpc::ServerContext* context,
      const intrinsic_proto::services::v1::EnableRequest* request,
      intrinsic_proto::services::v1::EnableResponse* response) override;

  grpc::Status Disable(
      grpc::ServerContext* context,
      const intrinsic_proto::services::v1::DisableRequest* request,
      intrinsic_proto::services::v1::DisableResponse* response) override;

 private:
  std::string name_;
  absl::Mutex mutex_;
  // We forward all requests to, and responses from, this stub.
  std::unique_ptr<intrinsic_proto::simulation::AggregatedResourceHealth::Stub>
      stub_ ABSL_GUARDED_BY(mutex_);
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_GAZEBO_HWM_STUB_HEALTH_FORWARDER_H_
