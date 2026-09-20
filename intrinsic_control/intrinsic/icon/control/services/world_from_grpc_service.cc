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

#include "intrinsic/icon/control/services/world_from_grpc_service.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/control/services/service.h"
#include "intrinsic/icon/control/services/service_registration.h"
#include "intrinsic/icon/control/services/world_service.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/proto/object_world_service.grpc.pb.h"

namespace intrinsic::icon {

absl::Status WorldFromGrpcService::Initialize(
    const Service::InitParameters& parameters, const std::string& world_id,
    std::shared_ptr<intrinsic_proto::world::ObjectWorldService::StubInterface>
        world_service_stub) {
  object_world_client_ = std::make_shared<world::ObjectWorldClient>(
      world_id, std::move(world_service_stub));

  // Make sure that our ObjectWorldClient actually works.
  return object_world_client_->GetRootObject().status();
}

absl::StatusOr<std::unique_ptr<WorldFromGrpcService>>
WorldFromGrpcService::Create(
    const Service::InitParameters& parameters, const std::string& world_id,
    std::shared_ptr<intrinsic_proto::world::ObjectWorldService::StubInterface>
        world_service_stub) {
  auto service = std::make_unique<WorldFromGrpcService>();
  INTR_RETURN_IF_ERROR(
      service->Initialize(parameters, world_id, std::move(world_service_stub)));
  return service;
}

std::shared_ptr<const world::ObjectWorldClient>
WorldFromGrpcService::GetObjectWorldClient() const {
  return object_world_client_;
}

INTRINSIC_REGISTER_SERVICE(WorldFromGrpcService, WorldService);

}  // namespace intrinsic::icon
