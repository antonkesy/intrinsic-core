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

#ifndef INTRINSIC_ICON_CONTROL_SERVICES_WORLD_FROM_GRPC_SERVICE_H_
#define INTRINSIC_ICON_CONTROL_SERVICES_WORLD_FROM_GRPC_SERVICE_H_

#include <memory>
#include <string>

#include "absl/base/nullability.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/control/services/service.h"
#include "intrinsic/icon/control/services/world_service.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/proto/object_world_service.grpc.pb.h"

namespace intrinsic::icon {

// Implementation of WorldService that reads information from a gRPC
// WorldService (see intrinsic/world/service/world_service.proto) and
// Geometry service
// (intrinsic/geometry/proto/geometry_service.proto).
class WorldFromGrpcService final
    : public ServiceImplBase<WorldFromGrpcService, WorldService> {
 public:
  static absl::StatusOr<std::unique_ptr<WorldFromGrpcService>> Create(
      const Service::InitParameters& parameters, const std::string& world_id,
      absl_nonnull
      std::shared_ptr<intrinsic_proto::world::ObjectWorldService::StubInterface>
          world_service_stub);

  std::shared_ptr<const world::ObjectWorldClient> GetObjectWorldClient()
      const override;

 private:
  absl::Status Initialize(
      const Service::InitParameters& parameters, const std::string& world_id,
      absl_nonnull
      std::shared_ptr<intrinsic_proto::world::ObjectWorldService::StubInterface>
          world_service_stub);

  std::shared_ptr<const world::ObjectWorldClient> object_world_client_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_SERVICES_WORLD_FROM_GRPC_SERVICE_H_
