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

#ifndef INTRINSIC_SIMULATION_SERVICE_SIMULATOR_WORLD_SYNC_V1_H_
#define INTRINSIC_SIMULATION_SERVICE_SIMULATOR_WORLD_SYNC_V1_H_

#include "absl/base/nullability.h"
#include "google/protobuf/empty.pb.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/simulation/service/proto/v1/simulator_world_sync.grpc.pb.h"
#include "intrinsic/simulation/service/proto/v1/simulator_world_sync.pb.h"
#include "intrinsic/simulation/service/simulation_service_impl.h"

namespace intrinsic {
namespace simulation {

// Implements the SimulatorWorldSync gRPC service for simulators.
class SimulatorWorldSyncV1 final
    : public intrinsic_proto::simulation::v1::SimulatorWorldSync::Service {
 public:
  explicit SimulatorWorldSyncV1(SimulationServiceImpl* absl_nonnull impl)
      : impl_(impl) {}

  // Disallow copy and move.
  SimulatorWorldSyncV1(SimulatorWorldSyncV1&&) = delete;
  SimulatorWorldSyncV1& operator=(SimulatorWorldSyncV1&&) = delete;
  SimulatorWorldSyncV1(const SimulatorWorldSyncV1&) = delete;
  SimulatorWorldSyncV1& operator=(const SimulatorWorldSyncV1&) = delete;

  ::grpc::Status SyncWorld(
      ::grpc::ServerContext* context,
      ::grpc::ServerReaderWriter<
          ::intrinsic_proto::simulation::v1::WorldMessage,
          ::intrinsic_proto::simulation::v1::SimulatorSceneMessage>* stream)
      final;

  ::grpc::Status GetSimulatorWorldInfo(
      ::grpc::ServerContext* context,
      const ::intrinsic_proto::simulation::v1::GetSimulatorWorldInfoRequest*
          request,
      ::intrinsic_proto::simulation::v1::SimulatorWorldInfo* response) final;

 private:
  // Externally owned.
  SimulationServiceImpl* impl_;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_SERVICE_SIMULATOR_WORLD_SYNC_V1_H_
