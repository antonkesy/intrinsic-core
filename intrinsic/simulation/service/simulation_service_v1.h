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

#ifndef INTRINSIC_SIMULATION_SERVICE_SIMULATION_SERVICE_V1_H_
#define INTRINSIC_SIMULATION_SERVICE_SIMULATION_SERVICE_V1_H_

#include "absl/base/nullability.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/simulation/service/proto/v1/simulation_service.grpc.pb.h"
#include "intrinsic/simulation/service/proto/v1/simulation_service.pb.h"
#include "intrinsic/simulation/service/simulation_service_impl.h"

namespace intrinsic {
namespace simulation {

// Implements first-party simulation service gRPC API for Gazebo.
class SimulationServiceV1 final
    : public intrinsic_proto::simulation::v1::SimulationService::Service {
 public:
  // Creates a new instance with the passed implementation instance.
  explicit SimulationServiceV1(SimulationServiceImpl* absl_nonnull impl)
      : impl_(impl) {};

  ::grpc::Status GetSimulatorName(
      ::grpc::ServerContext* context, const google::protobuf::Empty* request,
      google::protobuf::StringValue* response) final;

  ::grpc::Status ResetSimulation(
      ::grpc::ServerContext* context,
      const intrinsic_proto::simulation::v1::ResetSimulationRequest* request,
      intrinsic_proto::simulation::v1::ResetSimulationResponse* response) final;

 private:
  // Externally owned.
  SimulationServiceImpl* impl_;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_SERVICE_SIMULATION_SERVICE_V1_H_
