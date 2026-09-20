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

#include "intrinsic/simulation/service/simulation_service_v1.h"

#include "google/protobuf/empty.pb.h"
#include "grpcpp/client_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/simulation/service/proto/conversions.h"
#include "intrinsic/simulation/service/proto/first_party/simulation_service.pb.h"
#include "intrinsic/simulation/service/proto/v1/simulation_service.pb.h"
#include "intrinsic/util/status/status_conversion_grpc.h"

namespace intrinsic {
namespace simulation {

::grpc::Status SimulationServiceV1::GetSimulatorName(
    ::grpc::ServerContext* context, const google::protobuf::Empty* request,
    google::protobuf::StringValue* response) {
  return ToGrpcStatus(impl_->GetSimulatorName(context, request, response));
}

::grpc::Status SimulationServiceV1::ResetSimulation(
    ::grpc::ServerContext* context,
    const intrinsic_proto::simulation::v1::ResetSimulationRequest* request,
    intrinsic_proto::simulation::v1::ResetSimulationResponse* response) {
  intrinsic_proto::simulation::first_party::ResetSimulationRequest req_1p =
      FromV1(*request);
  return ToGrpcStatus(impl_->ResetSimulation(context, &req_1p, {}));
}

}  // namespace simulation
}  // namespace intrinsic
