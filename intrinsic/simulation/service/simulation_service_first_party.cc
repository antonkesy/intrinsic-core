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

#include "intrinsic/simulation/service/simulation_service_first_party.h"

#include "google/protobuf/empty.pb.h"
#include "grpcpp/client_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/simulation/service/proto/first_party/simulation_service.pb.h"
#include "intrinsic/util/status/status_conversion_grpc.h"

namespace intrinsic {
namespace simulation {

::grpc::Status SimulationServiceFirstParty::StartSolution(
    ::grpc::ServerContext* context,
    const intrinsic_proto::simulation::first_party::StartSolutionRequest*
        request,
    intrinsic_proto::simulation::first_party::StartSolutionResponse* response) {
  return ToGrpcStatus(impl_->StartSolution(context, request, response));
}

::grpc::Status SimulationServiceFirstParty::StopSolution(
    ::grpc::ServerContext* context,
    const intrinsic_proto::simulation::first_party::StopSolutionRequest*
        request,
    intrinsic_proto::simulation::first_party::StopSolutionResponse* response) {
  return ToGrpcStatus(impl_->StopSolution(context, request, response));
}

::grpc::Status SimulationServiceFirstParty::GetSimulatorName(
    ::grpc::ServerContext* context, const google::protobuf::Empty* request,
    google::protobuf::StringValue* response) {
  return ToGrpcStatus(impl_->GetSimulatorName(context, request, response));
}

::grpc::Status SimulationServiceFirstParty::ResetSimulation(
    ::grpc::ServerContext* context,
    const intrinsic_proto::simulation::first_party::ResetSimulationRequest*
        request,
    google::protobuf::Empty* response) {
  return ToGrpcStatus(impl_->ResetSimulation(context, request, response));
}

::grpc::Status SimulationServiceFirstParty::PauseSimulation(
    ::grpc::ServerContext* context, const google::protobuf::Empty* request,
    google::protobuf::Empty* response) {
  return ToGrpcStatus(impl_->PauseSimulation(context, request, response));
}

::grpc::Status SimulationServiceFirstParty::UnpauseSimulation(
    ::grpc::ServerContext* context, const google::protobuf::Empty* request,
    google::protobuf::Empty* response) {
  return ToGrpcStatus(impl_->UnpauseSimulation(context, request, response));
}

::grpc::Status SimulationServiceFirstParty::GetSimulationStatus(
    ::grpc::ServerContext* context, const google::protobuf::Empty* request,
    intrinsic_proto::simulation::first_party::GetSimulationStatusResponse*
        response) {
  return ToGrpcStatus(impl_->GetSimulationStatus(context, request, response));
}

::grpc::Status SimulationServiceFirstParty::GetSimulatorStatus(
    ::grpc::ServerContext* context,
    const intrinsic_proto::simulation::first_party::GetSimulatorStatusRequest*
        request,
    intrinsic_proto::simulation::first_party::GetSimulatorStatusResponse*
        response) {
  return ToGrpcStatus(impl_->GetSimulatorStatus(context, request, response));
}

::grpc::Status SimulationServiceFirstParty::RefreshConnectedSimulator(
    ::grpc::ServerContext* context,
    const intrinsic_proto::simulation::first_party::
        RefreshConnectedSimulatorRequest* request,
    intrinsic_proto::simulation::first_party::RefreshConnectedSimulatorResponse*
        response) {
  return ToGrpcStatus(
      impl_->RefreshConnectedSimulator(context, request, response));
}

::grpc::Status SimulationServiceFirstParty::RunVisualization(
    ::grpc::ServerContext* context,
    const intrinsic_proto::simulation::first_party::
        VisualizeWorldUpdatesRequest* request,
    google::protobuf::Empty* response) {
  return ToGrpcStatus(impl_->RunVisualization(context, request, response));
}

}  // namespace simulation
}  // namespace intrinsic
