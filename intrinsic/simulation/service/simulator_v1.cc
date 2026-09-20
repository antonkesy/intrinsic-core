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

#include "intrinsic/simulation/service/simulator_v1.h"

#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include "absl/status/status.h"
#include "grpc/grpc.h"
#include "grpcpp/client_context.h"
#include "intrinsic/simulation/service/proto/conversions.h"
#include "intrinsic/simulation/service/proto/first_party/simulation_service.pb.h"
#include "intrinsic/simulation/simulator/proto/v1/simulator_control_service.grpc.pb.h"
#include "intrinsic/simulation/simulator/proto/v1/simulator_control_service.pb.h"
#include "intrinsic/util/grpc/channel.h"
#include "intrinsic/util/grpc/connection_params.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace simulation {
namespace {

constexpr std::string_view kSimulatorVersionMismatchAbortedError =
    "Unable to control the simulator. Please re-deploy the solution and try "
    "again.";

constexpr absl::Duration kSimulatorRPCWaitForConnected = absl::Seconds(10);

intrinsic_proto::simulation::v1::ResetSimulatorRequest ToProto(
    const Simulator::ResetParams& reset_params) {
  intrinsic_proto::simulation::v1::ResetSimulatorRequest simulator_request;
  simulator_request.set_start_paused(reset_params.start_paused);
  if (!reset_params.simulator_world_id.empty()) {
    simulator_request.set_simulator_world_id(reset_params.simulator_world_id);
  }
  return simulator_request;
}

void ConfigureClientContext(::grpc::ClientContext& ctx,
                            const ConnectionParams& params) {
  if (!params.header.empty() && !params.instance_name.empty()) {
    ctx.AddMetadata(params.header, params.instance_name);
  }
}
}  // namespace

SimulatorV1::SimulatorV1(const Config& config)
    : sim_control_connection_params_(config.sim_control_connection_params),
      name_(config.simulator_name) {}

std::string SimulatorV1::GetName() const { return name_; }

absl::StatusOr<intrinsic_proto::simulation::v1::SimulatorControlService::Stub*>
SimulatorV1::GetStub() {
  absl::MutexLock lock(&stub_mutex_);
  if (stub_ != nullptr) {
    return stub_.get();
  }
  INTR_ASSIGN_OR_RETURN(
      channel_, Channel::MakeFromAddress(sim_control_connection_params_,
                                         kSimulatorRPCWaitForConnected));
  stub_ = intrinsic_proto::simulation::v1::SimulatorControlService::NewStub(
      channel_->GetChannel());
  return stub_.get();
}

absl::Status SimulatorV1::Reset(const Simulator::ResetParams& reset_request,
                                std::unique_ptr<::grpc::ClientContext> ctx) {
  INTR_RET_CHECK(ctx != nullptr);
  ConfigureClientContext(*ctx, sim_control_connection_params_);

  INTR_ASSIGN_OR_RETURN(auto* stub, GetStub());

  intrinsic_proto::simulation::v1::ResetSimulatorResponse resp;
  auto status = ToAbslStatus(
      stub->ResetSimulator(ctx.get(), ToProto(reset_request), &resp));
  if (absl::IsUnimplemented(status)) {
    return absl::AbortedError(kSimulatorVersionMismatchAbortedError);
  }
  return status;
}

absl::Status SimulatorV1::Pause() {
  grpc::ClientContext context;
  ConfigureClientContext(context, sim_control_connection_params_);

  INTR_ASSIGN_OR_RETURN(auto* stub, GetStub());

  intrinsic_proto::simulation::v1::PauseSimulatorRequest req;
  intrinsic_proto::simulation::v1::PauseSimulatorResponse resp;
  auto status = ToAbslStatus(stub->PauseSimulator(&context, req, &resp));
  if (absl::IsUnimplemented(status)) {
    return absl::AbortedError(kSimulatorVersionMismatchAbortedError);
  }
  return status;
}

absl::Status SimulatorV1::Unpause() {
  grpc::ClientContext context;
  ConfigureClientContext(context, sim_control_connection_params_);

  INTR_ASSIGN_OR_RETURN(auto* stub, GetStub());

  intrinsic_proto::simulation::v1::UnpauseSimulatorRequest req;
  intrinsic_proto::simulation::v1::UnpauseSimulatorResponse resp;
  auto status = ToAbslStatus(stub->UnpauseSimulator(&context, req, &resp));
  if (absl::IsUnimplemented(status)) {
    return absl::AbortedError(kSimulatorVersionMismatchAbortedError);
  }
  return status;
}

absl::StatusOr<
    intrinsic_proto::simulation::first_party::GetSimulationStatusResponse>
SimulatorV1::GetStatus() {
  grpc::ClientContext context;
  ConfigureClientContext(context, sim_control_connection_params_);

  INTR_ASSIGN_OR_RETURN(auto* stub, GetStub());

  intrinsic_proto::simulation::v1::GetSimulatorStatusRequest req;
  intrinsic_proto::simulation::v1::GetSimulatorStatusResponse simulator_resp;
  INTR_RETURN_IF_ERROR(
      ToAbslStatus(stub->GetSimulatorStatus(&context, req, &simulator_resp)))
      .With([](const absl::Status& status) {
        if (absl::IsUnimplemented(status)) {
          return absl::AbortedError(kSimulatorVersionMismatchAbortedError);
        }
        return status;
      });
  return FromSimulatorV1(simulator_resp);
}

}  // namespace simulation
}  // namespace intrinsic
