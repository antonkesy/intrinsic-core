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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SIM_INPUTS_SIM_INPUTS_SERVICE_SINGLETON_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SIM_INPUTS_SIM_INPUTS_SERVICE_SINGLETON_H_

#include <memory>

#include "absl/base/attributes.h"
#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/simulation/gazebo/plugins/sim_inputs/v1/set_simulated_inputs.grpc.pb.h"

namespace intrinsic::simulation {
class SimInputsServiceSingleton final : public intrinsic_proto::simulation::v1::
                                            SetSimulatedInputsService::Service {
 public:
  // This server looks for an environment variable with this name that holds an
  // address in the form "host:port".
  //
  // It starts a gRPC server for `SetSimulatedInputsService`
  // (intrinsic/simulation/gazebo/plugins/sim_inputs/v1/set_simulated_inputs.proto)
  // when it is created.
  //
  // If there is no such environment variable, the server defaults to
  // `0.0.0.0:12476`.
  static constexpr char kServiceAddressEnvironmentVariable[] =
      "SET_SIMULATED_INPUTS_SERVICE_ADDRESS";

  // Returns a singleton instance. The first call starts the gRPC server.
  static ABSL_MUST_USE_RESULT SimInputsServiceSingleton& Get();

  // Starts the gRPC server, forcing a crash if the address in the
  // `kServiceAddressEnvironmentVariable` environment variable is invalid or
  // already in use.
  SimInputsServiceSingleton();
  ~SimInputsServiceSingleton() override;

  // gRPC service methods. These delegate to the actual plugin if there is one,
  // and return UnimplementedError otherwise.
  ::grpc::Status SetSimulatedInputs(
      ::grpc::ServerContext* context,
      const intrinsic_proto::simulation::v1::SetSimulatedInputsRequest* request,
      intrinsic_proto::simulation::v1::SetSimulatedInputsResponse* response)
      override;
  ::grpc::Status ListSimulatedInputs(
      ::grpc::ServerContext* context,
      const intrinsic_proto::simulation::v1::ListSimulatedInputsRequest*
          request,
      intrinsic_proto::simulation::v1::ListSimulatedInputsResponse* response)
      override;

  // Registers a service implementation with this singleton.
  //
  // Note that SimInputsServiceSingleton does not take ownership of the
  // implementation!
  //
  // Returns AlreadyExistsError if SimInputsServiceSingleton already has a
  // (non-expired) implementation pointer. That is, there can only be one
  // implementation of SetSimulatedInputsService at any time.
  absl::Status RegisterServiceImplementation(
      std::weak_ptr<
          intrinsic_proto::simulation::v1::SetSimulatedInputsService::Service>
          implementation);

  // Call this to avoid using a test port outside the scope of a single test
  // case.
  void ShutdownServerTestOnly();

 private:
  std::unique_ptr<::grpc::Server> server_;
  absl::Mutex implementation_mutex_;
  std::weak_ptr<
      intrinsic_proto::simulation::v1::SetSimulatedInputsService::Service>
      implementation_ ABSL_GUARDED_BY(implementation_mutex_);
};
}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SIM_INPUTS_SIM_INPUTS_SERVICE_SINGLETON_H_
