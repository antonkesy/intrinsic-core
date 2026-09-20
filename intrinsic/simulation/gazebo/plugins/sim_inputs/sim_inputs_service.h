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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SIM_INPUTS_SIM_INPUTS_SERVICE_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SIM_INPUTS_SIM_INPUTS_SERVICE_H_

#include <cstddef>
#include <memory>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "gz/sim/EntityComponentManager.hh"
#include "intrinsic/simulation/gazebo/plugins/sim_inputs/v1/set_simulated_inputs.grpc.pb.h"
#include "intrinsic/simulation/gazebo/plugins/sim_inputs/v1/set_simulated_inputs.pb.h"

namespace intrinsic::simulation {

// A service that allows callers to set simulated inputs in a Gazebo
// EntityComponentManager.
//
// This is meant to be part of a top-level Gazebo plugin that offers a gRPC
// service to set simulated digital inputs (for now. the proto definition
// leaves room for adding analog inputs down the road!).
//
// It batches requests between Gazebo steps (as indicated by calls to
// `ProcessRequests()`) to ensure that all bits from each request get set in the
// same simulation step.
class SimInputsService final : public intrinsic_proto::simulation::v1::
                                   SetSimulatedInputsService::Service {
 public:
  ~SimInputsService() override;

  // Call this when you know that there won't be any further calls to
  // `SetSimulatedInputs()` OR `ProcessRequests()`.
  //
  // This finishes any pending requests with a CancelledError, so that pending
  // calls to `SetSimulatedInputs()` can finish, even if they have an infinite
  // deadline.
  void Shutdown();

  // Handles a request to set one or more simulated inputs.
  // Blocks until the next call to `ProcessRequests()`.
  //
  // Note that because of the way gRPC servers work, multiple calls to
  // SetSimulatedInputs may happen in parallel, causing concurrent calls to this
  // function!
  //
  // Returns NotFoundError if any of the input bits asks for a scene object that
  // doesn't exist, or an input block that doesn't exist on a scene object that
  // does.
  // Returns InvalidArgumentError if any bit indices are out of range.
  grpc::Status SetSimulatedInputs(
      grpc::ServerContext* context,
      const intrinsic_proto::simulation::v1::SetSimulatedInputsRequest* request,
      intrinsic_proto::simulation::v1::SetSimulatedInputsResponse* response)
      override;

  // Lists all simulated input blocks that the service knows about.
  grpc::Status ListSimulatedInputs(
      grpc::ServerContext* context,
      const intrinsic_proto::simulation::v1::ListSimulatedInputsRequest*
          request,
      intrinsic_proto::simulation::v1::ListSimulatedInputsResponse* response)
      override;

  // Processes any pending requests from SetSimulatedInputs(), and unblocks any
  // pending calls to that method.
  //
  // Also updates `available_inputs_`. This allows us to keep up to date in case
  // the information in the ECM changes.
  // That could happen because systems may get initialized in the "wrong" order,
  // so the input component is not available at initialization time.
  void ProcessRequests(gz::sim::EntityComponentManager& ecm);

  size_t NumPendingRequestsTestOnly() const;

 private:
  // Encapsulates the status of a pending request.
  // The processing thread (in `ProcessRequests()`) uses this to communicate a
  // status back to the `SetSimulatedInputs()` thread(s).
  struct RequestStatus {
    absl::Notification done_notification;
    absl::Mutex status_mutex;
    grpc::Status status ABSL_GUARDED_BY(status_mutex);
  };
  struct PendingRequest {
    intrinsic_proto::simulation::v1::SetSimulatedInputsRequest request;
    // This is a weak_ptr because the `SetSimulatedInputs()` thread may time out
    // before we get to processing the request. In that case, it doesn't make
    // sense to keep the request around, so we don't use shared_ptr here.
    std::weak_ptr<RequestStatus> request_status;
  };
  mutable absl::Mutex mutex_;
  std::vector<PendingRequest> pending_requests_ ABSL_GUARDED_BY(mutex_);
  intrinsic_proto::simulation::v1::ListSimulatedInputsResponse
      available_inputs_proto_ ABSL_GUARDED_BY(mutex_);
};

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SIM_INPUTS_SIM_INPUTS_SERVICE_H_
