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

#include "intrinsic/simulation/gazebo/plugins/sim_inputs/sim_inputs_service.h"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/Name.hh"
#include "intrinsic/simulation/gazebo/components/digital_io_components.h"
#include "intrinsic/simulation/gazebo/components/ppr_component.h"
#include "intrinsic/util/status/status_conversion_grpc.h"

namespace intrinsic::simulation {

SimInputsService::~SimInputsService() {
  // Drain any in-flight requests, just in case nobody has called Shutdown()
  // manually before.
  Shutdown();
}

void SimInputsService::Shutdown() {
  absl::MutexLock lock(mutex_);
  if (pending_requests_.empty()) {
    return;
  }
  for (const auto& request : pending_requests_) {
    auto status_ptr = request.request_status.lock();
    if (status_ptr == nullptr) {
      continue;
    }
    if (status_ptr->done_notification.HasBeenNotified()) {
      continue;
    }
    {
      absl::MutexLock l(status_ptr->status_mutex);
      status_ptr->status =
          ToGrpcStatus(absl::CancelledError("Server is shutting down"));
    }
    status_ptr->done_notification.Notify();
  }
}

grpc::Status SimInputsService::SetSimulatedInputs(
    grpc::ServerContext* context,
    const intrinsic_proto::simulation::v1::SetSimulatedInputsRequest* request,
    intrinsic_proto::simulation::v1::SetSimulatedInputsResponse* response) {
  auto request_status = std::make_shared<RequestStatus>();
  {
    absl::MutexLock lock(mutex_);
    pending_requests_.push_back({
        .request = *request,
        .request_status = request_status,
    });
  }
  if (bool notified_before_deadline =
          request_status->done_notification.WaitForNotificationWithDeadline(
              absl::FromChrono(context->deadline()));
      !notified_before_deadline) {
    auto error = absl::DeadlineExceededError(
        "Failed to set desired simulated inputs within deadline. The "
        "simulation might be stuck, please check the logs of the gzserver "
        "pod");
    LOG(ERROR) << "SimInputsService::SetSimulatedInputs: " << error;
    return ToGrpcStatus(error);
  }
  absl::MutexLock l(request_status->status_mutex);
  if (!request_status->status.ok()) {
    LOG(ERROR) << "SimInputsService::SetSimulatedInputs: "
               << ToAbslStatus(request_status->status);
  }

  return request_status->status;
}

grpc::Status SimInputsService::ListSimulatedInputs(
    grpc::ServerContext* context,
    const intrinsic_proto::simulation::v1::ListSimulatedInputsRequest* request,
    intrinsic_proto::simulation::v1::ListSimulatedInputsResponse* response) {
  absl::MutexLock l(mutex_);
  *response = available_inputs_proto_;
  return ToGrpcStatus(absl::OkStatus());
}

void SimInputsService::ProcessRequests(gz::sim::EntityComponentManager& ecm) {
  // Build a map of all available DigitalInput components (that have a parent
  // entity with a ResourceName).
  // The map is keyed by scene object name, then by input block name.
  //
  // We expect the DigitalInput component pointers to stay valid for the scope
  // of `ProcessRequest()`!
  absl::flat_hash_map<
      std::string,
      absl::flat_hash_map<std::string, intrinsic::simulation::DigitalInput*>>
      available_inputs;
  // We also populate the response for ListSimulatedInputs here
  intrinsic_proto::simulation::v1::ListSimulatedInputsResponse
      available_inputs_proto;
  ecm.Each<intrinsic::simulation::DigitalInput>(
      [&](const gz::sim::Entity& entity,
          intrinsic::simulation::DigitalInput* digital_input) {
        const std::optional<std::string> block_name =
            ecm.ComponentData<gz::sim::components::Name>(entity);
        if (!block_name.has_value()) {
          return true;
        }
        // Look up the scene object name of the input block's parent (see
        // intrinsic/simulation/gazebo/plugins/digital_input_output.cc
        // for how we add these input components).
        //
        // If there isn't a parent, or the parent doesn't have a ResourceName
        // component, skip this input block.
        const auto parent_entity = ecm.ParentEntity(entity);
        if (parent_entity == gz::sim::kNullEntity) {
          return true;
        }
        const std::optional<std::string> scene_object_name =
            ecm.ComponentData<intrinsic::simulation::ResourceName>(
                parent_entity);
        if (!scene_object_name.has_value()) {
          return true;
        }

        available_inputs[*scene_object_name][*block_name] = digital_input;
        for (const auto& input_bit : digital_input->Data().data) {
          (*(*available_inputs_proto
                  .mutable_scene_objects_with_inputs())[*scene_object_name]
                .mutable_input_blocks())[*block_name]
              .mutable_digital_input_block()
              ->add_values(input_bit);
        }
        return true;
      });

  {
    absl::MutexLock l(mutex_);
    available_inputs_proto_ = available_inputs_proto;
  }

  std::vector<PendingRequest> requests_to_process;
  {
    absl::MutexLock lock(mutex_);
    if (pending_requests_.empty()) {
      return;
    }
    requests_to_process.swap(pending_requests_);
  }
  for (auto& pending_request : requests_to_process) {
    // Attempt to get a lock for the shared pointer `request_status`. If the
    // `SetSimulatedInputs()` thread has timed out already, the request is
    // already cancelled and there's no status for us to write into.
    std::shared_ptr<RequestStatus> request_status =
        pending_request.request_status.lock();
    if (request_status == nullptr) {
      LOG(WARNING) << "A SetSimulatedInputs request timed out before we could "
                      "handle it. Either the simulation is stepping very "
                      "slowly, or the request deadline was extremely short.";
      continue;
    }

    if (request_status->done_notification.HasBeenNotified()) {
      LOG(WARNING) << "A SetSimulatedInputs request appeared in "
                      "ProcessRequests() twice!";
      continue;
    }
    absl::Status result;
    for (const auto& input : pending_request.request.inputs()) {
      if (!input.has_digital_input_bit()) {
        result = absl::UnimplementedError("Unsupported input type");
        break;
      }

      const auto& digital_input_bit = input.digital_input_bit();
      const auto object_it = available_inputs.find(input.scene_object_name());
      if (object_it == available_inputs.end()) {
        result = absl::NotFoundError(
            absl::StrCat("Could not find scene object with name '",
                         input.scene_object_name(), "'"));
        break;
      }

      const auto& block_map = object_it->second;
      const auto block_it = block_map.find(digital_input_bit.block_name());
      if (block_it == block_map.end()) {
        result = absl::NotFoundError(absl::StrCat(
            "Could not find input block '", digital_input_bit.block_name(),
            "' on scene object '", input.scene_object_name(), "'"));
        break;
      }

      intrinsic::simulation::DigitalInput* digital_input = block_it->second;
      std::vector<bool>& bits = digital_input->Data().data;
      if (digital_input_bit.bit_index() < 0 ||
          digital_input_bit.bit_index() >= bits.size()) {
        result = absl::InvalidArgumentError(
            absl::StrCat("Bit index ", digital_input_bit.bit_index(),
                         " is out of bounds for input block '",
                         digital_input_bit.block_name(), "' on scene object '",
                         input.scene_object_name(), "'. That block only has ",
                         bits.size(), " bits"));
      } else {
        bits.at(digital_input_bit.bit_index()) = digital_input_bit.value();
      }

      if (!result.ok()) {
        break;
      }
    }
    {
      absl::MutexLock l(request_status->status_mutex);
      request_status->status = ToGrpcStatus(std::move(result));
    }
    request_status->done_notification.Notify();
  }
}

size_t SimInputsService::NumPendingRequestsTestOnly() const {
  absl::MutexLock l(mutex_);
  return pending_requests_.size();
}

}  // namespace intrinsic::simulation
