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

#include "intrinsic/simulation/gazebo/plugins/service_state_aggregator.h"

#include <cstdlib>
#include <memory>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "grpcpp/security/server_credentials.h"
#include "grpcpp/server_builder.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Types.hh"
#include "intrinsic/assets/services/proto/v1/service_state.pb.h"
#include "intrinsic/simulation/gazebo/components/ppr_component.h"
#include "intrinsic/simulation/gazebo/components/service_state_component.h"
#include "intrinsic/simulation/gazebo/plugins/aggregated_resource_health.pb.h"
#include "intrinsic/simulation/gazebo/plugins/priority_constants.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"
#include "sdf/Element.hh"

namespace intrinsic::simulation {

void ServiceStateAggregator::Configure(
    const gz::sim::Entity& entity,
    const std::shared_ptr<const ::sdf::Element>& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& event_manager) {
  ::grpc::ServerBuilder builder;
  const char* server_address = getenv(kServiceAddressEnvironmentVariable);
  if (server_address != nullptr) {
    LOG(INFO) << "Using server address from env variable: " << server_address;
    // Listen on the given address without any authentication mechanism.
    builder.AddListeningPort(
        server_address,
        ::grpc::InsecureServerCredentials());  // NOLINT (insecure)
  } else {
    LOG(ERROR) << "No address specified for ServiceStateAggregator "
                  "service. Please set the environment variable '"
               << kServiceAddressEnvironmentVariable
               << "' to an address in the format host:port";
  }
  builder.RegisterService(this);
  aggregated_resource_health_server_ = builder.BuildAndStart();
}

gz::sim::System::PriorityType ServiceStateAggregator::ConfigurePriority() {
  return plugins::kServiceStateAggregatorPriority;
}

void ServiceStateAggregator::PreUpdate(const gz::sim::UpdateInfo& info,
                                       gz::sim::EntityComponentManager& ecm) {
  // Iterate over all Entities that have *both* a ResourceName and
  // ServiceState component.
  ecm.Each<intrinsic::simulation::ResourceName,
           intrinsic::simulation::ServiceState>(
      [&](const gz::sim::Entity& entity,
          intrinsic::simulation::ResourceName* name_component,
          intrinsic::simulation::ServiceState* health_component) {
        absl::MutexLock l(resource_data_mtx_);
        // If we do not have ServiceStateData for this resource, bail out
        // early.
        auto health_data =
            resource_health_data_by_resource_name_.find(name_component->Data());
        if (health_data == resource_health_data_by_resource_name_.end()) {
          return true;
        }
        if (health_data->second.clear_faults_requested) {
          health_component->Data().clear_faults_requested =
              health_data->second.clear_faults_requested;
          // Unset the flag in our buffer, so that we don't request more
          // ClearFault operations than intended.
          health_data->second.clear_faults_requested = false;
        }
        return true;
      });
}

void ServiceStateAggregator::PostUpdate(
    const gz::sim::UpdateInfo& info,
    const gz::sim::EntityComponentManager& ecm) {
  // After the Update() step, copy ServiceState component data to the
  // `resource_health_data_by_resource_name_` map.
  ecm.Each<intrinsic::simulation::ResourceName,
           intrinsic::simulation::ServiceState>(
      [&](const gz::sim::Entity& entity,
          const intrinsic::simulation::ResourceName* name_component,
          const intrinsic::simulation::ServiceState* health_component) {
        absl::MutexLock l(resource_data_mtx_);
        resource_health_data_by_resource_name_[name_component->Data()].state =
            health_component->Data().state;
        return true;
      });
}

::grpc::Status ServiceStateAggregator::CheckHealth(
    ::grpc::ServerContext* context,
    const ::intrinsic_proto::simulation::AggregatedResourceHealthStatusRequest*
        request,
    ::intrinsic_proto::simulation::AggregatedResourceHealthStatusResponse*
        response) {
  absl::MutexLock l(resource_data_mtx_);
  auto health_data =
      resource_health_data_by_resource_name_.find(request->resource_name());
  if (health_data == resource_health_data_by_resource_name_.end()) {
    return ToGrpcStatus(absl::UnavailableError(
        absl::StrCat("No ServiceState data available for asset '",
                     request->resource_name(), "' (yet)")));
  }

  if (!health_data->second.state.ok()) {
    return ToGrpcStatus(health_data->second.state.status());
  }
  // Convert the state code to the simulation OperationalStatus.
  switch (health_data->second.state->state_code()) {
    case intrinsic_proto::services::v1::SelfState::STATE_CODE_ERROR:
      response->mutable_response()->mutable_status()->set_state(
          intrinsic_proto::simulation::OperationalStatus::FAULTED);
      break;
    case intrinsic_proto::services::v1::SelfState::STATE_CODE_DISABLED:
      response->mutable_response()->mutable_status()->set_state(
          intrinsic_proto::simulation::OperationalStatus::DISABLED);
      break;
    case intrinsic_proto::services::v1::SelfState::STATE_CODE_ENABLED:
      response->mutable_response()->mutable_status()->set_state(
          intrinsic_proto::simulation::OperationalStatus::ENABLED);
      break;
    default:
      response->mutable_response()->mutable_status()->set_state(
          intrinsic_proto::simulation::OperationalStatus::UNSPECIFIED);
      break;
  }
  if (health_data->second.state->has_extended_status()) {
    response->mutable_response()->mutable_status()->set_explanation(
        health_data->second.state->extended_status().user_report().message());
  }
  return ::grpc::Status::OK;
}

::grpc::Status ServiceStateAggregator::Enable(
    ::grpc::ServerContext* context,
    const ::intrinsic_proto::simulation::AggregatedResourceEnableRequest*
        request,
    ::intrinsic_proto::simulation::AggregatedResourceEnableResponse* response) {
  {
    absl::MutexLock l(resource_data_mtx_);
    auto health_data =
        resource_health_data_by_resource_name_.find(request->resource_name());
    if (health_data == resource_health_data_by_resource_name_.end()) {
      return ToGrpcStatus(absl::UnavailableError(
          absl::StrCat("No ServiceState data available for asset '",
                       request->resource_name(), "' (yet)")));
    }
  }
  return ToGrpcStatus(
      absl::UnavailableError("Cannot enable a hardware module directly. "
                             "Hardware modules are enabled automatically "
                             "via the realtime control service when no "
                             "hardware module has an error."));
}

::grpc::Status ServiceStateAggregator::Disable(
    ::grpc::ServerContext* context,
    const ::intrinsic_proto::simulation::AggregatedResourceDisableRequest*
        request,
    ::intrinsic_proto::simulation::AggregatedResourceDisableResponse*
        response) {
  {
    absl::MutexLock l(resource_data_mtx_);
    auto health_data =
        resource_health_data_by_resource_name_.find(request->resource_name());
    if (health_data == resource_health_data_by_resource_name_.end()) {
      return ToGrpcStatus(absl::UnavailableError(
          absl::StrCat("No ServiceState data available for asset '",
                       request->resource_name(), "' (yet)")));
    }
  }
  return ToGrpcStatus(
      absl::UnavailableError("Cannot disable hardware module directly. They "
                             "are disabled automatically when an error is "
                             "detected."));
}

::grpc::Status ServiceStateAggregator::ClearFaults(
    ::grpc::ServerContext* context,
    const ::intrinsic_proto::simulation::AggregatedResourceClearFaultsRequest*
        request,
    ::intrinsic_proto::simulation::AggregatedResourceClearFaultsResponse*
        response) {
  {
    absl::MutexLock l(resource_data_mtx_);
    auto health_data =
        resource_health_data_by_resource_name_.find(request->resource_name());
    if (health_data == resource_health_data_by_resource_name_.end()) {
      return ToGrpcStatus(absl::UnavailableError(
          absl::StrCat("No ServiceState data available for asset '",
                       request->resource_name(), "' (yet)")));
    }
    health_data->second.clear_faults_requested = true;
  }
  // Wait (at most until the deadline) for Gazebo to process the ClearFaults
  // command.
  while (absl::Now() < absl::FromChrono(context->deadline())) {
    absl::MutexLock l(resource_data_mtx_);
    auto health_data =
        resource_health_data_by_resource_name_.find(request->resource_name());
    if (health_data == resource_health_data_by_resource_name_.end()) {
      return ToGrpcStatus(absl::UnavailableError(
          absl::StrCat("No ServiceState data available for asset '",
                       request->resource_name(), "' (yet)")));
    }
    if (health_data->second.clear_faults_requested == false) {
      return ::grpc::Status::OK;
    }
  }
  return ToGrpcStatus(absl::DeadlineExceededError(
      absl::StrCat("Asset '", request->resource_name(),
                   "' did not complete ClearFaults before the deadline.")));
}

}  // namespace intrinsic::simulation
