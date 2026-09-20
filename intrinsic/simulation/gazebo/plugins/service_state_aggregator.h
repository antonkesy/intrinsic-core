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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SERVICE_STATE_AGGREGATOR_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SERVICE_STATE_AGGREGATOR_H_

#include <memory>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "grpcpp/server.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Types.hh"
#include "intrinsic/simulation/gazebo/components/service_state_component.h"
#include "intrinsic/simulation/gazebo/plugins/aggregated_resource_health.grpc.pb.h"
#include "intrinsic/simulation/gazebo/plugins/aggregated_resource_health.pb.h"
#include "sdf/Element.hh"

namespace intrinsic::simulation {

// Top-level Gazebo System that
// * collects ResourceHealth data from models that expose it in the ECM
// * makes that data available via a gRPC service
// * relays ClearFault requests from the gRPC service to the corresponding
//   Gazebo model
//
// The intended use case is for simulated hardware modules using GazeboHwm: In
// this setup, the hardware module implementation lives in the gzserver pod, but
// other parts of the system (like the Equipment Manager) still look for the
// corresponding ResourceHealth service in the hardware module's pod (either via
// in-cluster DNS or the ingress service). ServiceStateAggregator allows the
// resource pod to offer a ResourceHealth service that relays data and commands
// to/from the gzserver pod.
class ServiceStateAggregator final
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemConfigurePriority,
      public gz::sim::ISystemPreUpdate,
      public gz::sim::ISystemPostUpdate,
      public ::intrinsic_proto::simulation::AggregatedResourceHealth::Service {
 public:
  static constexpr char kServiceAddressEnvironmentVariable[] =
      "HEALTH_AGGREGATOR_SERVICE_ADDRESS";

  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const ::sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& event_manager) override;
  // Configures this system to execute before the HardwareModuleLauncher system.
  gz::sim::System::PriorityType ConfigurePriority() override;
  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override;
  void PostUpdate(const gz::sim::UpdateInfo& info,
                  const gz::sim::EntityComponentManager& ecm) override;

  ::grpc::Status CheckHealth(
      ::grpc::ServerContext* context,
      const ::intrinsic_proto::simulation::
          AggregatedResourceHealthStatusRequest* request,
      ::intrinsic_proto::simulation::AggregatedResourceHealthStatusResponse*
          response) override;

  ::grpc::Status Enable(
      ::grpc::ServerContext* context,
      const ::intrinsic_proto::simulation::AggregatedResourceEnableRequest*
          request,
      ::intrinsic_proto::simulation::AggregatedResourceEnableResponse* response)
      override;

  ::grpc::Status Disable(
      ::grpc::ServerContext* context,
      const ::intrinsic_proto::simulation::AggregatedResourceDisableRequest*
          request,
      ::intrinsic_proto::simulation::AggregatedResourceDisableResponse*
          response) override;

  ::grpc::Status ClearFaults(
      ::grpc::ServerContext* context,
      const ::intrinsic_proto::simulation::AggregatedResourceClearFaultsRequest*
          request,
      ::intrinsic_proto::simulation::AggregatedResourceClearFaultsResponse*
          response) override;

 private:
  absl::Mutex resource_data_mtx_;
  absl::flat_hash_map<std::string, ::intrinsic::simulation::ServiceStateData>
      resource_health_data_by_resource_name_
          ABSL_GUARDED_BY(resource_data_mtx_);
  std::unique_ptr<grpc::Server> aggregated_resource_health_server_;
};

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SERVICE_STATE_AGGREGATOR_H_
