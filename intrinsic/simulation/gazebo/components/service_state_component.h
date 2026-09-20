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

#ifndef INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_SERVICE_STATE_COMPONENT_H_
#define INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_SERVICE_STATE_COMPONENT_H_

#include <optional>
#include <string>

#include "absl/status/statusor.h"
#include "gz/sim/components/Component.hh"
#include "intrinsic/assets/services/proto/v1/service_state.pb.h"

namespace intrinsic {
namespace simulation {

struct ServiceStateData {
  // If this is true at the start of a simulation step, the entity that the
  // ServiceStateComponent is attached to
  // 1. Clears any faults. This may cause a restart of the simulated resource,
  //   but the resource might fault again after restarting if the cause of the
  //   original fault still exists.
  // 2. Sets `clear_faults_requested` to false.
  bool clear_faults_requested = false;
  // The entity that the ServiceStateComponent is attached to fills this with
  // the current state once per simulation step.
  absl::StatusOr<intrinsic_proto::services::v1::SelfState> state;

  // These two variables are used to synchronize the clocks of the hwms that run
  // under a single ICON instance. They need to be synchronized to avoid
  // a deadlock that occurs when they start calling RequestIconTick() on
  // different cycles (i.e. there's a cycle where one HWM calls that method, but
  // the other doesn't).

  // Starts unset, then holds the cycle in which the corresponding
  // HardwareModuleLauncher was active during the Update call. We will wait to
  // start ticking the clock until all hardware modules under the icon instance
  // specified below are activated.
  std::optional<int> activated_cycle;
  // The name of the icon resource that controls the hardware module this object
  // is associated with. The name is not used to look up anything related to
  // ICON, but is treated as an opaque (but readable) id to identify which
  // hardware modules must be grouped together.
  std::string icon_resource_id;
  // The shared-memory name of the associated HWM. Note that this can be
  // different from the Gazebo model name *OR* the Intrinsic World resource
  // name...
  std::string hwm_name;
};

// Service state data for an entity. The entity should also have a
// intrinsic::simulation::ResourceName component!
using ServiceState =
    gz::sim::components::Component<ServiceStateData, class ServiceStateTag>;

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_SERVICE_STATE_COMPONENT_H_
