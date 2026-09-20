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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_ADD_HARDWARE_MODULE_LAUNCHERS_SYSTEM_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_ADD_HARDWARE_MODULE_LAUNCHERS_SYSTEM_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_set.h"
#include "absl/functional/function_ref.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/System.hh"
#include "sdf/Element.hh"

namespace intrinsic::simulation {

// This system uses both the Gazebo ECM and the Intrinsic Resource Registry
// service to add HardwareModuleLauncher plugins for all entities that
// correspond to an Intrinsic hardware module (HWM).
//
// It does this by finding entities in its `Configure()` function where
// * the entity has the intrinsic::simulation::ResourceName component
// * the ResourceName is a valid name in the Resource Registry
// * the resource instance's configuration type is
//   `intrinsic_proto.icon.HardwareModuleConfig`
//
// `AddHardwareModuleLaunchersSystem::Configure()` saves the entities so that
// `AddHardwareModuleLaunchersSystem::AddHardwareModuleLaunchers()` can use them
// later.
//
// The intended use case for this is:
// 1. Set up a SimulationServer (including a WorldModelConfigPlugin, since
//    plugin populates the ResourceName component!)
// 2. Build an `AddHardwareModuleLaunchersSystem`
// 3. Call `server.AddSystem()` with that system. This invokes
//    `AddHardwareModuleLaunchersSystem::Configure()`!
// 4. Use `AddHardwareModuleLaunchersSystem::AddHardwareModuleLaunchers(server)`
//    to add the `HardwareModuleLauncher` system to those entities that
//    correspond to a HWM.
//
// Note that this *doesn't* include loading `AddHardwareModuleLaunchersSystem`
// from a plugin tag, so there is no `_register` target that registers the
// system as a plugin. This is by design.
//
// This is a bit circuitous because of the `gazebo::Server` API, because we
// want to report errors nicely:
// * One cannot access the Entity Component Manager (ECM) from outside of a
//   system's methods (like `Configure()`).
// * It's not straightforward to add a new System to an entity from within the
//   `Configure()` method. It may not be *intended* to do so, in fact.
// * The `server.AddSystem()` function does not have a facility to relay system
//   configuration errors to the caller.
class AddHardwareModuleLaunchersSystem final
    : public ::gz::sim::System,
      public ::gz::sim::ISystemConfigure {
 public:
  void Configure(const ::gz::sim::Entity& entity,
                 const std::shared_ptr<const ::sdf::Element>& sdf,
                 ::gz::sim::EntityComponentManager& ecm,
                 ::gz::sim::EventManager& event_manager) override;

  // Adds a `HardwareModuleLauncher` system to each entity that `Configure()`
  // identified as corresponding to a simulated HWM by calling `add_system_fn`.
  // The passed `add_system_fn` is expected to wrap the
  // `gz::sim::Server::AddSystem()` method.
  //
  // Returns FailedPrecondition if called before `Configure()` finishes.
  absl::Status AddHardwareModuleLaunchers(
      absl::FunctionRef<absl::Status(std::shared_ptr<::gz::sim::System> system,
                                     ::gz::sim::Entity entity)>
          add_system_fn) const;

 private:
  mutable absl::Mutex m_;
  absl::StatusOr<absl::flat_hash_set<std::string>>
      hardware_module_resource_names_ ABSL_GUARDED_BY(m_) =
          absl::FailedPreconditionError(
              "You must call AddHardwareModuleLaunchersSystem::Configure() "
              "first!");
  absl::StatusOr<std::vector<gz::sim::Entity>> hardware_module_model_entities_
      ABSL_GUARDED_BY(m_) = absl::FailedPreconditionError(
          "You must call AddHardwareModuleLaunchersSystem::Configure() first!");
};

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_ADD_HARDWARE_MODULE_LAUNCHERS_SYSTEM_H_
