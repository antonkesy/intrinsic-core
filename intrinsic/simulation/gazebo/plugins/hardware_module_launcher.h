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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_HARDWARE_MODULE_LAUNCHER_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_HARDWARE_MODULE_LAUNCHER_H_

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/flags/declare.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "grpcpp/server.h"
#include "gz/common/events/Types.hh"
#include "gz/msgs/MessageTypes.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Types.hh"
#include "gz/transport/Node.hh"
#include "intrinsic/icon/hal/hardware_module_health_service.h"
#include "intrinsic/icon/hal/hardware_module_runtime.h"
#include "intrinsic/icon/hal/hardware_module_util.h"
#include "intrinsic/icon/hal/proto/hardware_module_config.pb.h"
#include "intrinsic/simulation/gazebo/asset_instances_client.h"
#include "intrinsic/simulation/gazebo/plugins/gazebo_hwm.h"
#include "intrinsic/simulation/gazebo/plugins/gravity_compensator.h"
#include "intrinsic/simulation/gazebo/plugins/sim_hardware_interface_data.h"
#include "intrinsic/simulation/gazebo/plugins/sim_hardware_module_config.pb.h"
#include "intrinsic/util/thread/thread.h"
#include "sdf/Element.hh"

ABSL_DECLARE_FLAG(std::string, shared_memory_namespace_testonly);
ABSL_DECLARE_FLAG(bool, use_sim_velocity_control);

namespace intrinsic::simulation {

constexpr char kGazeboHwmName[] = "gazebo_hwm";

// HardwareModuleLauncher is a plugin added to each Gazebo model that
// corresponds to an Intrinsic hardware module (HWM). See the documentation of
// intrinsic::simulation::AddHardwareModuleLaunchersSystem for details about how
// this plugin is added.
//
// This plugin is responsible for:
// * Setting up the GazeboHardwareModule to run within the Gazebo process so
//   that data can be exchanged via shared memory using the Icon hardware
//   interfaces specified in SimHardwareModuleConfig.hardware_interfaces
//   (see intrinsic::simulation::InferSimHardwareModuleConfig for details).
// * Reading sensor values from the Gazebo ECM during PreUpdate() and writing
//   them to the ICON shared memory interface.
// * Reading hardware commands from the ICON shared memory interface during
//   PreUpdate() and writing them to the Gazebo ECM.
// * Advancing the ICON clock as needed during Update(). If the ICON control
//   period is identical to the Gazebo step size, there should be one ICON step
//   requested during each Update() call. If they are different, there may be
//   zero, one, or multiple ICON steps requested during each Update() call.
//
// This split of functionality between PreUpdate() and Update() allows multiple
// HardwareModuleLauncher instances to run in a single Gazebo process. The
// ConfigurePriority() method ensures that the ICON ticks are requested in
// Update() before the Physics system Update() runs to ensure that the ICON
// tick and Physics updates are run simultaneously in separate threads.
class HardwareModuleLauncher final : public ::gz::sim::System,
                                     public ::gz::sim::ISystemConfigure,
                                     public ::gz::sim::ISystemConfigurePriority,
                                     public ::gz::sim::ISystemPreUpdate,
                                     public ::gz::sim::ISystemUpdate {
 public:
  ~HardwareModuleLauncher() override;

  void Configure(const ::gz::sim::Entity& entity,
                 const std::shared_ptr<const ::sdf::Element>& sdf,
                 ::gz::sim::EntityComponentManager& ecm,
                 ::gz::sim::EventManager& event_manager) override;
  // Configures this system to execute before the Physics system.
  ::gz::sim::System::PriorityType ConfigurePriority() override;
  // This behaves differently on first call vs all other invocations.
  //
  // Normally, this function
  // * waits for the HWM thread to signal that ICON is done reading from/writing
  //   to the HWM
  // * copies the current state of all hardware interfaces from the Gazebo ECM
  //   into the ICON shared memory interface. Note that this is the state from
  //   the end of the _previous_ simulation step (since we're only about to
  //   enter the next Update() step _after_ this function returns)
  // * handles the commands that ICON has written to the hardware interfaces, to
  //   prepare for the upcoming sim Update() step
  //
  // On the _first_ call, this function additionally launches the actual HWM
  // thread.
  //
  // We don't launch the HWM in Configure() because we rely on
  // WorldModelConfigPlugin to tell us the resource name that our Gazebo model
  // corresponds to, and there's no guarantee that WorldModelConfigPlugin's
  // Configure() method runs before ours.
  //
  // But there *is* a guarantee that PreUpdate() only runs after *all*
  // Configure() methods finish.
  void PreUpdate(const ::gz::sim::UpdateInfo& info,
                 ::gz::sim::EntityComponentManager& ecm) override;
  void Update(const ::gz::sim::UpdateInfo& info,
              ::gz::sim::EntityComponentManager& ecm) override;

  struct HardwareModuleRuntimeData {
    intrinsic_proto::icon::HardwareModuleConfig hwm_config;
    std::string service_inspection_topic;
    absl::Status init_error;
    absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>
        joint_groups_by_name;
    absl::flat_hash_set<std::string> joint_groups_with_reset_commands;
    absl::flat_hash_map<std::string,
                        std::vector<std::unique_ptr<GravityCompensator>>>
        gravity_compensators_for_braking_by_joint_group;
    absl::flat_hash_map<gz::sim::Entity, std::string>
        rangefinder_topic_by_entity;

    absl::flat_hash_map<std::string,
                        hardware_interface_data::HardwareInterfaceData>
        hardware_interface_name_to_gazebo_data;
    std::unique_ptr<grpc::Server> grpc_server;
    // HardwareModuleRuntime cannot be moved or copied. To give
    // HardwareModuleRuntimeData value semantics, we wrap it in a unique_ptr
    // here.
    absl_nonnull std::unique_ptr<icon::HardwareModuleRuntime> runtime;
    // `runtime` owns the object this points to, but we need to access it to
    // update the GazeboHwm with new data from the simulation (and vice versa).
    GazeboHardwareModule* absl_nonnull gazebo_hwm_rawptr;
    absl::Duration control_period;
  };

 private:
  //  Apply commands from the HWM shared memory interface(s) to the Gazebo ECM
  //  here (they are based on the previous Gazebo tick's sensor values)
  void HandleHwmCommands(
      const ::gz::sim::UpdateInfo& info, ::gz::sim::EntityComponentManager& ecm,
      GazeboHardwareModule::HardwareInterfaces& hardware_interfaces)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(hardware_interfaces.mutex);

  // Copy status (sensor readings, joint positions etc) from Gazebo ECM to HWM
  // shared memory interface(s) here.
  void UpdateHwmSensorValues(
      const ::gz::sim::UpdateInfo& info,
      const ::gz::sim::EntityComponentManager& ecm,
      GazeboHardwareModule::HardwareInterfaces& hardware_interfaces)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(hardware_interfaces.mutex);

  void OnStop();

  // This is a shared promise that is used to signal the HWM to restart. This is
  // given to the HWM runtime and ServiceState grpc service.
  std::shared_ptr<icon::SharedPromiseWrapper<icon::HardwareModuleExitCode>>
      shared_exit_code_promise_ = std::make_shared<
          icon::SharedPromiseWrapper<icon::HardwareModuleExitCode>>();
  // This is set to true when the ServiceState service or ICON wants the HWM to
  // be restarted in the next PreUpdate call.
  std::atomic_bool restart_hwm_ = false;
  // Thread that observes the shared_exit_code_promise_ for restart requests.
  intrinsic::Thread restart_request_observer_thread_;
  // This is optional to defer creation until we first initialize the simulated
  // HWM. After that, we can also use `optional` to re-initialize the health
  // service when we restart the GazeboHwm (for example due to clearing a fatal
  // initialization fault).
  std::optional<icon::HardwareModuleHealthService> health_service_;

  absl::StatusOr<HardwareModuleRuntimeData> gazebo_hwm_ =
      absl::UnavailableError("Gazebo HWM not initialized yet");
  gz::sim::Model model_;
  // Used when Gazebo's period is less than the `control_period_`.
  // Think of this as "how far Gazebo is ahead of ICON".
  // It is literally the difference between ICON's clock and Gazebo's.
  // We accumulate time until this is >= `control_period_`, and then request an
  // ICON tick.
  absl::Duration gazebo_icon_time_delta_ = absl::ZeroDuration();

  // Used to shut down HWMs on stop.
  gz::common::ConnectionPtr stop_connection_;
  // This tracks the first PreUpdate call, where we initialize the actual HWM
  // process.
  bool first_preupdate_ = true;
  bool use_sim_velocity_control_ = false;
  absl::Mutex rangefinder_data_mtx_;
  absl::flat_hash_map<gz::sim::Entity, gz::msgs::LaserScan>
      rangefinder_data_by_entity_ ABSL_GUARDED_BY(rangefinder_data_mtx_);
  // Gazebo transport node used to subscribe to the rangefinder topics.
  // TODO(b/391456177): get rangefinder data directly from the ECM.
  // Since the subscribe callback passed to the `Node` uses
  // `rangefinder_data_mtx_` and `rangefinder_data_by_entity_`, those variables
  // must outlive the `Node`. As such, `node_` is defined after those variables
  // to ensure that it is destroyed first.
  gz::transport::Node node_;

  absl::StatusOr<absl_nonnull std::unique_ptr<AssetInstancesClient>>
      asset_instances_client_;
};
}  // namespace intrinsic::simulation
#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_HARDWARE_MODULE_LAUNCHER_H_
