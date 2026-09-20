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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GAZEBO_HWM_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GAZEBO_HWM_H_

#include <atomic>
#include <memory>
#include <queue>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"
#include "gz/sim/Entity.hh"
#include "intrinsic/icon/control/realtime_clock_interface.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/flatbuffers/transform_types.fbs.h"
#include "intrinsic/icon/hal/command_validator.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_module_init_context.h"
#include "intrinsic/icon/hal/hardware_module_interface.h"
#include "intrinsic/icon/hal/interfaces/force_torque.fbs.h"
#include "intrinsic/icon/hal/interfaces/hardware_module_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/io_controller.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_limits.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/rangefinder.fbs.h"
#include "intrinsic/icon/hardware_modules/sim_bus/sim_bus_hardware_module.pb.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/simulation/gazebo/plugins/sim_hardware_interface_data.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/util/thread/thread_options.h"

namespace intrinsic::simulation {

// `GazeboHardwareModule` is a generic sim HWM that is parameterized and
// instantiated by the `HardwareModuleLauncher` Gazebo plugin.
// It is responsible for relaying commands from ICON to Gazebo, and for reading
// status from Gazebo and relaying it to ICON.
// The interaction between the Gazebo server and a `GazeboHardwareModule` is
// mediated by the owning `HardwareModuleLauncher` instance.
// Please refer to the `HardwareModuleLauncher` class documentation for more
// details.
// intrinsic/simulation/gazebo/plugins/hardware_module_launcher.h
class GazeboHardwareModule final : public icon::HardwareModuleInterface {
 public:
  struct Config {
    // If this is set, something went wrong during the initialization phase
    // (i.e. before creating a GazeboHardwareModule instance).
    //
    // If this happens, we want to "forward" the error to
    // `HardwareModuleRuntime` by returning it from `Init()`.
    absl::Status init_error;
    // GazeboHardwareModule uses the following members to determine which
    // hardware interfaces to register in its `Init()` method, and what
    // parameters to pass to those interfaces' constructors.
    //
    // These references are only used in `Init()`, but to be safe, the maps they
    // point to should outlive the GazeboHardwareModule.
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name;
    const absl::flat_hash_map<std::string,
                              hardware_interface_data::HardwareInterfaceData>&
        hardware_interface_name_to_gazebo_data;
    std::string hwm_name;
  };

  struct HardwareInterfaces {
    absl::Mutex mutex;

    // LINT.IfChange(interface_handle_types)
    using InterfaceHandleTypes = std::variant<
        icon::StrictHardwareInterfaceHandle<
            intrinsic_fbs::JointPositionCommand>,
        icon::HardwareInterfaceHandle<intrinsic_fbs::JointPositionCommand>,
        icon::StrictHardwareInterfaceHandle<intrinsic_fbs::JointTorqueCommand>,
        icon::HardwareInterfaceHandle<intrinsic_fbs::JointTorqueCommand>,
        icon::MutableHardwareInterfaceHandle<intrinsic_fbs::JointPositionState>,
        icon::MutableHardwareInterfaceHandle<
            intrinsic_fbs::JointCommandedPosition>,
        icon::MutableHardwareInterfaceHandle<intrinsic_fbs::JointVelocityState>,
        icon::MutableHardwareInterfaceHandle<
            intrinsic_fbs::JointAccelerationState>,
        icon::MutableHardwareInterfaceHandle<intrinsic_fbs::JointTorqueState>,
        icon::HardwareInterfaceHandle<intrinsic_fbs::PayloadCommand>,
        icon::MutableHardwareInterfaceHandle<intrinsic_fbs::PayloadState>,
        icon::HardwareInterfaceHandle<intrinsic_fbs::Wrench>,
        icon::MutableHardwareInterfaceHandle<intrinsic_fbs::DIOStatus>,
        icon::HardwareInterfaceHandle<intrinsic_fbs::DIOCommand>,
        icon::HardwareInterfaceHandle<intrinsic_fbs::ForceTorqueCommand>,
        icon::MutableHardwareInterfaceHandle<intrinsic_fbs::ForceTorqueStatus>,
        icon::MutableHardwareInterfaceHandle<intrinsic_fbs::AIOStatus>,
        icon::HardwareInterfaceHandle<intrinsic_fbs::AIOCommand>,
        icon::MutableHardwareInterfaceHandle<
            intrinsic_fbs::SafetyStatusMessage>,
        icon::HardwareInterfaceHandle<intrinsic_fbs::JointLimits>,
        icon::MutableHardwareInterfaceHandle<intrinsic_fbs::RangeFinderStatus>>;
    // LINT.ThenChange(//intrinsic/simulation/gazebo/plugins/gazebo_hwm.cc:register_interfaces)

    // Stores handles to the hardware interfaces that the GazeboHardwareModule
    // supports (i.e. those that it sets up in its `Init()` function)
    class Handles {
     public:
      Handles() = default;
      explicit Handles(
          absl::flat_hash_map<
              std::string,
              GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
              interface_name_to_handle)
          : interface_name_to_handle_(std::move(interface_name_to_handle)) {}

      // Returns a pointer to a HardwareInterfaceHandle<T> if `name` is the name
      // of a non-strict, non-mutable hardware interface with flatbuffer message
      // type `T`.
      //
      // Returns NotFoundError if `name` is not the name of *any* hardware
      // interface.
      // Returns InvalidArgumentError if the hardware interface `name` is
      // strict, mutable, or does not have the message type `T`.
      template <class T>
      absl::StatusOr<const icon::HardwareInterfaceHandle<T>* absl_nonnull>
      GetInterface(absl::string_view name) const;

      // Returns a pointer to a HardwareInterfaceHandle<T> if `name` is the name
      // of a strict, non-mutable hardware interface with flatbuffer message
      // type `T`.
      //
      // Returns NotFoundError if `name` is not the name of *any* hardware
      // interface.
      // Returns InvalidArgumentError if the hardware interface `name` is
      // non-strict, mutable, or does not have the message type `T`.
      template <class T>
      absl::StatusOr<const icon::StrictHardwareInterfaceHandle<T>* absl_nonnull>
      GetStrictInterface(absl::string_view name) const;

      // Returns a pointer to a HardwareInterfaceHandle<T> if `name` is the name
      // of a non-strict, mutable hardware interface with flatbuffer message
      // type `T`.
      //
      // Returns NotFoundError if `name` is not the name of *any* hardware
      // interface.
      // Returns InvalidArgumentError if the hardware interface `name` is
      // strict, non-mutable, or does not have the message type `T`.
      template <class T>
      absl::StatusOr<icon::MutableHardwareInterfaceHandle<T>* absl_nonnull>
      GetMutableInterface(absl::string_view name);

      // Returns a pointer to a HardwareInterfaceHandle<T> if `name` is the name
      // of a strict, mutable hardware interface with flatbuffer message
      // type `T`.
      //
      // Returns NotFoundError if `name` is not the name of *any* hardware
      // interface.
      // Returns InvalidArgumentError if the hardware interface `name` is
      // non-strict, non-mutable, or does not have the message type `T`.
      template <class T>
      absl::StatusOr<
          icon::MutableStrictHardwareInterfaceHandle<T>* absl_nonnull>
      GetMutableStrictInterface(absl::string_view name);

     private:
      absl::flat_hash_map<std::string, InterfaceHandleTypes>
          interface_name_to_handle_;
    };
    Handles handles ABSL_GUARDED_BY(mutex);

    // The command validator is used by ApplyInterfaceDataToEcmVisitor in
    // HardwareModuleLauncher.
    intrinsic::icon::Validator command_validator ABSL_GUARDED_BY(mutex);
  };

  explicit GazeboHardwareModule(Config config);
  ~GazeboHardwareModule() override;

  std::string Name() const { return config_.hwm_name; }
  // Blocks until all requested ticks have finished.
  //
  // No-op if this HWM is not a clock driver (i.e. we rely on another HWM to
  // regulate ICON's tick rate).
  icon::RealtimeStatus WaitForIconTicksToFinish();

  // Requests ICON to tick once. Does not block while ICON processes the tick.
  // In fact, you can call this multiple times and it will queue additional
  // ticks.
  // Internally, each requested tick allows the clock ticking thread to call
  // `TickBlockingWithTimeout()` once, which causes ICON to read from and write
  // to the HWM's interfaces.
  //
  // No-op if this HWM is not a clock driver (i.e. we rely on another HWM to
  // regulate ICON's tick rate).
  icon::RealtimeStatus RequestIconTick(::intrinsic::Time sim_time);

  HardwareInterfaces& GetHardwareInterfaces() { return hardware_interfaces_; }

  absl::Status Init(
      intrinsic::icon::HardwareModuleInitContext& init_context) override;
  absl::Status Prepare() override;
  icon::RealtimeStatus Activate() override;
  icon::RealtimeStatus Deactivate() override;
  absl::Status EnableMotion() override;
  icon::RealtimeStatus Enabled() override;
  icon::RealtimeStatus Disabled() override;
  absl::Status DisableMotion() override;
  absl::Status ClearFaults() override;
  absl::Status Shutdown() override;
  // Blocks unless unlocked by AllowReadStatusOnce()
  icon::RealtimeStatus ReadStatus() override;
  // Blocks unless unlocked by AllowApplyCommandOnce()
  icon::RealtimeStatus ApplyCommand() override;
  absl::Status ProvideInspectionData(
      intrinsic_proto::icon::v1::HardwareModuleInspectionData& data) override;

  bool IsActive() const { return active_; }
  bool IsEnabled() const { return enabled_; }

 private:
  bool NewSimTimeOrShutdown() ABSL_EXCLUSIVE_LOCKS_REQUIRED(sim_clock_mutex_);
  bool SimTimeQueueEmptyOrShutdown()
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(sim_clock_mutex_);
  bool NonZeroExpectedReadStatusCallsOrShutdown()
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(read_status_calls_mutex_);
  bool NoMoreExpectedReadStatusCallsOrShutdown()
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(read_status_calls_mutex_);
  bool ReadyToStepOrShutdown() ABSL_EXCLUSIVE_LOCKS_REQUIRED(sim_clock_mutex_);
  void DeactivateAndClearQueues();

  intrinsic_fbs::StateCode state_ ABSL_GUARDED_BY(sim_clock_mutex_) =
      intrinsic_fbs::StateCode::kDeactivated;

  // If `realtime_clock` is set, we need to drive the ICON clock.
  //
  //  Since `realtime_clock->TickBlockingWithTimeout()` blocks, we don't want to
  // call it in either PreUpdate() or PostUpdate() (so we don't block other
  // systems from running).
  //
  // That's why we spin up a thread that runs this function, which just loops
  // and calls `TickBlockingWithTimeout()` once the Gazebo tick is done (see
  // `StartIconTick()`).
  //
  // Use a combination of `shutdown_.Notify()` and `realtime_clock_->Reset()` to
  // tell this thread to stop.
  void TickRealtimeClock();

  std::atomic_bool active_ = false;
  std::atomic_bool enabled_ = false;
  const Config config_;
  icon::RealtimeClockInterface* realtime_clock_ = nullptr;
  ::intrinsic::Thread tick_realtime_clock_thread_;
  ::intrinsic::ThreadOptions tick_realtime_clock_thread_options_;
  HardwareInterfaces hardware_interfaces_;
  absl::Mutex sim_clock_mutex_;
  // Notification is thread safe, but we use Mutex::AwaitWithTimeout() to wait
  // for `NewSimTimeOrShutdown()` and `SimTimeQueueEmptyOrShutdown()`.
  // `Mutex::AwaitWithTimeout()` wakes the waiting thread (roughly) whenever
  // another thread releases the mutex. We need to make sure that any threads
  // waiting on these functions get woken up when we signal `shutdown_`, and
  // marking it as ABSL_GUARDED_BY(sim_clock_mutex_) forces users to acquire
  // (and subsequently release) `sim_clock_mutex_`.
  std::unique_ptr<absl::Notification> shutdown_
      ABSL_GUARDED_BY(sim_clock_mutex_);
  std::queue<::intrinsic::Time> sim_time_queue_
      ABSL_GUARDED_BY(sim_clock_mutex_);
  // Ensure that mutexes are acquired in a consistent order.
  absl::Mutex read_status_calls_mutex_ ABSL_ACQUIRED_BEFORE(sim_clock_mutex_);
  // Similar to above, we need to make sure that any threads waiting on
  // `num_expected_read_status_calls_` get woken up when we signal
  // `read_status_shutdown_`.
  std::unique_ptr<absl::Notification> read_status_shutdown_
      ABSL_GUARDED_BY(read_status_calls_mutex_);
  // We expect this many calls to `ReadStatus()` before we block again and wait
  // for the next simulation step.
  int num_expected_read_status_calls_
      ABSL_GUARDED_BY(read_status_calls_mutex_) = 0;
};

namespace internal {
template <class HandleT>
absl::StatusOr<absl_nonnull std::add_pointer_t<HandleT>> LookupInterface(
    absl::string_view name,
    absl::flat_hash_map<
        std::string,
        GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>&
        interface_name_to_handle) {
  auto interface_any = interface_name_to_handle.find(name);
  if (interface_any == interface_name_to_handle.end()) {
    return absl::NotFoundError(absl::StrCat(
        "Hardware interface '", name,
        "' does not exist in simulation. Check your (sim) "
        "configuration if you think that it should.. Available interfaces: [",
        absl::StrJoin(interface_name_to_handle, ", ",
                      [](std::string* os, const auto& name_and_signature) {
                        absl::StrAppend(os, name_and_signature.first);
                      }),
        "]"));
  }
  // std::get_if<T>(variant) returns a non-null T* if `variant` is a `T`, and
  // `nullptr` if it is not.
  auto* interface_handle = std::get_if<HandleT>(&(interface_any->second));
  if (interface_handle == nullptr) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Hardware interface '", name,
        "' does not have the requested type. This could be because you're "
        "trying to access a mutable interface as immutable (or vice versa), a "
        "strict interface as non-strict (or vice versa), or because you're "
        "asking for the wrong flatbuffer message type."));
  }
  return interface_handle;
}

template <class HandleT>
absl::StatusOr<absl_nonnull std::add_pointer_t<std::add_const_t<HandleT>>>
LookupInterface(
    absl::string_view name,
    const absl::flat_hash_map<
        std::string,
        GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>&
        interface_name_to_handle) {
  auto interface_any = interface_name_to_handle.find(name);
  if (interface_any == interface_name_to_handle.end()) {
    return absl::NotFoundError(absl::StrCat(
        "Hardware interface '", name,
        "' does not exist in simulation. Check your (sim) "
        "configuration if you think that it should. Available interfaces: [",
        absl::StrJoin(interface_name_to_handle, ", ",
                      [](std::string* os, const auto& name_and_signature) {
                        absl::StrAppend(os, name_and_signature.first);
                      }),
        "]"));
  }
  // std::get_if<T>(variant) returns a non-null T* if `variant` is a `T`, and
  // `nullptr` if it is not.
  auto* interface_handle = std::get_if<HandleT>(&(interface_any->second));
  if (interface_handle == nullptr) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Hardware interface '", name,
        "' does not have the requested type. This could be because you're "
        "trying to access a mutable interface as immutable (or vice versa), a "
        "strict interface as non-strict (or vice versa), or because you're "
        "asking for the wrong flatbuffer message type."));
  }
  return interface_handle;
}

}  // namespace internal

template <class T>
absl::StatusOr<const icon::HardwareInterfaceHandle<T>* absl_nonnull>
GazeboHardwareModule::HardwareInterfaces::Handles::GetInterface(
    absl::string_view name) const {
  return internal::LookupInterface<icon::HardwareInterfaceHandle<T>>(
      name, interface_name_to_handle_);
}

template <class T>
absl::StatusOr<const icon::StrictHardwareInterfaceHandle<T>* absl_nonnull>
GazeboHardwareModule::HardwareInterfaces::Handles::GetStrictInterface(
    absl::string_view name) const {
  return internal::LookupInterface<icon::StrictHardwareInterfaceHandle<T>>(
      name, interface_name_to_handle_);
}

template <class T>
absl::StatusOr<icon::MutableHardwareInterfaceHandle<T>* absl_nonnull>
GazeboHardwareModule::HardwareInterfaces::Handles::GetMutableInterface(
    absl::string_view name) {
  return internal::LookupInterface<icon::MutableHardwareInterfaceHandle<T>>(
      name, interface_name_to_handle_);
}

template <class T>
absl::StatusOr<icon::MutableStrictHardwareInterfaceHandle<T>* absl_nonnull>
GazeboHardwareModule::HardwareInterfaces::Handles::GetMutableStrictInterface(
    absl::string_view name) {
  return internal::LookupInterface<
      icon::MutableStrictHardwareInterfaceHandle<T>>(name,
                                                     interface_name_to_handle_);
}

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GAZEBO_HWM_H_
