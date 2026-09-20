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

#ifndef INTRINSIC_ICON_HAL_HARDWARE_MODULE_PROXY_H_
#define INTRINSIC_ICON_HAL_HARDWARE_MODULE_PROXY_H_

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "intrinsic/icon/hal/get_hardware_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/icon_state_register.h"  // IWYU pragma: keep
#include "intrinsic/icon/hal/interfaces/hardware_module_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/icon_state.fbs.h"
#include "intrinsic/icon/hal/module_config.h"
#include "intrinsic/icon/interprocess/remote_trigger/remote_trigger_client.h"
#include "intrinsic/icon/interprocess/shared_memory_manager/domain_socket_utils.h"
#include "intrinsic/icon/interprocess/shared_memory_manager/memory_segment.h"
#include "intrinsic/icon/interprocess/shared_memory_manager/segment_info.fbs.h"
#include "intrinsic/icon/server/config/icon_main_config.pb.h"
#include "intrinsic/icon/utils/duration.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// The `HardwareModuleProxy` class connects to a `HardwareModule` across process
// boundaries. Given a module name, the class looks up the exported information
// from a hardware module and provides access to obtain handles to the hardware
// interfaces.
// The interface handles are shared between multiple processes and thus
// allow for a zero-copy data transfer. Exclusive access to those handles is
// obtained via calls to `ReadStatus` and `ApplyCommands`. During such a call,
// the hardware module has exclusive ownership of the shared interfaces and can
// thus safely perform any operation on that data.
class HardwareModuleProxy {
 public:
  // Creates a HardwareModuleProxy without establishing a connection to a
  // hardware module.
  // This allows to create proxies to hardware modules and wait for them to be
  // fully established.
  // After creating the object, one has to call `Connect()` in order to fetch
  // the data from the hardware module.
  static absl::StatusOr<HardwareModuleProxy> Create(
      const ModuleConfig& module_config);

  // Creates a HardwareModuleProxy without establishing a connection to a
  // hardware module.
  // The call behaves equivalent to `Create(const ModuleConfig&
  // module_config&)` by specifying only the module name.
  static absl::StatusOr<HardwareModuleProxy> Create(
      absl::string_view shared_memory_namespace, absl::string_view module_name);

  // Attaches to an existing hardware module and fetches all exported
  // information such as its hardware interfaces from it.
  // Returns a connected HardwareModuleProxy if successful, or fails otherwise.
  // Returns DeadlineExceeded if no connection can be established in
  // `connection_timeout`.
  static absl::StatusOr<HardwareModuleProxy> Attach(
      const ModuleConfig& module_config, absl::Duration connection_timeout);

  // Attaches to an existing hardware module and fetches all exported
  // information such as its hardware interfaces from it.
  // The call behaves equivalent to `Attach(const ModuleConfig&
  // module_config)` by specifying only the module name.
  // Returns DeadlineExceeded if no connection can be established in
  // `connection_timeout`.
  static absl::StatusOr<HardwareModuleProxy> Attach(
      absl::string_view shared_memory_namespace, absl::string_view module_name,
      absl::Duration connection_timeout);

  // Closes the file descriptors of the shared memory segments.
  ~HardwareModuleProxy();

  HardwareModuleProxy(const HardwareModuleProxy& other) = delete;

  // Custom move constructor required because of `used_interfaces_mutex_`.
  HardwareModuleProxy(HardwareModuleProxy&& other);

  HardwareModuleProxy& operator=(const HardwareModuleProxy& other) = delete;

  // Explicit move assignment operator.
  // See https://en.cppreference.com/w/cpp/language/rule_of_three#Rule_of_five
  HardwareModuleProxy& operator=(HardwareModuleProxy&& other) noexcept;

  // Returns the name of the hardware module the proxy object connects to.
  absl::string_view Name() const;

  // In case the proxy class did not automatically connect, a call to `Connect`
  // establishes a connection to the hardware module and fetches its
  // information.
  // Returns DeadlineExceeded if no connection can be established in
  // `connection_timeout`.
  absl::Status Connect(absl::Duration connection_timeout);

  // Returns whether the module is connected or not.
  bool IsConnected() const;

  // Returns the names and file descriptors of all segments that the connected
  // hardware module has advertised.
  absl::StatusOr<SegmentNameToFileDescriptorMap>
  GetSegmentNameToFileDescriptorMap() const;

  // Provides the list of all hardware interfaces names exported by the hardware
  // module.
  // Those names can then later be specified in a call to `GetHardwareInterface`
  // to obtain a handle to a shared hardware interface.
  std::vector<std::string> GetHardwareInterfaceNames() const;

  // Returns the list of hardware module interface names that are marked as
  // required.
  absl::StatusOr<std::vector<std::string>> GetRequiredInterfaceNames() const;

  // Returns the list of hardware module interface names that are marked as
  // required, but not used by ICON.
  // Empty if all required interfaces are used by the proxy.
  // An interface is used if GetHardwareInterface or GetMutableHardwareInterface
  // has been called for the interface.
  absl::StatusOr<std::vector<std::string>> GetUnusedRequiredInterfaceNames()
      const;

  // Provides read only access to the shared memory location specified by
  // `segment_name`.
  // Returns NotFoundError if the hardware module has not advertised such a
  // segment.
  // Forwards mapping errors.
  template <class T>
  absl::StatusOr<ReadOnlyMemorySegment<T>> GetReadOnlyMemorySegment(
      absl::string_view segment_name) const {
    if (!IsConnected()) {
      return absl::FailedPreconditionError(
          "HardwareModuleProxy is not connected to a hardware module. Did you "
          "forget to call Connect()?");
    }
    return ReadOnlyMemorySegment<T>::Get(segment_name_to_file_descriptor_map_,
                                         segment_name);
  }

  // Provides read write access to the shared memory location specified by
  // `segment_name`.
  // Returns NotFoundError if the hardware module has not advertised such a
  // segment.
  // Forwards mapping errors.
  template <class T>
  absl::StatusOr<ReadWriteMemorySegment<T>> GetReadWriteMemorySegment(
      absl::string_view segment_name) const {
    if (!IsConnected()) {
      return absl::FailedPreconditionError(
          "HardwareModuleProxy is not connected to a hardware module. Did you "
          "forget to call Connect()?");
    }
    return ReadWriteMemorySegment<T>::Get(segment_name_to_file_descriptor_map_,
                                          segment_name);
  }

  // Returns a non-mutable interface to a shared hardware interface.
  // Fails if the hardware interface is not in the list of exported interfaces
  // or the specified template type does not match the exported type by the
  // hardware module.
  template <class InterfaceT>
  absl::StatusOr<HardwareInterfaceHandle<InterfaceT>> GetHardwareInterface(
      absl::string_view interface_name) const {
    if (!IsConnected()) {
      return absl::FailedPreconditionError(
          "HardwareModuleProxy is not connected to a hardware module. Did you "
          "forget to call Connect()?");
    }
    if (std::find(interface_names_.begin(), interface_names_.end(),
                  interface_name) == interface_names_.end()) {
      return absl::NotFoundError(absl::StrCat(
          "Hardware module '", module_name_, "' does not export interface '",
          interface_name,
          "'. Only provides: ", absl::StrJoin(interface_names_, ", ")));
    }
    {
      absl::MutexLock l(&used_interfaces_mutex_);
      used_interfaces_.insert(std::string(interface_name));
    }
    return GetInterfaceHandle<InterfaceT>(segment_name_to_file_descriptor_map_,
                                          interface_name);
  }

  // Returns a mutable interface to a shared hardware interface.
  // Fails if the hardware interface is not in the list of exported interfaces
  // or the specified template type does not match the exported type by the
  // hardware module.
  template <class InterfaceT>
  absl::StatusOr<MutableHardwareInterfaceHandle<InterfaceT>>
  GetMutableHardwareInterface(absl::string_view interface_name) const {
    if (!IsConnected()) {
      return absl::FailedPreconditionError(
          "HardwareModuleProxy is not connected to a hardware module. Did you "
          "forget to call Connect()?");
    }
    if (std::find(interface_names_.begin(), interface_names_.end(),
                  interface_name) == interface_names_.end()) {
      return absl::NotFoundError(absl::StrCat(
          "Hardware module '", module_name_, "' does not export interface '",
          interface_name,
          "'. Only provides: ", absl::StrJoin(interface_names_, ", ")));
    }
    {
      absl::MutexLock l(&used_interfaces_mutex_);
      used_interfaces_.insert(std::string(interface_name));
    }
    return GetMutableInterfaceHandle<InterfaceT>(
        segment_name_to_file_descriptor_map_, interface_name);
  }

  // Triggers a call to `Prepare` on the hardware module.
  absl::Status Prepare(absl::Time deadline) const;

  // Triggers a call to `Activate` on the hardware module.
  RealtimeStatus Activate(absl::Time deadline) const;
  // Triggers a call to `Deactivate` on the hardware module.
  RealtimeStatus Deactivate(absl::Time deadline) const;

  // Triggers an async call to `Prepare`.
  RealtimeStatusOr<RemoteTriggerClient::AsyncRequest> PrepareAsync() const;
  // Triggers an async call to `EnableMotion`.
  RealtimeStatusOr<RemoteTriggerClient::AsyncRequest> EnableMotionAsync() const;
  // Triggers an async call to `DisableMotion`.
  RealtimeStatusOr<RemoteTriggerClient::AsyncRequest> DisableMotionAsync()
      const;
  // Triggers an async call to `ClearFaults`.
  RealtimeStatusOr<RemoteTriggerClient::AsyncRequest> ClearFaultsAsync() const;
  // Triggers an async call to `ActivateAsync`.
  RealtimeStatusOr<RemoteTriggerClient::AsyncRequest> ActivateAsync() const;
  // Triggers an async call to `Deactivate`.
  RealtimeStatusOr<RemoteTriggerClient::AsyncRequest> DeactivateAsync() const;

  // Triggers a call to `ReadStatus` on the hardware module.
  // The status hardware interfaces are being updated during the time of the
  // call.
  RealtimeStatus ReadStatus(absl::Time deadline) const;
  RealtimeStatusOr<RemoteTriggerClient::AsyncRequest> ReadStatusAsync() const;
  // Triggers a call to `ApplyCommand` on the hardware module.
  // Specified command values are being applied on the hardware.
  // Updates `current_cycle` in the special `icon_state` segment.
  RealtimeStatus ApplyCommand(absl::Time deadline);
  RealtimeStatusOr<RemoteTriggerClient::AsyncRequest> ApplyCommandAsync();

  // Triggers a call to `Restart` on the hardware module and blocks until the
  // request is completed. Note: Does not wait until the hardware module is
  // restarted, but only until the request to restart the hardware module is
  // completed.
  //
  // Returns FailedPreconditionError if the hardware module is currently
  // enabled. Since this is only a trigger, if the hardware module enables after
  // sending the request, the call will fail silently.
  absl::Status Restart(absl::Time deadline) const;
  // Triggers an async call to `Restart`.
  //
  // Returns an AsyncRequest that can be used to wait for the request to
  // complete.
  // Returns FailedPreconditionError if the hardware module is currently
  // enabled.
  RealtimeStatusOr<RemoteTriggerClient::AsyncRequest> RestartAsync() const;

  // Returns the state of the hardware module.
  // The current state of the hardware module can either be a transition state
  // such as "Activating", "MotionEnabling" or absolute states such as
  // "Activated", "MotionEnabled".
  const intrinsic_fbs::HardwareModuleState* GetHardwareModuleState() const;

 private:
  explicit HardwareModuleProxy(absl::string_view shared_memory_namespace,
                               absl::string_view module_name);

  std::string shared_memory_namespace_;
  std::string module_name_;
  ReadOnlyMemorySegment<intrinsic_fbs::SegmentInfo> module_info_;
  std::vector<std::string> interface_names_;

  mutable absl::Mutex used_interfaces_mutex_;
  // Interfaces that GetHardwareInterface, or GetMutableHardwareInterface has
  // been called for.
  // Can be mutable because the behavior of the HardwareModuleProxy doesn't
  // depend on the state of used interfaces. It is metadata for inspection by
  // the HardwareModuleManager and the mutex ensures thread safety.
  mutable absl::flat_hash_set<std::string> ABSL_GUARDED_BY(
      used_interfaces_mutex_) used_interfaces_;

  SegmentNameToFileDescriptorMap segment_name_to_file_descriptor_map_;

  HardwareInterfaceHandle<intrinsic_fbs::HardwareModuleState>
      hardware_module_state_;
  MutableHardwareInterfaceHandle<intrinsic_fbs::IconState> icon_state_;
  std::unique_ptr<RemoteTriggerClient> prepare_client_;
  std::unique_ptr<RemoteTriggerClient> activate_client_;
  std::unique_ptr<RemoteTriggerClient> deactivate_client_;
  std::unique_ptr<RemoteTriggerClient> enable_motion_client_;
  std::unique_ptr<RemoteTriggerClient> disable_motion_client_;
  std::unique_ptr<RemoteTriggerClient> clear_faults_client_;
  std::unique_ptr<RemoteTriggerClient> read_status_client_;
  std::unique_ptr<RemoteTriggerClient> apply_command_client_;
  std::unique_ptr<RemoteTriggerClient> restart_client_;
};

// Waits for a hardware module to become available and connects to it.
//
// `shared_memory_namespace`: The namespace used for shared memory
// communication.
// `module_name`: The name of the hardware module to connect to.
// `connection_timeout`: The maximum duration to wait for a connection.
// `expected_control_period`: The expected control period of the hardware
// module. If not set, the control period check is skipped.
absl::StatusOr<intrinsic::icon::HardwareModuleProxy> WaitForHardwareModule(
    absl::string_view shared_memory_namespace, absl::string_view module_name,
    absl::Duration connection_timeout,
    std::optional<intrinsic::Duration> expected_control_period);

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_HAL_HARDWARE_MODULE_PROXY_H_
