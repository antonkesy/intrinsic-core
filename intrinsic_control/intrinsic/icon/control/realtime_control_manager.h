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

#ifndef INTRINSIC_ICON_CONTROL_REALTIME_CONTROL_MANAGER_H_
#define INTRINSIC_ICON_CONTROL_REALTIME_CONTROL_MANAGER_H_

#include <atomic>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include "absl/base/nullability.h"
#include "absl/container/fixed_array.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "grpcpp/channel.h"
#include "grpcpp/server.h"
#include "grpcpp/support/channel_arguments.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.grpc.pb.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/control/context.h"
#include "intrinsic/icon/control/initialized_async_buffer.h"
#include "intrinsic/icon/control/logging_mode.h"
#include "intrinsic/icon/control/parts/proto/v1/realtime_part_config.pb.h"
#include "intrinsic/icon/control/parts/realtime_log_context.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/realtime_function_bridge.h"
#include "intrinsic/icon/control/realtime_operational_state_bridge.h"
#include "intrinsic/icon/control/realtime_operational_status.h"
#include "intrinsic/icon/control/realtime_part_manager.h"
#include "intrinsic/icon/control/realtime_session_channels.h"
#include "intrinsic/icon/control/realtime_state_manager_interface.h"
#include "intrinsic/icon/control/robot_connection_rtcl_controller.h"
#include "intrinsic/icon/control/rtcl_action_instance.h"
#include "intrinsic/icon/control/rtcl_controller_bridge.h"
#include "intrinsic/icon/control/rtcl_initialization.h"
#include "intrinsic/icon/control/rtcl_safety_message_handler.h"
#include "intrinsic/icon/control/rtcl_session_manager.h"
#include "intrinsic/icon/control/safety/extern/safety_status.fbs.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_module_manager.h"
#include "intrinsic/icon/proto/v1/jogging_service.grpc.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/server/config/realtime_control_config.pb.h"
#include "intrinsic/icon/server/custom_action_plugin_loader.h"
#include "intrinsic/icon/server/icon_api_service.h"
#include "intrinsic/icon/server/service.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/async_buffer.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/world/objects/object_world_client.h"

namespace intrinsic::icon {

// Default maximum number of concurrent read-only ICON sessions.
constexpr size_t kMaxNumberOfMonitoringSessions = 10;

using RealtimePartConfigMap =
    absl::flat_hash_map<std::string, RealtimePartConfig>;
using RealtimePartConfigProtoMap =
    absl::flat_hash_map<std::string, intrinsic_proto::v1::RealtimePartConfig>;

// This the main entrypoint of ICON's real-time control layer (RTCL).
// It is used within the ICON MainLoop to call update for all sessions.
// It creates the ICON server, RtclControllerBridge and initializes parts and
// default actions.
// It delegates Part management to a RealtimePartManager, but handles Action /
// Session interactions directly.
class RealtimeControlManager {
 public:
  static absl::StatusOr<std::unique_ptr<RealtimeControlManager>> Create(
      const ::intrinsic_proto::icon::RealtimeControlConfig& config,
      const ::intrinsic_proto::icon::v1::ServerConfig& server_config,
      const Context& context,
      RealtimeStateManagerInterface& realtime_state_manager,
      LoggingMode default_logging_mode);
  // Create an RealtimeControlManager with a RtclControllerBridge that controls
  // the Parts `parts_and_configs`.
  //
  // RealtimeControlManager implements the ICON API and GPIO services. Use the
  // corresponding getters to access these services and register them with a
  // gRPC server.
  //
  // RealtimeControlManager uses `realtime_state_manager` to control and read
  // the state of the hardware abstraction components, e.g. hardware modules (it
  // doesn't have any direct dependency on hardware modules for controlling the
  // state).
  //
  // The RealtimeControlManager supports only hardware modules as the source for
  // the safety status of the system. The parameter `safety_module_name` is used
  // to find the optional device or hardware module exporting a
  // `safety_status_message`. An empty `safety_module_name` means that no safety
  // device/module should be configured.
  //
  // The `default_logging_mode` is the logging mode after creation. It can be
  // changed at runtime.
  //
  // The parameter `safety_message_handler_config` is used to configure the
  // SafetyMessageHandler. It can multiplex SafetyStatusMessages and inject
  // safety behaviors. An empty config is valid.
  static absl::StatusOr<std::unique_ptr<RealtimeControlManager>> Create(
      RtclPartsAndConfigs parts_and_configs, const Context& context,
      RealtimeStateManagerInterface& realtime_state_manager,
      double control_frequency_hz, LoggingMode default_logging_mode,
      intrinsic_proto::icon::SafetyMessageHandlerConfig
          safety_message_handler_config);

  ~RealtimeControlManager();

  // Returns an InProcessChannel connected to the application layer Grpc server,
  // or nullptr if that server has not been set up yet.
  std::shared_ptr<grpc::Channel> InProcChannel(
      const grpc::ChannelArguments& channel_args) INTRINSIC_NON_REALTIME_ONLY;

  // Shuts down operational state commands and stops all sessions with error
  // `status`. Does not block.
  // Thread unsafe; only call from real-time thread.
  void Shutdown(RealtimeStatus status);

  // Runs at every cycle at the beginning of the control phase. Afterwards, all
  // command buffers hold new values, ready for the rest of the framework to
  // send them to the hardware modules.
  RealtimeStatus Update(Time cycle_time) INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns a pointer to the current active Action for the Part with the given
  // name. Intended for use in unit tests.
  // Returns NotFoundError if no Part with that Name is available.
  // Returns nullptr if the Part is not enabled, or if no Action is active for
  // that Part (in which case we run the default Action).
  RealtimeStatusOr<const RtclActionInstance*> GetCurrentActionForPartTestOnly(
      absl::string_view part_name) const INTRINSIC_NON_REALTIME_ONLY;

  bool PartStatusBufferEmptyTestOnly() const INTRINSIC_NON_REALTIME_ONLY;

  IconApiService* absl_nullable IconService() { return icon_service_.get(); }
  intrinsic_proto::gpio::v1::GPIOService::Service* absl_nullable GpioService() {
    return gpio_service_.get();
  }
  intrinsic_proto::icon::v1::JoggingService::Service* absl_nullable
  JoggingService() {
    return jogging_service_.get();
  }

 private:
  struct OperationalStateHandle {
    RealtimeFunctionBridge<RealtimeStatus(
        OperationalStateCommand cmd)>::RealtimeFunctionHandle handle;
    // The command ID (if any) from RealtimePartManager for the request that
    // corresponds to `handle` above.
    std::optional<HardwareModuleManager::CommandId> command_id;
  };

  explicit RealtimeControlManager(
      RtclPartsAndConfigs parts_and_configs,
      RealtimeStateManagerInterface& realtime_state_manager,
      LoggingMode default_logging_mode,
      intrinsic_proto::icon::SafetyMessageHandlerConfig
          safety_message_handler_config);

  // Helper function used by Create() to set up the RealtimeControlManager.
  absl::Status Initialize(const Context& context, double control_frequency_hz)
      INTRINSIC_NON_REALTIME_ONLY;

  // Sets the session status for all sessions in session_managers_.
  void SetSessionManagerErrorCleanupNotification(RealtimeStatus status);

  // Performs operations necessary to prepare for UpdateSessions.
  RealtimeStatusOr<AggregatedRobotStatus> PreUpdateSessions(Time cycle_time)
      INTRINSIC_CHECK_REALTIME_SAFE;
  RealtimeStatus HandleOperationalStatus() INTRINSIC_CHECK_REALTIME_SAFE;
  RealtimeStatus UpdateSessions(Time cycle_time,
                                const AggregatedRobotStatus& robot_status)
      INTRINSIC_CHECK_REALTIME_SAFE;
  RealtimeStatus HandleStartSession() INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns true if the enable button is engaged while in T1.
  // The result will be true only for the 1st cycle after switching to engaged.
  bool IsExpectedToAutomaticallyEnable();

  // Returns true if the enable button is disengaged while in T1.
  // The result will be true only for the 1st cycle after switching to
  // disengaged.
  bool IsExpectedToAutomaticallyDisable();

  // Updates the OperationalStatus as reported to ICON clients, based on the
  // states of all known Parts.
  //
  // `ignore_active_command` is a special-case flag. Use when calling this
  // function from within an active OperationalStatus command.
  void UpdateOperationalStatus(bool ignore_active_command = false)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Checks the given `request_state`, which originates from `cmd`, for
  // completion or errors. In case the request is done, writes the result
  // into `return_value` and returns the result state for the
  // RealtimeFunctionBridge.
  RealtimeFunctionState CheckCommandState(
      RealtimeStateManagerInterface::RequestState request_state,
      OperationalStateCommand cmd,
      RealtimeStatus& return_value) INTRINSIC_CHECK_REALTIME_SAFE;

  // Reads the safety_status_message flatbuffer and notifies part_manager.
  // Closes all Sessions if ModeOfSafeOperation changed since the previous tick.
  void HandleSafetyStatus() INTRINSIC_CHECK_REALTIME_SAFE;

  // Starts additional gRPC servers for GPIO and Joint/Cartesian Jogging.
  // Note that the ICON server may not support jogging or GPIO for several
  // reasons. We still start the servers in this case, but they return errors
  // when called.
  // Returns absl::OkStatus if both the GPIO and jogging services were started
  // successfully.
  // Returns an error status otherwise.
  absl::Status StartGpioAndJoggingServers(const Context& context);

  // Calculates how many control sessions are in use.
  size_t CalculateUsedControlSessions() const INTRINSIC_CHECK_REALTIME_SAFE;
  // Calculates how many monitoring sessions are in use.
  size_t CalculateUsedMonitoringSessions() const INTRINSIC_CHECK_REALTIME_SAFE;

  // TODO(b/175654117): Consider initializing at construction once the config is
  // available. ATM, the environment containing the robot name is only available
  // during initialize().
  ::intrinsic_proto::icon::v1::ServerConfig server_config_;

  RealtimeFunctionBridge<RealtimeStatus(
      SessionId session_id, absl::flat_hash_set<size_t> part_indices,
      RealtimeSessionChannels* channels, RealtimeLogContext log_context)>
      start_realtime_session_bridge_;
  std::optional<RealtimeFunctionBridge<RealtimeStatus(
      SessionId session_id, absl::flat_hash_set<size_t> part_indices,
      RealtimeSessionChannels* channels,
      RealtimeLogContext log_context)>::RealtimeFunctionHandle>
      current_start_realtime_session_handle_ = std::nullopt;

  AsyncBuffer<RealtimeOperationalStateBridge::OperationalCellControlStatus>
      operational_cell_control_status_buffer_;
  RealtimeFunctionBridge<RealtimeStatus(OperationalStateCommand cmd)>
      operational_state_command_bridge_;

  // FunctionData object to trigger the enable operational state command from
  // inside the realtime thread.
  RealtimeFunctionBridge<RealtimeStatus(
      OperationalStateCommand cmd)>::FunctionData auto_enable_function_data_ = {
      .args = {OperationalStateCommand::kEnable}};

  // FunctionData object to trigger the disable operational state command from
  // inside the realtime thread.
  RealtimeFunctionBridge<RealtimeStatus(
      OperationalStateCommand cmd)>::FunctionData auto_disable_function_data_ =
      {.args = {OperationalStateCommand::kDisable}};

  std::optional<OperationalStateHandle>
      current_operational_state_command_handle_ = std::nullopt;
  // Auto-enable on server start is allowed before operational state commands
  // come in, and when the mode of safe operation is automated.
  bool operational_state_command_ever_set_ = false;

  // Reference to the RealtimeStateManager to use. Not owned by this class and
  // must outlive this class.
  RealtimeStateManagerInterface& realtime_state_manager_;

  // Order is important here:
  // * rtcl_controller_bridge_ holds a reference to
  //   start_realtime_session_bridge_ above, and to a PartStatus buffeer owned
  //   by part_manager_
  // * robot_connection_ holds references to rtcl_controller_bridge_ and
  //   operational_state_
  // * icon_service_ holds a reference to robot_connection_
  // * session_managers_ holds a reference to part_manager_
  // * session_managers_ needs to be constructed before
  //   realtime_part_configs_by_part_name_
  //
  // They must be declared after the owners of their references/pointers so that
  // they get initialized and destroyed without violating any lifetime
  // requirements.
  RealtimeOperationalStateBridge operational_state_;
  std::unique_ptr<RealtimePartManager> part_manager_;
  std::atomic<double> speed_override_buffer_ = 0;
  static_assert(decltype(speed_override_buffer_)::is_always_lock_free);
  InitializedAsyncBuffer<LoggingMode> logging_mode_buffer_ =
      InitializedAsyncBuffer<LoggingMode>{LoggingMode::kThrottled};
  std::unique_ptr<RtclControllerBridge> rtcl_controller_bridge_;
  std::unique_ptr<RobotConnectionRtclController> robot_connection_;
  std::unique_ptr<IconApiService> icon_service_;
  // in-process only gRPC server for gpio_service_ and jogging_service_ to use.
  // This references `icon_service_`.
  std::unique_ptr<grpc::Server> icon_in_process_server_;
  std::unique_ptr<intrinsic_proto::gpio::v1::GPIOService::Service>
      gpio_service_;
  std::unique_ptr<intrinsic_proto::icon::v1::JoggingService::Service>
      jogging_service_;
  RtclPartsAndConfigs parts_and_configs_;
  // Number of sessions allocated for monitoring sessions at the end of the
  // `session_managers_` array.
  absl::FixedArray<RtclSessionManager> session_managers_;

  PubSub pub_sub_;
  std::atomic_bool shutdown_publishing_thread_ = false;
  // Publishes status information when available and saves runtime settings.
  Thread publishing_thread_;
  // Used to initialize the SafSafetyMessageHandler during Initialize.
  const intrinsic_proto::icon::SafetyMessageHandlerConfig
      safety_message_handler_config_;

  // We need to hold onto the CustomActionPluginLoader because it unloads the
  // dlopen'ed plugins when destructed.
  CustomActionPluginLoader plugins_;

  // Multiplexes SafetyStatusMessages. Potentially injects a requested safety
  // behavior.
  std::unique_ptr<SafetyMessageHandler> safety_message_handler_;

  // Remembers the latest ModeOfSafeOperation in order to react to a change.
  intrinsic_fbs::ModeOfSafeOperation latest_mode_of_safe_operation_ =
      intrinsic_fbs::ModeOfSafeOperation::UNKNOWN;
  // Remembers the latest status of the enable button in order to capture the
  // moment of change.
  intrinsic_fbs::ButtonStatus latest_enable_button_status_ =
      intrinsic_fbs::ButtonStatus::UNKNOWN;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_REALTIME_CONTROL_MANAGER_H_
