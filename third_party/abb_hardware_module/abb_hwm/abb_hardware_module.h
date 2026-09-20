#ifndef ABB_HARDWARE_MODULE_ABB_HWM_ABB_HARDWARE_MODULE_H_
#define ABB_HARDWARE_MODULE_ABB_HWM_ABB_HARDWARE_MODULE_H_

#include <atomic>
#include <memory>
#include <optional>
#include <string>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/joint_position_pid_velocity_controller.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/control/safety/safety_messages_utils.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_interface_registry.h"
#include "intrinsic/icon/hal/hardware_interface_traits.h"
#include "intrinsic/icon/hal/hardware_module_init_context.h"
#include "intrinsic/icon/hal/hardware_module_interface.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/hal/module_config.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "third_party/abb_hardware_module/abb_hwm/abb_config.pb.h"
#include "third_party/abb_hardware_module/abb_hwm/egm_data.h"
#include "third_party/abb_hardware_module/abb_hwm/egm_server.h"
#include "third_party/abb_hardware_module/abb_hwm/rws_client.h"
#include "third_party/abb_hardware_module/abb_hwm/rws_egm_utils.h"
#include "third_party/abb_hardware_module/utils/rate_monitor.h"

namespace abb_hardware_module {

// Default frequency of the hardware module.
static constexpr int kDefaultFrequency = 250;

class AbbHardwareModule final
    : public intrinsic::icon::HardwareModuleInterface {
 public:
  // Configuration parameters for the ABB Hardware Module. These are resultant
  // from parsing the proto config of the module and adding default values where
  // absent from the proto config.
  struct AbbConfig {
    int num_dofs;
    std::string robot_controller_ip;
    int egm_port;
    int rws_port;
    int egm_sequence_tolerance;
    std::optional<std::string> robot_type;
    ::intrinsic_proto::icon::EgmConfig egm_config;
    std::optional<
        intrinsic_proto::icon::JointPositionPidVelocityControllerConfig>
        advanced_control_config;
  };

  explicit AbbHardwareModule();

  absl::Status Init(intrinsic::icon::HardwareModuleInitContext& init_context)
      override INTRINSIC_NON_REALTIME_ONLY;

  intrinsic::icon::RealtimeStatus Activate() override
      INTRINSIC_CHECK_REALTIME_SAFE;

  intrinsic::icon::RealtimeStatus Deactivate() override
      INTRINSIC_CHECK_REALTIME_SAFE;

  intrinsic::icon::RealtimeStatus Enabled() override
      INTRINSIC_CHECK_REALTIME_SAFE;

  intrinsic::icon::RealtimeStatus Disabled() override
      INTRINSIC_CHECK_REALTIME_SAFE;

  absl::Status Prepare() override INTRINSIC_NON_REALTIME_ONLY;

  absl::Status EnableMotion() override INTRINSIC_NON_REALTIME_ONLY;

  absl::Status DisableMotion() override INTRINSIC_NON_REALTIME_ONLY;

  absl::Status ClearFaults() override INTRINSIC_NON_REALTIME_ONLY;

  absl::Status Shutdown() override INTRINSIC_NON_REALTIME_ONLY;

  intrinsic::icon::RealtimeStatus ReadStatus() override
      INTRINSIC_CHECK_REALTIME_SAFE;

  intrinsic::icon::RealtimeStatus ApplyCommand() override
      INTRINSIC_CHECK_REALTIME_SAFE;

 private:
  absl::Duration cycle_duration_;
  double control_frequency_hz_;

  std::unique_ptr<EgmServer> egm_server_;
  std::unique_ptr<RwsClient> rws_client_;
  RateMonitor rate_monitor_;
  std::unique_ptr<EgmSequenceChecker> egm_sequence_checker_;

  egm_data::EgmRobot egm_robot_;
  egm_data::EgmSensor egm_sensor_;
  std::atomic<bool> ready_for_motion_;
  std::atomic<bool> enabled_;
  std::optional<intrinsic::icon::RealtimeStatus> last_elog_;

  AbbConfig abb_config_;

  std::unique_ptr<intrinsic::icon::JointPositionPIDVelocityController>
      pid_controller_ = nullptr;
  std::optional<intrinsic::eigenmath::VectorNd> last_position_sensed_;

  absl::Status ApplyEgmSettings(const ::intrinsic_proto::icon::EgmConfig&
                                    egm_config) INTRINSIC_NON_REALTIME_ONLY;

  absl::Status WaitForDefaultEgmSettings() INTRINSIC_NON_REALTIME_ONLY;
  absl::Status WaitForEgmRobotMessage() INTRINSIC_NON_REALTIME_ONLY;
  absl::Status WaitForEgmStream() INTRINSIC_NON_REALTIME_ONLY;
  absl::Status WaitForStateMachineState(rapid::StateMachineState state)
      INTRINSIC_NON_REALTIME_ONLY;
  absl::Status WaitForReadyForEgmMotion() INTRINSIC_NON_REALTIME_ONLY;

  absl::Status InitInterfaces(
      intrinsic::icon::HardwareInterfaceRegistry& interface_registry)
      INTRINSIC_NON_REALTIME_ONLY;
  absl::Status InitRealtimeClock(const intrinsic::icon::ModuleConfig& config)
      INTRINSIC_NON_REALTIME_ONLY;
  absl::Status ParseConfig(const intrinsic::icon::ModuleConfig& config)
      INTRINSIC_NON_REALTIME_ONLY;
  absl::Status InitInternalState() INTRINSIC_NON_REALTIME_ONLY;
  absl::Status InitAdvancedControl() INTRINSIC_NON_REALTIME_ONLY;
  absl::Status PrepareForMotion() INTRINSIC_NON_REALTIME_ONLY;

  // HAL interface handles.
  intrinsic::icon::MutableHardwareInterfaceHandle<
      intrinsic_fbs::JointPositionState>
      joint_position_state_;
  intrinsic::icon::HardwareInterfaceHandle<intrinsic_fbs::JointPositionCommand>
      joint_position_command_;
  intrinsic::icon::MutableHardwareInterfaceHandle<
      intrinsic_fbs::JointVelocityState>
      joint_velocity_state_;
  intrinsic::icon::MutableHardwareInterfaceHandle<
      intrinsic_fbs::SafetyStatusMessage>
      safety_status_;
  int iterations_till_update_velocity_;
};

}  // namespace abb_hardware_module

// Register the additional interfaces we use.
namespace intrinsic::icon {
namespace hardware_interface_traits {
INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::SafetyStatusMessage,
                                 intrinsic_fbs::BuildSafetyStatusMessage,
                                 "intrinsic_fbs.SafetyStatus")
}  // namespace hardware_interface_traits
}  // namespace intrinsic::icon

#endif  // ABB_HARDWARE_MODULE_ABB_HWM_ABB_HARDWARE_MODULE_H_
