#include "third_party/abb_hardware_module/abb_hwm/abb_hardware_module.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "google/protobuf/text_format.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/joint_position_pid_velocity_controller.h"
#include "intrinsic/icon/control/realtime_clock_interface.h"
#include "intrinsic/icon/control/safety/extern/safety_status.fbs.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/flatbuffers/transform_view.h"
#include "intrinsic/icon/hal/hardware_interface_registry.h"
#include "intrinsic/icon/hal/hardware_interface_traits.h"
#include "intrinsic/icon/hal/hardware_module_init_context.h"
#include "intrinsic/icon/hal/hardware_module_registry.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_command_utils.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state_utils.h"
#include "intrinsic/icon/hal/module_config.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/util/status/status_macros.h"
#include "third_party/abb_hardware_module/abb_hwm/abb_config.pb.h"
#include "third_party/abb_hardware_module/abb_hwm/abb_default_configs.h"
#include "third_party/abb_hardware_module/abb_hwm/abb_elog.h"
#include "third_party/abb_hardware_module/abb_hwm/egm_data.h"
#include "third_party/abb_hardware_module/abb_hwm/egm_server.h"
#include "third_party/abb_hardware_module/abb_hwm/rws_client.h"
#include "third_party/abb_hardware_module/abb_hwm/rws_egm_utils.h"

namespace {

using abb_hardware_module::egm_data::EgmHeader;
using abb_hardware_module::egm_data::EgmJoints;
using abb_hardware_module::egm_data::EgmMCIState;
using abb_hardware_module::egm_data::EgmMotorState;
using abb_hardware_module::egm_data::EgmPlanned;
using abb_hardware_module::egm_data::EgmRapidCtrlExecState;
using abb_hardware_module::egm_data::EgmRobot;
using abb_hardware_module::egm_data::EgmSensor;
using abb_hardware_module::egm_data::EgmSpeedRef;
using RobotType = ::intrinsic_proto::icon::RobotType;

static constexpr absl::string_view kIrb1100_4_058_RobotTypeName =
    "IRB 1100-4/0.58";
static constexpr absl::string_view kIrb1300_10_115_RobotTypeName =
    "IRB 1300-10/1.15";

absl::flat_hash_map<RobotType, absl::string_view>
GetRobotTypeToControllerStringMap() {
  return {
      {RobotType::IRB_1100_4_058, kIrb1100_4_058_RobotTypeName},
      {RobotType::IRB_1300_10_115, kIrb1300_10_115_RobotTypeName},
  };
}

absl::flat_hash_map<absl::string_view, absl::string_view>
GetDefaultPIDConfigMap() {
  return {
      {kIrb1100_4_058_RobotTypeName,
       abb_hardware_module::kIrb1100_4_058_DefaultPidConfig},
      {kIrb1300_10_115_RobotTypeName,
       abb_hardware_module::kIrb1300_10_115_DefaultPidConfig},
  };
}

absl::flat_hash_map<absl::string_view, absl::string_view>
GetDefaultEGMConfigMap() {
  return {
      {kIrb1100_4_058_RobotTypeName,
       abb_hardware_module::kDefaultEgmWithPidConfig},
      {kIrb1300_10_115_RobotTypeName,
       abb_hardware_module::kDefaultEgmWithPidConfig},
  };
}

static constexpr double kRateMonitoringTolerance = 0.05;
static constexpr double kRadToDeg = 180.0 / M_PI;
static constexpr double kDegToRad = M_PI / 180.0;
static constexpr int kStepsConstantVelAfterRepeatSequenceNumber = 2;

absl::StatusOr<intrinsic_proto::icon::JointPositionPidVelocityControllerConfig>
GetDefaultJointCollectionPrefilterConfig(absl::string_view robot_type_name) {
  const auto pid_config_map = GetDefaultPIDConfigMap();
  if (const auto it = pid_config_map.find(robot_type_name);
      it != pid_config_map.end()) {
    intrinsic_proto::icon::JointPositionPidVelocityControllerConfig config;
    if (!google::protobuf::TextFormat::ParseFromString(it->second, &config)) {
      return absl::InternalError(absl::StrCat(
          "Failed to parse default prefilter config for robot model ",
          robot_type_name));
    }
    LOG(INFO) << absl::StrCat(
        "Default advanced control configuration found for robot model ",
        robot_type_name);
    return config;
  }
  std::string available_keys;
  for (auto it = pid_config_map.begin(); it != pid_config_map.end(); ++it) {
    absl::StrAppend(&available_keys, it->first, ", ");
  }

  return absl::NotFoundError(absl::StrCat(
      "No default advanced control configuration found for robot model ",
      robot_type_name, ". Available robot models are ", available_keys));
}

absl::StatusOr<::intrinsic_proto::icon::EgmConfig> GetDefaultEGMConfig(
    absl::string_view robot_type_name) {
  const auto egm_config_map = GetDefaultEGMConfigMap();
  if (const auto it = egm_config_map.find(robot_type_name);
      it != egm_config_map.end()) {
    ::intrinsic_proto::icon::EgmConfig config;
    if (!google::protobuf::TextFormat::ParseFromString(it->second, &config)) {
      return absl::InternalError(
          absl::StrCat("Failed to parse default EGM config for robot model ",
                       robot_type_name));
    }
    LOG(INFO) << absl::StrCat(
        "Default EGM configuration found for robot model ", robot_type_name);
    return config;
  }
  std::string available_keys;
  for (auto it = egm_config_map.begin(); it != egm_config_map.end(); ++it) {
    absl::StrAppend(&available_keys, it->first, ", ");
  }

  return absl::NotFoundError(absl::StrCat(
      "No default advanced control configuration found for robot model ",
      robot_type_name, ". Available robot models are ", available_keys));
}

EgmJoints CreateEgmJoints(size_t num_dofs) {
  EgmJoints egm_joints;
  egm_joints.joints.resize(num_dofs);
  return egm_joints;
}

EgmSpeedRef CreateEgmSpeedRef(size_t num_dofs) {
  EgmSpeedRef egm_speed_ref;
  egm_speed_ref.joints = CreateEgmJoints(num_dofs);
  return egm_speed_ref;
}

EgmPlanned CreateEgmPlanned(size_t num_dofs) {
  EgmPlanned egm_planned;
  egm_planned.joints = CreateEgmJoints(num_dofs);
  return egm_planned;
}

static uint32_t seq_num = 0;
EgmHeader CreateEgmHeader() {
  auto now = intrinsic::Clock::Now();
  auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
  auto value = now_ms.time_since_epoch();
  uint32_t tm = value.count();

  EgmHeader header{seq_num++, tm, EgmHeader::MSGTYPE_CORRECTION};
  return header;
}

EgmSensor CreateSensorMessage(size_t num_dofs) {
  EgmSensor egm_sensor{.header = CreateEgmHeader(),
                       .planned = CreateEgmPlanned(num_dofs),
                       .speedRef = CreateEgmSpeedRef(num_dofs)};
  return egm_sensor;
}

::intrinsic::icon::RealtimeStatus EgmRobotHasCorrectlySizedJointData(
    const abb_hardware_module::egm_data::EgmRobot& egm_robot, size_t num_dofs) {
  if (!egm_robot.feedBack.has_value()) {
    return intrinsic::icon::RealtimeStatus(
        absl::StatusCode::kInternal,
        "EgmRobot does not have a feedback message.");
  }
  if (!egm_robot.feedBack->joints.has_value()) {
    return intrinsic::icon::RealtimeStatus(
        absl::StatusCode::kInternal,
        "EgmFeedBack does not have a joints message.");
  }

  if (egm_robot.feedBack->joints->joints.size() != num_dofs) {
    return intrinsic::icon::RealtimeStatus(
        absl::StatusCode::kInternal,
        intrinsic::icon::FixedStrCat<
            intrinsic::icon::RealtimeStatus::kMaxMessageLength>(
            "EgmFeedBack message has ",
            egm_robot.feedBack->joints->joints.size(), " joints, but ",
            num_dofs, " were expected."));
  }
  return intrinsic::icon::OkStatus();
}

::intrinsic::icon::RealtimeStatus EgmRobotHasSequenceNumber(
    const abb_hardware_module::egm_data::EgmRobot& egm_robot) {
  if (!egm_robot.header.has_value()) {
    return intrinsic::icon::RealtimeStatus(
        absl::StatusCode::kInternal,
        "EgmRobot message does not have a header.");
  }
  if (!egm_robot.header->seqno.has_value()) {
    return intrinsic::icon::RealtimeStatus(
        absl::StatusCode::kInternal,
        "EgmHeader does not have a sequence number.");
  }
  return intrinsic::icon::OkStatus();
}

::intrinsic::icon::RealtimeStatus VerifyReadyForEgmMotion(
    const EgmRobot& egm_robot) {
  if (!egm_robot.motorState.has_value()) {
    return ::intrinsic::icon::RealtimeStatus(
        absl::StatusCode::kFailedPrecondition,
        "The EgmRobot message does not have a motorState field! Run "
        "ClearFaults to reinitialize EGM.");
  }

  if (egm_robot.motorState->state != EgmMotorState::MOTORS_ON) {
    return ::intrinsic::icon::RealtimeStatus(
        absl::StatusCode::kFailedPrecondition,
        "Robot motors are not on. Run ClearFaults to recover and/or get more "
        "details.");
  }

  if (!egm_robot.rapidExecState.has_value()) {
    return ::intrinsic::icon::RealtimeStatus(
        absl::StatusCode::kFailedPrecondition,
        "The EgmRobot message does not have a rapidExecState field! Run "
        "ClearFaults to reinitialize EGM.");
  }

  if (egm_robot.rapidExecState->state != EgmRapidCtrlExecState::RAPID_RUNNING) {
    return ::intrinsic::icon::RealtimeStatus(
        absl::StatusCode::kFailedPrecondition,
        "RAPID is not running. Please run ClearFaults to get more error "
        "information.");
  }
  if (!egm_robot.mciState.has_value()) {
    return ::intrinsic::icon::RealtimeStatus(
        absl::StatusCode::kFailedPrecondition,
        "The EgmRobot message does not have a mciState field! Run ClearFaults "
        "to reinitialize EGM.");
  }

  if (egm_robot.mciState->state == EgmMCIState::MCI_ERROR) {
    return ::intrinsic::icon::RealtimeStatus(
        absl::StatusCode::kFailedPrecondition,
        "Motion Correction Interface the ABB Controller has an error status. "
        "Please run ClearFaults to get more error information.");
  }
  return ::intrinsic::icon::OkStatus();
}

constexpr absl::string_view kEgmSettingsRapidSymbolPath =
    "/RAPID/T_ROB1/TRobEGM/settings";

absl::Status ValidateEgmConfig(
    const ::intrinsic_proto::icon::EgmConfig& egm_config) {
  if (egm_config.setup_uc_comm_timeout() < 0) {
    return absl::InvalidArgumentError(
        "setup_uc_comm_timeout must be greater than or equal to 0.");
  }
  if (egm_config.activate_cond_min_max() < 0) {
    return absl::InvalidArgumentError(
        "activate_cond_min_max must be greater than or equal to 0.");
  }
  if (egm_config.activate_lp_filter() < 0 ||
      egm_config.activate_lp_filter() > 100) {
    return absl::InvalidArgumentError(
        "activate_lp_filter must be in the range [0, 100].");
  }
  // We currently only support 4ms sample time
  if (egm_config.activate_sample_time() != 4) {
    return absl::InvalidArgumentError(
        "activate_sample_time has only one legal value: 4ms.");
  }
  if (egm_config.activate_max_speed_deviation() <= 0) {
    return absl::InvalidArgumentError(
        "activate_max_speed_deviation must be greater than 0.");
  }
  if (egm_config.run_cond_time() < 0) {
    return absl::InvalidArgumentError(
        "run_cond_time must be greater than or equal to 0.");
  }
  if (egm_config.run_ramp_in_time() < 0) {
    return absl::InvalidArgumentError(
        "run_ramp_in_time must be greater than or equal to 0.");
  }
  if (egm_config.run_pos_corr_gain() < 0 ||
      egm_config.run_pos_corr_gain() > 20) {
    return absl::InvalidArgumentError(
        "run_pos_corr_gain must be in the range [0, 20].");
  }
  if (egm_config.stop_ramp_out_time() < 0) {
    return absl::InvalidArgumentError(
        "stop_ramp_out_time must be greater than or equal to 0.");
  }

  return absl::OkStatus();
}

}  // namespace

namespace abb_hardware_module {

using ::intrinsic::icon::HardwareInterfaceRegistry;
using ::intrinsic::icon::OkStatus;
using ::intrinsic::icon::RealtimeStatus;
using ::intrinsic_fbs::ButtonStatus;
using ::intrinsic_fbs::JointPositionCommand;
using ::intrinsic_fbs::JointPositionState;
using ::intrinsic_fbs::JointVelocityState;
using ::intrinsic_fbs::ModeOfSafeOperation;
using ::intrinsic_fbs::RequestedBehavior;
using ::intrinsic_fbs::SafetyStatusMessage;

AbbHardwareModule::AbbHardwareModule() = default;

absl::Status AbbHardwareModule::InitInterfaces(
    HardwareInterfaceRegistry& interface_registry) {
  INTR_ASSIGN_OR_RETURN(
      joint_position_command_,
      interface_registry.AdvertiseInterface<JointPositionCommand>(
          "joint_position_command", abb_config_.num_dofs));
  INTR_ASSIGN_OR_RETURN(
      joint_position_state_,
      interface_registry.AdvertiseMutableInterface<JointPositionState>(
          "joint_position_state", abb_config_.num_dofs));
  INTR_ASSIGN_OR_RETURN(
      joint_velocity_state_,
      interface_registry.AdvertiseMutableInterface<JointVelocityState>(
          "joint_velocity_state", abb_config_.num_dofs));

  INTR_ASSIGN_OR_RETURN(
      safety_status_,
      interface_registry.AdvertiseMutableInterface<SafetyStatusMessage>(
          "safety_status",
          /*mode_of_safe_operation=*/ModeOfSafeOperation::AUTOMATIC,
          /*estop_button_status=*/ButtonStatus::UNKNOWN,
          /*enable_button_status=*/ButtonStatus::UNKNOWN,
          /*requested_behavior=*/RequestedBehavior::UNKNOWN));

  return absl::OkStatus();
}

absl::Status AbbHardwareModule::InitRealtimeClock(
    const intrinsic::icon::ModuleConfig& config) {
  intrinsic::icon::RealtimeClockInterface* realtime_clock =
      config.GetRealtimeClock();
  if (realtime_clock != nullptr) {
    return absl::InvalidArgumentError(
        "Realtime clock found in module config, which is not allowed, as "
        "ABBHardwareModule only supports ICON driving the clock.");
  }

  INTR_ASSIGN_OR_RETURN(absl::Duration control_period,
                        config.GetControlPeriod());
  // verify that the control_period is 4ms
  auto control_period_ms = control_period / absl::Milliseconds(1);
  if (control_period_ms != 4) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "ABB Hardware Module currently only supports a control period of 4ms. "
        "ICON is currently driving the clock at %d ms",
        control_period_ms));
  }

  // Store these as class variables for use throughout the module.
  cycle_duration_ = control_period;
  control_frequency_hz_ = 1.0 / absl::ToDoubleSeconds(control_period);

  INTR_RETURN_IF_ERROR(rate_monitor_.Init(
      control_frequency_hz_ * (1 - kRateMonitoringTolerance),
      control_frequency_hz_ * (1 + kRateMonitoringTolerance)));

  return absl::OkStatus();
}

absl::Status AbbHardwareModule::ParseConfig(
    const intrinsic::icon::ModuleConfig& config) {
  INTR_ASSIGN_OR_RETURN(const auto abb_config_proto,
                        config.GetConfig<::intrinsic_proto::icon::AbbConfig>());

  if (!abb_config_proto.has_num_dof()) {
    return absl::InvalidArgumentError(
        "Number of degrees of freedom must be provided.");
  }
  if (abb_config_proto.num_dof() != 6) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Only 6 DOF robots are currently supported. Provided: %d",
        abb_config_proto.num_dof()));
  }

  if (!abb_config_proto.has_robot_controller_ip() ||
      !abb_config_proto.has_egm_port()) {
    return absl::InvalidArgumentError(
        "Robot controller IP and egm port must be provided.");
  }

  if (!abb_config_proto.has_rws_port()) {
    return absl::InvalidArgumentError("RWS port must be provided.");
  }

  int egm_sequence_tolerance = 3;
  if (abb_config_proto.has_egm_sequence_tolerance()) {
    egm_sequence_tolerance = abb_config_proto.egm_sequence_tolerance();
  }

  std::optional<std::string> robot_type = std::nullopt;
  if (abb_config_proto.has_robot_type() &&
      abb_config_proto.robot_type() != RobotType::UNKNOWN) {
    if (!GetRobotTypeToControllerStringMap().contains(
            abb_config_proto.robot_type())) {
      return absl::InternalError(absl::StrFormat("Robot type not found."));
    }
    robot_type =
        GetRobotTypeToControllerStringMap().at(abb_config_proto.robot_type());
  }

  ::intrinsic_proto::icon::EgmConfig egm_config;
  if (abb_config_proto.has_egm_config()) {
    egm_config = abb_config_proto.egm_config();
  } else if (robot_type.has_value()) {
    INTR_ASSIGN_OR_RETURN(egm_config, GetDefaultEGMConfig(robot_type.value()));
  } else {
    return absl::InvalidArgumentError(
        "No EGM config found in the module config. Please eitherprovide an "
        "EgmConfig or a valid robot type for which the default config can be "
        "used.");
  }

  INTR_RETURN_IF_ERROR(ValidateEgmConfig(egm_config));

  std::optional<intrinsic_proto::icon::JointPositionPidVelocityControllerConfig>
      pid_config = std::nullopt;
  switch (abb_config_proto.advanced_control_options_case()) {
    case ::intrinsic_proto::icon::AbbConfig::kNoAdvancedControl: {
      break;
    }
    case ::intrinsic_proto::icon::AbbConfig::kDefaultAdvancedControl: {
      if (!robot_type.has_value()) {
        return absl::InvalidArgumentError(
            "Robot type must be provided to use default advanced control.");
      };
      INTR_ASSIGN_OR_RETURN(
          pid_config,
          GetDefaultJointCollectionPrefilterConfig(robot_type.value()));
      break;
    }
    case ::intrinsic_proto::icon::AbbConfig::kCustomAdvancedControl: {
      pid_config = abb_config_proto.custom_advanced_control()
                       .joint_position_pid_velocity_controller_config();
      break;
    }
    case ::intrinsic_proto::icon::AbbConfig::ADVANCED_CONTROL_OPTIONS_NOT_SET: {
      break;
    }
  }

  AbbConfig abb_config{
      .num_dofs = abb_config_proto.num_dof(),
      .robot_controller_ip = abb_config_proto.robot_controller_ip(),
      .egm_port = abb_config_proto.egm_port(),
      .rws_port = abb_config_proto.rws_port(),
      .egm_sequence_tolerance = egm_sequence_tolerance,
      .robot_type = robot_type,
      .egm_config = egm_config,
      .advanced_control_config = pid_config,
  };

  abb_config_ = std::move(abb_config);

  return absl::OkStatus();
}

absl::Status AbbHardwareModule::InitInternalState() {
  egm_sequence_checker_ =
      std::make_unique<EgmSequenceChecker>(abb_config_.egm_sequence_tolerance);

  return absl::OkStatus();
}

absl::Status AbbHardwareModule::InitAdvancedControl() {
  if (abb_config_.advanced_control_config.has_value()) {
    INTR_ASSIGN_OR_RETURN(
        pid_controller_,
        intrinsic::icon::JointPositionPIDVelocityController::Create(
            *abb_config_.advanced_control_config));
  }
  return absl::OkStatus();
};

absl::Status AbbHardwareModule::PrepareForMotion() {
  LOG(INFO) << "Prepare for motion called.";
  ready_for_motion_ = false;
  if (rws_client_ == nullptr) {
    INTR_ASSIGN_OR_RETURN(
        rws_client_,
        CreateRwsClient(abb_config_.robot_controller_ip, abb_config_.rws_port),
        _.LogError().SetPrepend()
            << "PrepareForMotion: Failed to create RWS client: ");
  }
  if (egm_server_ == nullptr) {
    INTR_ASSIGN_OR_RETURN(
        egm_server_,
        CreateEgmServer(abb_config_.robot_controller_ip, abb_config_.egm_port),
        _.LogError().SetPrepend()
            << "PrepareForMotion: Failed to create EGM server: ");
  }

  INTR_ASSIGN_OR_RETURN(auto robot_type_from_rws, rws_client_->GetRobotType());
  LOG(INFO) << "Robot type from RWS: " << robot_type_from_rws;
  if (abb_config_.robot_type.has_value() &&
      robot_type_from_rws != abb_config_.robot_type.value()) {
    return absl::FailedPreconditionError(absl::StrFormat(
        "PrepareForMotion: The robot type from RWS (%s) does not match the "
        "robot type from the config (%s).",
        robot_type_from_rws, abb_config_.robot_type.value()));
  }

  // Check that AUTO mode is on.
  INTR_ASSIGN_OR_RETURN(
      auto operation_mode, rws_client_->GetOperationMode(),
      _.LogError().SetPrepend()
          << "PrepareForMotion: Failed to get operation mode: ");
  if (operation_mode != AbbOperationMode::kAuto) {
    return absl::FailedPreconditionError(
        "PrepareForMotion: The Robot is not in AUTO mode. Please set the "
        "robot to AUTO mode and then ClearFaults again.");
  }

  // Check if we already have mastership, and if not, request it. This is
  // necessary if mastership has been taken by teach pendant or the controller
  // has restarted after the hardware module was started. NOTE: status-macros
  // don't seem to work with function templates!
  auto we_have_mastership_or_status =
      rws_client_->GetMastershipStatus</*edit_domain=*/true,
                                       /*motion_domain=*/false>();
  if (!we_have_mastership_or_status.ok()) {
    LOG(ERROR) << "Failed to get mastership status. This may indicate a "
                  "problem with the RWS connection.";
    return we_have_mastership_or_status.status();
  }
  if (!(we_have_mastership_or_status.value())) {
    // We did not already have the required mastership, so lets request it.
    auto status = rws_client_->RequestMastership</*edit_domain=*/true,
                                                 /*motion_domain=*/false>();
    if (!status.ok()) {
      LOG(ERROR) << "Failed to request mastership. This may indicate a problem "
                    "with the RWS connection.";
      return status;
    }
  }

  INTR_RETURN_IF_ERROR(rws_client_->StartStateMachine()).LogError().SetPrepend()
      << "PrepareForMotion: Failed StartStateMachine: ";

  INTR_RETURN_IF_ERROR(rws_client_->InitEgm()).LogError().SetPrepend()
      << "PrepareForMotion: Failed InitEgm: ";

  // After we start the state machine, we need to ensure the RAPID program has
  // passed the default initialization of the EGM settings before we write our
  // changes, lest they be overwritten by the default values.
  INTR_RETURN_IF_ERROR(WaitForDefaultEgmSettings()).LogError().SetPrepend()
      << "Failed to get the default EGM settings from the controller: ";
  INTR_RETURN_IF_ERROR(ApplyEgmSettings(abb_config_.egm_config))
          .LogError()
          .SetPrepend()
      << "Failed to apply EGM settings: ";
  INTR_RETURN_IF_ERROR(rws_client_->StartEgmStream()).LogError().SetPrepend()
      << "Failed to start EGM stream: ";

  INTR_RETURN_IF_ERROR(WaitForEgmRobotMessage()).LogError().SetPrepend()
      << "Failed to verify that EGM data was received from robot: ";
  INTR_RETURN_IF_ERROR(WaitForEgmStream()).LogError().SetPrepend()
      << "Failed to verify that EGM stream was started: ";
  ready_for_motion_ = true;
  LOG(INFO) << "Prepare for motion finished.";

  return absl::OkStatus();
}

absl::Status AbbHardwareModule::Init(
    intrinsic::icon::HardwareModuleInitContext& init_context) {
  intrinsic::icon::HardwareInterfaceRegistry& interface_registry =
      init_context.GetInterfaceRegistry();
  const intrinsic::icon::ModuleConfig& config = init_context.GetModuleConfig();

  INTR_RETURN_IF_ERROR(ParseConfig(config));
  INTR_RETURN_IF_ERROR(InitInterfaces(interface_registry));
  INTR_RETURN_IF_ERROR(InitRealtimeClock(config));
  INTR_RETURN_IF_ERROR(InitInternalState());
  INTR_RETURN_IF_ERROR(InitAdvancedControl());

  egm_sensor_ = CreateSensorMessage(abb_config_.num_dofs);
  LOG(INFO) << "Finished initializing the ABB Hardware Module.";
  return absl::OkStatus();
}

absl::Status AbbHardwareModule::ApplyEgmSettings(
    const ::intrinsic_proto::icon::EgmConfig& egm_config) {
  INTR_ASSIGN_OR_RETURN(
      auto current_settings_string,
      rws_client_->GetSymbolValue(kEgmSettingsRapidSymbolPath));
  INTR_ASSIGN_OR_RETURN(auto new_settings_string,
                        rapid::ApplyEgmConfigToEgmSettingsRapidString(
                            current_settings_string, egm_config));
  INTR_RETURN_IF_ERROR(rws_client_->SetSymbolValue(kEgmSettingsRapidSymbolPath,
                                                   new_settings_string));
  return absl::OkStatus();
}

absl::Status AbbHardwareModule::WaitForDefaultEgmSettings() {
  return WaitFor(
      [this]() -> absl::StatusOr<bool> {
        INTR_ASSIGN_OR_RETURN(
            auto current_config_string,
            rws_client_->GetSymbolValue(kEgmSettingsRapidSymbolPath));
        INTR_ASSIGN_OR_RETURN(
            auto current_config,
            rapid::ParseEgmSettingsJsonFromRapidString(current_config_string));
        // section 0 is a bool which flips from false to true on default
        // initialization of the settings struct.
        return current_config[0] == true;
      },
      absl::Seconds(2), absl::Milliseconds(100),
      absl::DeadlineExceededError("Timeout waiting for default EGM settings."));
}

absl::Status AbbHardwareModule::WaitForEgmRobotMessage() {
  return WaitFor(
      [this]() -> absl::StatusOr<bool> {
        return egm_server_->GetDataFromRobot().ok();
      },
      absl::Seconds(2), absl::Milliseconds(100),
      absl::DeadlineExceededError(absl::StrFormat(
          "Timeout waiting for EGM data from robot. Make sure you configured "
          "UCdevice on the ABB controller with the IP of the IPC"
          " and the same port as specified in the ABB Hardware Module config. "
          "The configured port number is currently: %d."
          "Refer to the ABB Hardware Module documentation for details on ABB "
          "controller configuration.",
          abb_config_.egm_port)));
}

absl::Status AbbHardwareModule::WaitForEgmStream() {
  std::optional<uint32_t> last_seq_no;
  return WaitFor(
      [this, &last_seq_no]() -> absl::StatusOr<bool> {
        auto egm_robot_or = egm_server_->GetDataFromRobot();
        if (!egm_robot_or.ok()) {
          return egm_robot_or.status();  // implicitly converts to absl::Status
        }
        if (!EgmRobotHasSequenceNumber(egm_robot_or.value()).ok()) {
          return absl::InvalidArgumentError(
              "EgmRobot message does not have a sequence number.");
        }
        if (!last_seq_no.has_value()) {  // first run
          last_seq_no = egm_robot_or.value().header->seqno.value();
          return false;
        }
        uint32_t current_seq_no = egm_robot_or.value().header->seqno.value();
        if (current_seq_no == last_seq_no.value()) {
          last_seq_no = current_seq_no;
          return false;
        }
        return true;
      },
      absl::Seconds(4), absl::Milliseconds(4),
      absl::DeadlineExceededError(
          "Timeout waiting for EGM stream to be started."));
}

absl::Status AbbHardwareModule::WaitForReadyForEgmMotion() {
  return WaitFor(
      [this]() -> absl::StatusOr<bool> {
        INTR_ASSIGN_OR_RETURN(auto egm_robot, egm_server_->GetDataFromRobot());
        return VerifyReadyForEgmMotion(egm_robot).ok();
      },
      absl::Seconds(1), absl::Milliseconds(4),
      absl::DeadlineExceededError(
          "Timeout waiting for robot to be ready for EGM motion."));
}

RealtimeStatus AbbHardwareModule::Activate() {
  INTRINSIC_RT_LOG(INFO) << "Activating abb hardware module";

  return OkStatus();
}

absl::Status AbbHardwareModule::Prepare() {
  LOG(INFO) << "Prepare on abb hardware module";

  auto status = PrepareForMotion();
  if (!status.ok()) {
    LOG(ERROR) << "PrepareForMotion failed. This is not a fatal error, I will"
                  "try again on next ClearFaults: "
               << status;
  }
  return absl::OkStatus();
}

RealtimeStatus AbbHardwareModule::Deactivate() {
  INTRINSIC_RT_LOG(INFO) << "Deactivating abb hardware module";
  // We don't need to do anything with egm client here.
  return OkStatus();
}

absl::Status AbbHardwareModule::EnableMotion() {
  LOG(INFO) << "Enabling motion on abb hardware module.";

  if (!ready_for_motion_) {
    return absl::FailedPreconditionError(
        "EnableMotion: The module is not ready for motion.");
  }

  if (pid_controller_ != nullptr) {
    pid_controller_->Reset();
    last_position_sensed_ = std::nullopt;
  }

  INTR_RETURN_IF_ERROR(rws_client_->StopEgm()).LogError().SetPrepend()
      << "Failed to stop EGM: ";

  INTR_RETURN_IF_ERROR(rws_client_->StartEgmJoint()).LogError().SetPrepend()
      << "Failed to start EGM motion; ";

  INTR_RETURN_IF_ERROR(WaitForReadyForEgmMotion()).LogError().SetPrepend()
      << "Failed to verify robot is ready for EGM motion: ";

  INTR_RETURN_IF_ERROR(WaitForEgmStream()).LogError().SetPrepend()
      << "Failed to verify EGM stream started: ";

  return absl::OkStatus();
}

RealtimeStatus AbbHardwareModule::Enabled() {
  INTRINSIC_RT_ASSIGN_OR_RETURN(egm_robot_, egm_server_->GetDataFromRobot());
  INTRINSIC_RT_RETURN_IF_ERROR(EgmRobotHasSequenceNumber(egm_robot_));
  egm_sequence_checker_->Init(egm_robot_.header->seqno.value());
  enabled_ = true;
  return OkStatus();
}

RealtimeStatus AbbHardwareModule::Disabled() {
  enabled_ = false;
  return OkStatus();
}

absl::Status AbbHardwareModule::DisableMotion() {
  LOG(INFO) << "Disabling motion on abb hardware module";

  LOG(INFO) << "Disabling motion on abb hardware module";
  INTR_RETURN_IF_ERROR(rws_client_->StopEgm()).LogError().SetPrepend()
      << "DisableMotion: Failed to stop EGM: ";

  INTR_RETURN_IF_ERROR(rws_client_->StartEgmStream()).LogError().SetPrepend()
      << "DisableMotion: Failed to start EGM stream: ";

  INTR_RETURN_IF_ERROR(WaitForEgmStream()).LogError().SetPrepend()
      << "DisableMotion: Failed to verify EGM stream started: ";
  return absl::OkStatus();
}

absl::Status AbbHardwareModule::ClearFaults() {
  LOG(INFO) << "Clearing faults on abb hardware module";

  if (rws_client_ != nullptr) {
    INTR_ASSIGN_OR_RETURN(
        AbbControllerState state, rws_client_->GetControllerState(),
        _.LogError().SetCode(absl::StatusCode::kFailedPrecondition).SetPrepend()
            << "ClearFaults: Failed to get controller state: ");
    if (state != AbbControllerState::kMotorOn) {
      INTR_ASSIGN_OR_RETURN(std::vector<AbbEventLogEntry> elogs,
                            rws_client_->GetElogs());
      if (!elogs.empty()) {
        last_elog_ = RealtimeStatus(absl::StatusCode::kFailedPrecondition,
                                    elogs.back().ToString());
      }
    }
  }

  INTR_RETURN_IF_ERROR(PrepareForMotion()).LogError().SetPrepend()
      << "Failed to prepare for motion: ";

  return absl::OkStatus();
}

absl::Status AbbHardwareModule::Shutdown() {
  INTR_RETURN_IF_ERROR(rws_client_->StopExecution());
  INTR_RETURN_IF_ERROR(rws_client_->ReleaseMastership());
  egm_server_->Stop();
  return absl::OkStatus();
}

RealtimeStatus AbbHardwareModule::ApplyCommand() {
  RealtimeStatus rate_monitor_status = rate_monitor_.Tick();
  if (!rate_monitor_status.ok()) {
    INTRINSIC_RT_LOG_THROTTLED(WARNING) << rate_monitor_status.ToString();
  }

  if (!ready_for_motion_) {
    return intrinsic::icon::RealtimeStatus(
        absl::StatusCode::kFailedPrecondition,
        "ApplyCommand: The module is not ready for motion. Run ClearFaults for "
        "recovery and/or more error information. ");
  }

  // Check for error signals in the last egm message.
  INTRINSIC_RT_RETURN_IF_ERROR(VerifyReadyForEgmMotion(egm_robot_));

  const intrinsic::eigenmath::VectorNd position_sensed =
      intrinsic_fbs::ViewAs<intrinsic::eigenmath::VectorNd>(
          joint_position_state_->position());
  const intrinsic::eigenmath::VectorNd velocity_sensed =
      intrinsic_fbs::ViewAs<intrinsic::eigenmath::VectorNd>(
          joint_velocity_state_->velocity());

  intrinsic::eigenmath::VectorNd v_command =
      intrinsic::eigenmath::VectorNd::Zero(abb_config_.num_dofs);

  if (pid_controller_ != nullptr) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        v_command,
        pid_controller_
            ->CalculateSetpoints(/*position_desired=*/
                                 intrinsic_fbs::ViewAs<
                                     intrinsic::eigenmath::VectorNd>(
                                     joint_position_command_->position()),
                                 /*velocity_feedforward=*/
                                 intrinsic_fbs::ViewAs<
                                     intrinsic::eigenmath::VectorNd>(
                                     joint_position_command_
                                         ->velocity_feedforward()),
                                 /*position_state=*/
                                 position_sensed,
                                 /*velocity_state=*/
                                 velocity_sensed));
  } else {
    for (int i = 0; i < abb_config_.num_dofs; ++i) {
      v_command[i] = joint_position_command_->velocity_feedforward()->Get(i);
    }
  }

  for (int i = 0; i < abb_config_.num_dofs; ++i) {
    egm_sensor_.planned->joints->joints[i] =
        kRadToDeg * joint_position_command_->position()->Get(i);
    egm_sensor_.speedRef->joints->joints[i] = kRadToDeg * v_command[i];
  }
  egm_server_->SetDataToRobot(egm_sensor_);

  return OkStatus();
}

RealtimeStatus AbbHardwareModule::ReadStatus() {
  if (!ready_for_motion_) {
    if (last_elog_.has_value()) {
      return last_elog_.value();
    }
    return intrinsic::icon::RealtimeStatus(
        absl::StatusCode::kFailedPrecondition,
        "ReadStatus: The module is not ready for motion. Run ClearFaults for "
        "recovery information.");
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(egm_robot_, egm_server_->GetDataFromRobot());
  INTRINSIC_RT_RETURN_IF_ERROR(
      EgmRobotHasCorrectlySizedJointData(egm_robot_, abb_config_.num_dofs));

  if (enabled_) {
    INTRINSIC_RT_RETURN_IF_ERROR(EgmRobotHasSequenceNumber(egm_robot_));
    RealtimeStatus status = egm_sequence_checker_->CheckSequenceNumber(
        egm_robot_.header->seqno.value());
    if (!status.ok()) {
      return intrinsic::icon::RealtimeStatus(
          absl::StatusCode::kInternal,
          intrinsic::icon::FixedStrCat<
              intrinsic::icon::RealtimeStatus::kMaxMessageLength>(
              "Sequence number error: ", status.message()));
    }
  }

  for (int i = 0; i < abb_config_.num_dofs; ++i) {
    joint_position_state_->mutable_position()->Mutate(
        i, kDegToRad * egm_robot_.feedBack->joints->joints[i]);
  }

  const intrinsic::eigenmath::VectorNd position_sensed =
      intrinsic_fbs::ViewAs<intrinsic::eigenmath::VectorNd>(
          joint_position_state_->position());

  // The EGM interface intermittently will not provide a new package and
  // sometimes skip. When this happens we get more reliable velocity estimate by
  // just assuming constant velocity for
  // `kStepsConstantVelAfterRepeatSequenceNumber` cycles.
  if (last_position_sensed_.has_value()) {
    if (egm_sequence_checker_->IsLastSequenceNumberRepeated()) {
      iterations_till_update_velocity_ =
          kStepsConstantVelAfterRepeatSequenceNumber;
    } else if (iterations_till_update_velocity_ > 0) {
      iterations_till_update_velocity_--;
    } else {
      const auto velocity = control_frequency_hz_ *
                            (position_sensed - last_position_sensed_.value());
      for (int i = 0; i < abb_config_.num_dofs; ++i) {
        joint_velocity_state_->mutable_velocity()->Mutate(i, velocity[i]);
      }
    }
  }

  last_position_sensed_ = intrinsic_fbs::ViewAs<intrinsic::eigenmath::VectorNd>(
      joint_position_state_->position());

  if (!rws_client_->IsConnected()) {
    return intrinsic::icon::RealtimeStatus(
        absl::StatusCode::kUnavailable,
        "Lost network connection to ABB "
        "Controller! Verify network connection "
        "and ClearFaults.");
  }

  return OkStatus();
}

}  // namespace abb_hardware_module

// Register the interfaces we use.
namespace intrinsic::icon {
namespace hardware_interface_traits {
INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::JointPositionCommand,
                                 intrinsic_fbs::BuildJointPositionCommand,
                                 "intrinsic_fbs.JointPositionCommand")

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::JointPositionState,
                                 intrinsic_fbs::BuildJointPositionState,
                                 "intrinsic_fbs.JointPositionState")

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::JointVelocityState,
                                 intrinsic_fbs::BuildJointVelocityState,
                                 "intrinsic_fbs.JointVelocityState")

}  // namespace hardware_interface_traits
}  // namespace intrinsic::icon

REGISTER_HARDWARE_MODULE(abb_hardware_module::AbbHardwareModule)
